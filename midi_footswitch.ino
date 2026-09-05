// ESP32 MIDI Looper Engine v49 - Octave transpose (-12/+12) + Quantize (live MIDI Clock) + Presets + Waterfall Web UI
#include <HardwareSerial.h>
#include <MIDI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#define LOOPER_PIN 18
#define LED_PIN 2
#define MIDICHANNEL 12
#define EXPPIN1 34
#define EXPPIN2 35
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PRESET_COUNT 5
HardwareSerial MIDI_Serial(2);
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_Serial, MIDI);
const int pwmFreq = 5000;
const int pwmRes = 8;
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
unsigned long lastBleUpdate = 0;
const unsigned long BLE_INTERVAL = 30;

// --- ПРЕСЕТЫ (save/load лупов через LittleFS) ---
bool needSendPresetStatus = false;
unsigned long connectStatusTime = 0;
bool visualDumpActive = false;
int visualDumpIndex = 0;
unsigned long lastVisualDumpTime = 0;
unsigned long visualDumpLastPosSend = 0;
int visualDumpFinishStage = 0;
unsigned long visualDumpFinishTime = 0;
const unsigned long VISUAL_DUMP_EVENT_INTERVAL = 5;
const unsigned long VISUAL_DUMP_POS_INTERVAL   = 80;
int pendingLoadSlot = -1;

// --- КВАНТИЗАЦИЯ ---
enum QuantTarget { QUANT_START = 0, QUANT_END = 1, QUANT_BOTH = 2 };
int quantGrid = 0;
int quantStrength = 100;
int quantTarget = QUANT_BOTH;
unsigned long loopBeatDuration = 545; // всегда синхронизируется с реальным MIDI Clock в handleClock()
const int MIN_NOTE_LEN_MS = 20;

uint16_t activeNoteStartTs[17][128];

// ДВИЖОК ЛУПЕРА
enum LooperState { STATE_IDLE, STATE_RECORDING, STATE_PLAYING, STATE_OVERDUB };
LooperState looperState = STATE_IDLE;
volatile bool pendingStart = false;
volatile bool pendingStop = false;  
volatile unsigned long lastBeatTime = 0;  
volatile unsigned long lastClockTime = 0;
volatile int beatInBar = 0;       
volatile int tickInBeat = 0;      
unsigned long beatDuration = 545;
struct MidiEvent {
  uint16_t timestamp;
  byte type, d1, d2, ch, layer;
  bool played;
};
MidiEvent events[5000];
int eventCount = 0;         
byte currentLayer = 0;      
byte ccMaxLayer[17][128];   
unsigned long loopLen = 0, recStart = 0, playStart = 0, btnTime = 0;
bool ignoreRel = false;
unsigned long lastSentNoteTime[16][128], lastSentCCTime[16][128], lastSentATTime[16], ccTakeover[17][128];
void sendBleNotify(String text) {
  if (deviceConnected && pCharacteristic) {
    pCharacteristic->setValue(text.c_str());
    pCharacteristic->notify();
  }
}
String stateName() {
  switch (looperState) {
    case STATE_RECORDING: return "RECORDING";
    case STATE_PLAYING:   return "PLAYING";
    case STATE_OVERDUB:   return "OVERDUB";
    default:              return "IDLE";
  }
}
void sendBeatInfo() {
  sendBleNotify("/beatinfo " + String(loopBeatDuration));
}
void resetActiveNoteTracking() {
  for (int i = 0; i < 17; i++) for (int j = 0; j < 128; j++) activeNoteStartTs[i][j] = 0xFFFF;
}
void resetLooper() {
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) {
      MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
    }
  }

  looperState = STATE_IDLE;
  eventCount = 0;
  currentLayer = 0;
  loopLen = 0;
  recStart = 0;
  playStart = 0;
  pendingStart = false;
  pendingStop = false;
  pendingLoadSlot = -1;
  resetActiveNoteTracking();

  sendBleNotify("/reset 1");
}
void halfLoop() {
  if (loopLen < 500 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
  }
  loopLen /= 2;
  int newCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].timestamp < loopLen) {
      events[newCount++] = events[i];
    }
  }
  eventCount = newCount;
}
void doubleLoop() {
  if (loopLen == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (loopLen * 2 > 65000) return;
  int origCount = eventCount;
  unsigned long oldLen = loopLen;
  for (int i = 0; i < origCount; i++) {
    if (eventCount < 5000) {
      MidiEvent newEv = events[i];
      newEv.timestamp = (uint16_t)(events[i].timestamp + oldLen);
      newEv.played = false;
      events[eventCount++] = newEv;
    }
  }
  loopLen *= 2;
}

// --- ТРАНСПОНИРОВАНИЕ ЛУПА НА ОКТАВУ (-12/+12) ---
void transposeLoop(int semitones) {
  if (loopLen == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (semitones == 0) return;

  // Гасим ноты со старой высотой перед сдвигом
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90 || events[i].type == 0x80) {
      int newPitch = (int)events[i].d1 + semitones;
      if (newPitch < 0) newPitch = 0;
      if (newPitch > 127) newPitch = 127;
      events[i].d1 = (byte)newPitch;
    }
  }

  // Пересобираем визуализацию (тот же неблокирующий дамп, что при /load и /qapply)
  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

// Длительность шага текущей сетки квантования в мс (0, если квант выключен)
float quantUnitMs() {
  if (quantGrid == 0 || loopBeatDuration == 0) return 0.0f;
  switch (quantGrid) {
    case 1: return (float)loopBeatDuration / 8.0f;  // Q32
    case 2: return (float)loopBeatDuration / 4.0f;  // Q16
    case 3: return (float)loopBeatDuration / 2.0f;  // Q8
    case 4: return (float)loopBeatDuration;         // Q4
    default: return 0.0f;
  }
}

uint16_t quantizeTimestamp(uint16_t rawTs, unsigned long loopLenLocal) {
  if (quantGrid == 0 || quantStrength <= 0 || loopBeatDuration == 0) return rawTs;
  float unit = quantUnitMs();
  if (unit < 1.0f) unit = 1.0f;

  float nearest = round((float)rawTs / unit) * unit;
  float blended = (float)rawTs + (nearest - (float)rawTs) * ((float)quantStrength / 100.0f);
  long result = (long)(blended + 0.5f);

  if (result < 0) result = 0;
  if (loopLenLocal > 0 && result >= (long)loopLenLocal) result = (long)loopLenLocal - 1;
  if (result > 65000) result = 65000;
  return (uint16_t)result;
}

// Защита от "коротких"/перевёрнутых нот после квантования.
// Минимальная длина ноты = реальный шаг сетки (Q4/Q8/Q16/Q32).
uint16_t protectNoteEnd(uint16_t startTs, uint16_t endTs, unsigned long loopLenLocal, bool isDuringRecording) {
  float unit = quantUnitMs();
  long minLen = (unit >= 1.0f) ? (long)(unit + 0.5f) : MIN_NOTE_LEN_MS;
  if (minLen < MIN_NOTE_LEN_MS) minLen = MIN_NOTE_LEN_MS;

  long diff = (long)endTs - (long)startTs;
  long wrapThreshold = (long)(unit * 3.0f);
  if (wrapThreshold < 200) wrapThreshold = 200;
  bool looksLikeWrap = (diff < -wrapThreshold);
  if (looksLikeWrap || diff >= minLen) return endTs;

  long minEnd = (long)startTs + minLen;
  long bound = isDuringRecording ? 65000L : (loopLenLocal > 0 ? (long)loopLenLocal - 1 : 65000L);
  if (minEnd > bound) minEnd = bound;
  if (minEnd < 0) minEnd = 0;
  return (uint16_t)minEnd;
}

void applyQuantizeToLoop() {
  if (eventCount <= 0 || loopLen == 0) {
    sendBleNotify("/qapply_err 1");
    return;
  }
  if (quantGrid == 0 || quantStrength <= 0) {
    sendBleNotify("/qapply_err 2");
    return;
  }

  resetActiveNoteTracking();

  for (int i = 0; i < eventCount; i++) {
    bool isNoteStart = (events[i].type == 0x90 && events[i].d2 > 0);
    bool isNoteEnd   = (events[i].type == 0x80 || (events[i].type == 0x90 && events[i].d2 == 0));
    byte ch = events[i].ch;
    byte pitch = events[i].d1;
    uint16_t rawTs = events[i].timestamp;

    bool doQuant = false;
    if (events[i].type == 0xB0) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteStart) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteEnd) {
      doQuant = (quantTarget == QUANT_END || quantTarget == QUANT_BOTH);
    }

    uint16_t newTs = rawTs;
    if (doQuant) newTs = quantizeTimestamp(rawTs, loopLen);

    if (isNoteStart) {
      activeNoteStartTs[ch][pitch] = newTs;
    } else if (isNoteEnd) {
      uint16_t startTs = activeNoteStartTs[ch][pitch];
      if (startTs != 0xFFFF) {
        newTs = protectNoteEnd(startTs, newTs, loopLen, false);
        activeNoteStartTs[ch][pitch] = 0xFFFF;
      }
    }

    events[i].timestamp = newTs;
  }

  sendBleNotify("/qapply_ok 1");

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

String presetPath(int slot) {
  return "/preset" + String(slot) + ".bin";
}
bool presetExists(int slot) {
  return LittleFS.exists(presetPath(slot));
}
void sendPresetStatus() {
  String s = "/presets ";
  for (int i = 1; i <= PRESET_COUNT; i++) s += presetExists(i) ? "1" : "0";
  sendBleNotify(s);
}
void sendQuantStatus() {
  sendBleNotify("/quant " + String(quantGrid) + " " + String(quantStrength) + " " + String(quantTarget));
}
bool savePreset(int slot) {
  if (eventCount <= 0 || loopLen == 0 || looperState == STATE_IDLE || looperState == STATE_RECORDING) {
    sendBleNotify("/save_err " + String(slot));
    return false;
  }
  File f = LittleFS.open(presetPath(slot), "w");
  if (!f) { sendBleNotify("/save_err " + String(slot)); return false; }
  uint32_t magic = 0x4C505345UL;
  uint32_t ll = (uint32_t)loopLen;
  int32_t ec = (int32_t)eventCount;
  f.write((uint8_t*)&magic, sizeof(magic));
  f.write((uint8_t*)&ll, sizeof(ll));
  f.write((uint8_t*)&ec, sizeof(ec));
  f.write((uint8_t*)events, sizeof(MidiEvent) * eventCount);
  f.close();
  sendBleNotify("/save_ok " + String(slot));
  sendPresetStatus();
  return true;
}
bool loadPreset(int slot) {
  if (!presetExists(slot)) { sendBleNotify("/load_err " + String(slot)); return false; }
  File f = LittleFS.open(presetPath(slot), "r");
  if (!f) { sendBleNotify("/load_err " + String(slot)); return false; }

  uint32_t magic = 0, ll = 0;
  int32_t ec = 0;
  f.read((uint8_t*)&magic, sizeof(magic));
  f.read((uint8_t*)&ll, sizeof(ll));
  f.read((uint8_t*)&ec, sizeof(ec));

  if (magic != 0x4C505345UL || ec <= 0 || ec > 5000) {
    f.close();
    sendBleNotify("/load_err " + String(slot));
    return false;
  }

  unsigned long now = millis();
  unsigned long prevPos = 0;
  bool hadPrevLoop = (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB));
  if (hadPrevLoop) {
    prevPos = (now - playStart) % loopLen;
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
  }

  size_t toRead = sizeof(MidiEvent) * (size_t)ec;
  size_t got = f.read((uint8_t*)events, toRead);
  f.close();
  if (got != toRead) { sendBleNotify("/load_err " + String(slot)); return false; }

  loopLen = ll;
  eventCount = ec;
  currentLayer = 0;
  for (int i = 0; i < 17; i++) for (int j = 0; j < 128; j++) ccMaxLayer[i][j] = 0;
  for (int i = 0; i < eventCount; i++) {
    events[i].played = false;
    if (events[i].layer > currentLayer) currentLayer = events[i].layer;
    if (events[i].type == 0xB0 && events[i].layer > ccMaxLayer[events[i].ch][events[i].d1]) {
      ccMaxLayer[events[i].ch][events[i].d1] = events[i].layer;
    }
  }

  pendingStart = false;
  pendingStop = false;
  looperState = STATE_PLAYING;

  unsigned long newPos = (loopLen > 0) ? (prevPos % loopLen) : 0;
  playStart = now - newPos;

  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;

  sendBleNotify("/load_ok " + String(slot));
  return true;
}
void deletePreset(int slot) {
  String path = presetPath(slot);
  if (LittleFS.exists(path)) LittleFS.remove(path);
  sendBleNotify("/del_ok " + String(slot));
  sendPresetStatus();
}

class MyCharacteristicCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pChar) {
      std::string rxValue = pChar->getValue().c_str();
      if (rxValue.length() > 0) {
        String cmd = String(rxValue.c_str());
        cmd.trim();
        if (cmd == "/half") {
          halfLoop();
        }
        else if (cmd == "/double") {
          doubleLoop();
        }
        else if (cmd == "/reset") {
          resetLooper();
        }
        else if (cmd == "/rec") {
          if (looperState == STATE_IDLE) pendingStart = true;
        }
        else if (cmd == "/stop") {
          if (looperState == STATE_RECORDING) pendingStop = true;
        }
        else if (cmd == "/overdub") {
          if (looperState == STATE_PLAYING) {
            currentLayer++;
            looperState = STATE_OVERDUB;
            sendBleNotify("/state OVERDUB");
          }
        }
        else if (cmd == "/play") {
          if (looperState == STATE_OVERDUB) {
            looperState = STATE_PLAYING;
            sendBleNotify("/state PLAYING");
          }
        }
        else if (cmd.startsWith("/transpose ")) {
          int v = cmd.substring(11).toInt();
          transposeLoop(v);
        }
        else if (cmd.startsWith("/save ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) savePreset(slot);
        }
        else if (cmd.startsWith("/load ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) {
            if (looperState == STATE_IDLE || looperState == STATE_RECORDING || loopLen == 0) {
              pendingLoadSlot = -1;
              loadPreset(slot);
            } else {
              pendingLoadSlot = slot;
              sendBleNotify("/load_pending " + String(slot));
            }
          }
        }
        else if (cmd.startsWith("/delp ")) {
          int slot = cmd.substring(6).toInt();
          if (slot >= 1 && slot <= PRESET_COUNT) deletePreset(slot);
        }
        else if (cmd == "/getpresets") {
          sendPresetStatus();
        }
        else if (cmd.startsWith("/qgrid ")) {
          int v = cmd.substring(7).toInt();
          if (v >= 0 && v <= 4) { quantGrid = v; sendQuantStatus(); }
        }
        else if (cmd.startsWith("/qstrength ")) {
          int v = cmd.substring(11).toInt();
          if (v < 0) v = 0; if (v > 100) v = 100;
          quantStrength = v;
          sendQuantStatus();
        }
        else if (cmd.startsWith("/qtarget ")) {
          int v = cmd.substring(9).toInt();
          if (v >= 0 && v <= 2) { quantTarget = v; sendQuantStatus(); }
        }
        else if (cmd == "/getquant") {
          sendQuantStatus();
        }
        else if (cmd == "/qapply") {
          applyQuantizeToLoop();
        }
      }
    }
};
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      needSendPresetStatus = true;
      connectStatusTime = millis();
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      visualDumpActive = false;
      visualDumpFinishStage = 0;
      BLEDevice::startAdvertising();
    }
};
const int buttonPins[11] = {19, 23, 5, 13, 12, 14, 27, 25, 26, 32, 33};
const int cc_list[11]    = {46, 47, 38, 41, 41, 41, 42, 35, 35, 37, 40};
const int cc_val[11]     = {0, 1, 100, 100, 0, 104, 0, 1, 5, 0, 0};
unsigned long last_debounce[11] = {0};
bool last_btn_logical_state[11] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH};
int lastValExp = 0, lastValExp2 = 0;
float errmeasure = 40, errestimate = 40, q = 0.5;
float currentestimate = 0.0, lastestimate = 0.0, kalmangain = 0.0;
float errmeasure2 = 40, errestimate2 = 40, q2 = 0.5;
float currentestimate2 = 0.0, lastestimate2 = 0.0, kalmangain2 = 0.0;
float filter1(int value) {
  kalmangain = errestimate / (errestimate + errmeasure);
  currentestimate = lastestimate + kalmangain * (value - lastestimate);
  errestimate = (1.0 - kalmangain) * errestimate + fabs(lastestimate - currentestimate) * q;
  lastestimate = currentestimate; return currentestimate;
}
float filter2(int value2) {
  kalmangain2 = errestimate2 / (errestimate2 + errmeasure2);
  currentestimate2 = lastestimate2 + kalmangain2 * (value2 - lastestimate2);
  errestimate2 = (1.0 - kalmangain2) * errestimate2 + fabs(lastestimate2 - currentestimate2) * q2;
  lastestimate2 = currentestimate2; return currentestimate2;
}
void triggerQuantizedAction() {
  unsigned long now = millis();
  if (pendingStart) {
    eventCount = 0; currentLayer = 0; recStart = now;
    looperState = STATE_RECORDING;
    for(int i=0; i<17; i++) for(int j=0; j<128; j++) ccMaxLayer[i][j] = 0;
    resetActiveNoteTracking();
    pendingStart = false;
    sendBleNotify("/state RECORDING");
  }
  else if (pendingStop) {
    loopLen = now - recStart; playStart = now;
    looperState = STATE_PLAYING;
    pendingStop = false;
    sendBeatInfo();
    sendBleNotify("/state PLAYING");
  }
}
void handleStart() { tickInBeat = 0; beatInBar = 0; lastBeatTime = millis(); }
void handleClock() {
  unsigned long now = millis();
  lastClockTime = now;

  if (tickInBeat == 0) {
    beatDuration = now - lastBeatTime;
    lastBeatTime = now;
    loopBeatDuration = beatDuration; // сетка квантования всегда = живой темп MIDI Clock
    if (beatInBar == 0) triggerQuantizedAction();
  }
  tickInBeat++;
  if (tickInBeat >= 24) { tickInBeat = 0; beatInBar = (beatInBar + 1) % 4; }
}
void recordEvent(byte t, byte d1, byte d2, byte ch) {
  if (ch == 11 || ch == 12) return;
  unsigned long now = millis();

  if (t == 0xB0) { if (now - lastSentCCTime[ch-1][d1] < 15) return; ccTakeover[ch][d1] = now; ccMaxLayer[ch][d1] = currentLayer; }
  if (t == 0xD0 || t == 0xA0) if (now - lastSentATTime[ch-1] < 15) return;

  bool isNoteStart = (t == 0x90 && d2 > 0);
  bool isNoteEnd   = (t == 0x80 || (t == 0x90 && d2 == 0));
  bool isRecOrOver = (looperState == STATE_RECORDING || looperState == STATE_OVERDUB);

  uint16_t finalTs = 0;
  if (isRecOrOver) {
    unsigned long rawTs = (looperState == STATE_RECORDING) ? (now - recStart) : ((now - playStart) % loopLen);
    finalTs = (uint16_t)(rawTs > 65000 ? 65000 : rawTs);

    bool doQuant = false;
    if (t == 0xB0) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteStart) {
      doQuant = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    } else if (isNoteEnd) {
      doQuant = (quantTarget == QUANT_END || quantTarget == QUANT_BOTH);
    }
    if (doQuant) {
      unsigned long loopLenForClamp = (looperState == STATE_RECORDING) ? 0 : loopLen;
      finalTs = quantizeTimestamp(finalTs, loopLenForClamp);
    }

    if (isNoteStart) {
      activeNoteStartTs[ch][d1] = finalTs;
    } else if (isNoteEnd) {
      uint16_t startTs = activeNoteStartTs[ch][d1];
      if (startTs != 0xFFFF) {
        if (quantGrid != 0 && quantStrength > 0) {
          unsigned long loopLenLocal = (looperState == STATE_RECORDING) ? 0 : loopLen;
          finalTs = protectNoteEnd(startTs, finalTs, loopLenLocal, looperState == STATE_RECORDING);
        }
        activeNoteStartTs[ch][d1] = 0xFFFF;
      }
    }
  }

  if (deviceConnected) {
    if (looperState == STATE_RECORDING) {
      if (t == 0xB0) sendBleNotify("/rec_cc " + String(finalTs));
      else if (isNoteStart) sendBleNotify("/rec_noteon " + String(d1) + " " + String(finalTs));
      else if (isNoteEnd) sendBleNotify("/rec_noteoff " + String(d1) + " " + String(finalTs));
    }
    else {
      float posNorm = 0.0;
      if (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
        if (looperState == STATE_OVERDUB) {
          posNorm = (float)finalTs / (float)loopLen;
        } else {
          posNorm = (float)((now - playStart) % loopLen) / (float)loopLen;
        }
      }

      if (t == 0xB0) sendBleNotify("/cc " + String(posNorm, 4));
      else if (t == 0x90 && d2 > 0) sendBleNotify("/noteon " + String(d1) + " " + String(posNorm, 4));
      else if (t == 0x80 || (t == 0x90 && d2 == 0)) sendBleNotify("/noteoff " + String(d1) + " " + String(posNorm, 4));
    }
  }

  if (!isRecOrOver) return;
  if ((t == 0x90 || t == 0x80) && (now - lastSentNoteTime[ch-1][d1] < 20)) return;

  if (eventCount < 5000) {
    events[eventCount++] = {finalTs, t, d1, d2, ch, currentLayer, false};
  }
}
void playback() {
  if (looperState != STATE_PLAYING && looperState != STATE_OVERDUB) return;
  unsigned long now = millis();
  unsigned long pos = (now - playStart) % loopLen;
  static unsigned long lastPos = 0;

  if (pos < lastPos) {
    for (int i = 0; i < eventCount; i++) events[i].played = false;

    if (pendingLoadSlot >= 1) {
      int slotToLoad = pendingLoadSlot;
      pendingLoadSlot = -1;
      loadPreset(slotToLoad);
      return;
    }
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].played) continue;
    if (pos >= events[i].timestamp) {
      if (events[i].type == 0xB0) {
        if (now - ccTakeover[events[i].ch][events[i].d1] < 1200) { events[i].played = true; continue; }
        if (events[i].layer < ccMaxLayer[events[i].ch][events[i].d1]) { events[i].played = true; continue; }
        lastSentCCTime[events[i].ch-1][events[i].d1] = now;
      }
      if (events[i].type == 0x90 || events[i].type == 0x80) lastSentNoteTime[events[i].ch-1][events[i].d1] = now;

      MIDI.send((midi::MidiType)events[i].type, events[i].d1, events[i].d2, events[i].ch);
      events[i].played = true;
    }
  }
  lastPos = pos;
}
void handleButton() {
  byte btn = digitalRead(LOOPER_PIN); unsigned long now = millis(); static byte lastBtn = HIGH;

  if (now - lastClockTime >= 1200) {
    if (now - lastBeatTime >= 545) {
      lastBeatTime = now;
      if (beatInBar == 0) triggerQuantizedAction();
      beatInBar = (beatInBar + 1) % 4;
    }
  }

  if (btn != lastBtn) {
    delay(20);
    if (btn == LOW) { btnTime = now; ignoreRel = false; }
    else if (!ignoreRel) {
      if (looperState == STATE_IDLE) pendingStart = true;
      else if (looperState == STATE_RECORDING) pendingStop = true;
      else if (looperState == STATE_PLAYING) {
        currentLayer++;
        looperState = STATE_OVERDUB;
        sendBleNotify("/state OVERDUB");
      }
      else if (looperState == STATE_OVERDUB) {
        looperState = STATE_PLAYING;
        sendBleNotify("/state PLAYING");
      }
    }
    lastBtn = btn;
  }

  if (btn == LOW && !ignoreRel && (now - btnTime > 1500)) {
    pendingStart = false; pendingStop = false; ignoreRel = true;
    resetLooper();
  }
}
void handleExpression() {
  if (millis() % 20 == 0) {
    int v1 = constrain(map(filter1(analogRead(34) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v1 - lastValExp) > 1 || (v1 == 0 && lastValExp != 0) || (v1 == 127 && lastValExp != 127)) { MIDI.sendControlChange(30, v1, 11); lastValExp = v1; }

    int v2 = constrain(map(filter2(analogRead(35) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v2 - lastValExp2) > 1 || (v2 == 0 && lastValExp2 != 0) || (v2 == 127 && lastValExp2 != 127)) { MIDI.sendControlChange(31, v2, 11); lastValExp2 = v2; }
  }
}
void handleAerosController() {
  unsigned long now = millis();
  for(int i=0; i<11; i++) {
    byte current_reading = digitalRead(buttonPins[i]);
    if (current_reading != last_btn_logical_state[i]) {
      if (now - last_debounce[i] > 50) {
        last_btn_logical_state[i] = current_reading;
        last_debounce[i] = now;
        if (current_reading == LOW) {
          if (cc_list[i] == 42) MIDI.sendControlChange(43, 127, MIDICHANNEL);
          MIDI.sendControlChange(cc_list[i], cc_val[i], MIDICHANNEL);
          if (cc_list[i] == 42) MIDI.sendControlChange(43, 1, MIDICHANNEL);
        }
      }
    }
  }
}
void updateLED() {
  unsigned long now = millis(); unsigned long diff = now - lastBeatTime; int brightness = 0;

  if (digitalRead(LOOPER_PIN) == LOW && !ignoreRel && (now - btnTime > 500)) { brightness = (now % 80 < 40) ? 255 : 0; }
  else if (pendingStart || pendingStop) { brightness = (now % 100 < 50) ? 255 : 10; }
  else {
    float env = 0; unsigned long dur = (beatInBar == 0) ? 120 : 45;
    if (diff < dur) { env = 1.0 - ((float)diff / (float)dur); env = env * env; }
    if (looperState == STATE_IDLE || looperState == STATE_PLAYING) {
      int maxB = (beatInBar == 0) ? 255 : 70; brightness = (int)(env * (float)maxB);
    } else {
      brightness = (int)((1.0 - env) * 255.0);
      if (env > 0.1) { if (beatInBar == 0) brightness = 0; else brightness = 10; }
    }
  }
  ledcWrite(LED_PIN, brightness);
}
void setup() {
  LittleFS.begin(true);
  resetActiveNoteTracking();
  ledcAttach(LED_PIN, pwmFreq, pwmRes);
  pinMode(LOOPER_PIN, INPUT_PULLUP);
  for(int i=0; i<11; i++) pinMode(buttonPins[i], INPUT_PULLUP);

  MIDI_Serial.begin(31250, SERIAL_8N1, 16, 17);
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();

  MIDI.setHandleClock(handleClock); MIDI.setHandleStart(handleStart); MIDI.setHandleContinue(handleStart);
  MIDI.setHandleNoteOn([](byte ch, byte n, byte v) { recordEvent(0x90, n, v, ch); });
  MIDI.setHandleNoteOff([](byte ch, byte n, byte v) { recordEvent(0x80, n, v, ch); });
  MIDI.setHandleControlChange([](byte ch, byte n, byte v) { recordEvent(0xB0, n, v, ch); });
  BLEDevice::init("ESP32_Looper_UI");
  BLEDevice::setMTU(500);
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_WRITE
                    );
  pCharacteristic->setCallbacks(new MyCharacteristicCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);

  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}
void loop() {
  while (MIDI.read()) { }
  handleButton();          
  handleAerosController();
  handleExpression();      
  playback();              
  updateLED();

  if (needSendPresetStatus && deviceConnected && millis() - connectStatusTime > 400) {
    sendPresetStatus();
    sendQuantStatus();
    sendBeatInfo();
    needSendPresetStatus = false;
  }

  if (visualDumpActive && deviceConnected) {
    unsigned long nowDump = millis();

    if (visualDumpIndex < eventCount) {
      if (visualDumpIndex > 0 && nowDump - visualDumpLastPosSend >= VISUAL_DUMP_POS_INTERVAL) {
        visualDumpLastPosSend = nowDump;
        sendBleNotify("/rec_pos " + String(events[visualDumpIndex - 1].timestamp + 1));
      }

      if (nowDump - lastVisualDumpTime >= VISUAL_DUMP_EVENT_INTERVAL) {
        lastVisualDumpTime = nowDump;
        MidiEvent &e = events[visualDumpIndex];
        if (e.type == 0x90 && e.d2 > 0) {
          sendBleNotify("/rec_noteon " + String(e.d1) + " " + String(e.timestamp));
        } else if (e.type == 0x80 || (e.type == 0x90 && e.d2 == 0)) {
          sendBleNotify("/rec_noteoff " + String(e.d1) + " " + String(e.timestamp));
        } else if (e.type == 0xB0) {
          sendBleNotify("/rec_cc " + String(e.timestamp));
        }
        visualDumpIndex++;
      }
    }
    else if (visualDumpFinishStage == 0) {
      sendBleNotify("/rec_pos " + String(loopLen));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 1;
      visualDumpFinishTime = nowDump;
    }
    else if (visualDumpFinishStage == 1 && nowDump - visualDumpFinishTime >= 20) {
      sendBleNotify("/rec_pos " + String(loopLen));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 0;
      visualDumpActive = false;
    }
  }

  if (deviceConnected && !visualDumpActive && (millis() - lastBleUpdate >= BLE_INTERVAL)) {
    lastBleUpdate = millis();
    if (loopLen > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
      uint32_t currentPos = (millis() - playStart) % loopLen;
      float posNorm = (float)currentPos / (float)loopLen;
      sendBleNotify("/pos " + String(posNorm, 4));
    }
    else if (looperState == STATE_RECORDING) {
      sendBleNotify("/rec_pos " + String(millis() - recStart));
    }
    else if (looperState == STATE_IDLE) {
      unsigned long now = millis();
      unsigned long timeInBeat = now - lastBeatTime;
      if (timeInBeat > beatDuration) timeInBeat = beatDuration;

      float idlePosNorm = ((float)beatInBar + ((float)timeInBeat / (float)beatDuration)) / 4.0f;
      sendBleNotify("/pos " + String(idlePosNorm, 4));
    }
  }
}
