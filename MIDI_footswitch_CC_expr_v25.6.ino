#include <HardwareSerial.h>
#include <MIDI.h>

// --- НАСТРОЙКИ ПИНОВ ---
#define LOOPER_PIN 18      
#define LED_PIN 2          
#define MIDICHANNEL 12 
#define EXPPIN1 34
#define EXPPIN2 35

HardwareSerial MIDI_Serial(2);
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_Serial, MIDI); 

const int pwmFreq = 5000;
const int pwmRes = 8;

// --- ПЕРЕМЕННЫЕ КОНТРОЛЛЕРА ---
const int buttonPins[11] = {19, 23, 5, 13, 12, 14, 27, 25, 26, 32, 33}; 
const int cc_list[11]    = {46, 47, 38, 41, 41, 41, 42, 35, 35, 37, 40};
const int cc_val[11]     = {0, 1, 100, 100, 0, 104, 0, 1, 5, 0, 0};
unsigned long last_debounce[11] = {0};
bool last_btn_logical_state[11] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; 

// --- ПЕРЕМЕННЫЕ ЭКСПРЕССИИ ---
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

// --- ДВИЖОК ЛУПЕРА ---
enum LooperState { STATE_IDLE, STATE_RECORDING, STATE_PLAYING, STATE_OVERDUB };
LooperState looperState = STATE_IDLE;
struct MidiEvent { uint16_t timestamp; byte type, d1, d2, ch, layer; bool played; };
MidiEvent events[5000]; 
int eventCount = 0; byte currentLayer = 0; byte ccMaxLayer[17][128]; 
unsigned long loopLen = 0, recStart = 0, playStart = 0, btnTime = 0;
bool ignoreRel = false;
unsigned long lastSentNoteTime[16][128], lastSentCCTime[16][128], lastSentATTime[16], ccTakeover[17][128]; 

// --- СИНХРОНИЗАЦИЯ ---
volatile bool pendingStart = false;
volatile bool pendingStop = false;
volatile unsigned long lastBeatTime = 0;   
volatile unsigned long lastClockTime = 0;
volatile int beatInBar = 0;       
volatile int tickInBeat = 0;      
unsigned long beatDuration = 545; 

void triggerQuantizedAction() {
  unsigned long now = millis();
  if (pendingStart) {
    eventCount = 0; currentLayer = 0; recStart = now; 
    looperState = STATE_RECORDING;
    for(int i=0; i<17; i++) for(int j=0; j<128; j++) ccMaxLayer[i][j] = 0;
    pendingStart = false;
  } 
  else if (pendingStop) {
    loopLen = now - recStart; playStart = now; 
    looperState = STATE_PLAYING;
    pendingStop = false;
  }
}

void handleStart() { 
  tickInBeat = 0; beatInBar = 0; lastBeatTime = millis(); 
  MIDI.sendRealTime(midi::Start); // Пробрасываем команду Start
}

void handleClock() { 
  unsigned long now = millis();
  
  // ПРОБРОС: Если пришел тик, отправляем его дальше
  MIDI.sendRealTime(midi::Clock);
  
  lastClockTime = now;
  if (tickInBeat == 0) { 
    beatDuration = now - lastBeatTime; 
    lastBeatTime = now; 
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
  if (looperState != STATE_RECORDING && looperState != STATE_OVERDUB) return;
  if ((t == 0x90 || t == 0x80) && (now - lastSentNoteTime[ch-1][d1] < 20)) return;
  if (eventCount < 5000) {
    unsigned long rawTs = (looperState == STATE_RECORDING) ? (now - recStart) : ((now - playStart) % loopLen);
    events[eventCount++] = {(uint16_t)(rawTs > 65000 ? 65000 : rawTs), t, d1, d2, ch, currentLayer, false};
  }
}

void playback() {
  if (looperState != STATE_PLAYING && looperState != STATE_OVERDUB) return;
  unsigned long now = millis();
  unsigned long pos = (now - playStart) % loopLen;
  static unsigned long lastPos = 0;
  if (pos < lastPos) for (int i = 0; i < eventCount; i++) events[i].played = false;
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
  // ВНУТРЕННИЙ ТАЙМЕР: Работает только если внешнего клока нет дольше 1.2 сек
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
      else if (looperState == STATE_PLAYING) { currentLayer++; looperState = STATE_OVERDUB; }
      else if (looperState == STATE_OVERDUB) looperState = STATE_PLAYING;
    }
    lastBtn = btn;
  }
  if (btn == LOW && !ignoreRel && (now - btnTime > 1500)) {
    for (int i = 0; i < eventCount; i++) if (events[i].type == 0x90) MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
    looperState = STATE_IDLE; eventCount = 0; currentLayer = 0; pendingStart = false; pendingStop = false; ignoreRel = true;
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
  ledcAttach(LED_PIN, pwmFreq, pwmRes);
  pinMode(LOOPER_PIN, INPUT_PULLUP);
  for(int i=0; i<11; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  MIDI_Serial.begin(31250, SERIAL_8N1, 16, 17);
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();
  
  // Привязываем обработчики
  MIDI.setHandleClock(handleClock); 
  MIDI.setHandleStart(handleStart); 
  MIDI.setHandleContinue(handleStart);
  
  MIDI.setHandleNoteOn([](byte ch, byte n, byte v) { recordEvent(0x90, n, v, ch); });
  MIDI.setHandleNoteOff([](byte ch, byte n, byte v) { recordEvent(0x80, n, v, ch); });
  MIDI.setHandleControlChange([](byte ch, byte n, byte v) { recordEvent(0xB0, n, v, ch); });
}

void loop() {
  while (MIDI.read()) { } 
  handleButton();
  handleAerosController();
  handleExpression();
  playback();
  updateLED();
}