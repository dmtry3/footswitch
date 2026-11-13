// ESP32 MIDI Looper Engine v55 - tick-based engine + BPM moving average + MIDI Start reset
// v52: load preset from IDLE waits for first beat, pre-roll notes, soft LED pulse while waiting
// v53: quantize rewrite (pair-based, loop-fit grid, wrap), stuck-note repair, robust tempo measurement
// v53b: compact active-note table (DRAM overflow fix)
// v54: время лупа в ТИКАХ клока (PPQN=24), BPM = скользящее среднее, Start сбрасывает позицию лупа,
//      неблокирующий debounce педали
// v55: ЗАПИСЬ CC переписана по образцу рабочего лупера:
//      - в одном тике хранится одно значение CC (последнее) -> нет переполнения events[]
//      - CC в овердабе: перезапись поверх (старые CC стираются только там, где прошла ручка)
//      - убран ccMaxLayer (глушил CC старых слоёв по всему лупу)
//      - CC больше не квантуются (ни при записи, ни в /qapply)
//      - поток BLE-уведомлений по CC ограничен (notify блокировал основной цикл)
//      - CC/ноты, записанные в овердабе "вживую", не проигрываются повторно сразу
//      - компактные структуры (без 26 КБ DRAM)
// v56: кнопки футсвитча (CC 21-31) шлют 127 при нажатии и 0 при отпускании (momentary)
// v57: порт на ESP32-S3-Zero: новые пины, MIDI на UART1 (GPIO43/44), экспрессия на ADC1 (GPIO1/2)
// v58: Aeros убран (CC43/pendingAerosStart, проброс клока и канала 12 IN->OUT); управление Zynthian
//      по USB MIDI (родной USB) + DIN; все отправки идут через midiSendMsg/midiCC/midiNoteOff

#include <Arduino.h>
#include <HardwareSerial.h>
#include <MIDI.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <LittleFS.h>
#include <USB.h>
#include <USBMIDI.h>

// ===== КАНАЛЫ СВЯЗИ MIDI (включены оба) =====
#define USE_USB_MIDI 1   // USB MIDI (родной USB ESP32-S3): Zynthian видит футсвитч как обычное MIDI-устройство
#define USE_DIN_MIDI 1   // DIN MIDI на UART1 (GPIO43/44); поставьте 0, если схемы DIN нет
// --- ПИНЫ ESP32-S3-Zero (Waveshare) ---
#define LOOPER_PIN 14
#define LED_PIN 15            // внешний светодиод через резистор (на плате встроенный RGB, он тут не используется)
#define MIDICHANNEL 12
#define EXPPIN1 1             // ADC1
#define EXPPIN2 2             // ADC1
#define MIDI_RX_PIN 44        // вход DIN
#define MIDI_TX_PIN 43        // выход DIN
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define PRESET_COUNT 5

// Магическое число пресета (длина лупа в тиках). Старые пресеты в мс отклоняются.
#define PRESET_MAGIC 0x4C505346UL

HardwareSerial MIDI_Serial(1);   // на S3 всего 3 UART; UART1 оставляем под MIDI
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_Serial, MIDI);

// ===== MIDI-ВЫХОД: USB и/или DIN =====
#if USE_USB_MIDI
USBMIDI usbMidi;
#endif

// Один USB-MIDI пакет: header = (кабель 0 << 4) | CIN. Если буфер полон или хост не подключён — пропускаем.
void usbSendPacket(uint8_t header, uint8_t b1, uint8_t b2, uint8_t b3) {
#if USE_USB_MIDI
  midiEventPacket_t p = { header, b1, b2, b3 };
  for (int i = 0; i < 4; i++) {
    if (usbMidi.writePacket(&p)) return;
    yield();
  }
#endif
}

// type: 0x80..0xE0 (без канала), ch: 1..16
void midiSendMsg(midi::MidiType type, byte d1, byte d2, byte ch) {
  if (ch < 1 || ch > 16) return;
  byte status = (byte)type | (byte)(ch - 1);
  usbSendPacket((uint8_t)((byte)type >> 4), status, d1, d2);   // CIN = старший полубайт статуса
#if USE_DIN_MIDI
  MIDI.send(type, d1, d2, ch);
#endif
}
void midiNoteOff(byte note, byte vel, byte ch) { midiSendMsg(midi::NoteOff, note, vel, ch); }
void midiCC(byte cc, byte val, byte ch)        { midiSendMsg(midi::ControlChange, cc, val, ch); }
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
unsigned long loopBeatDuration = 545; // мс на долю: сглаженное значение (скользящее среднее по тикам MIDI Clock)

// ===== ТИКОВОЕ ВРЕМЯ =====
#define PPQN        24
#define BEAT_TICKS  PPQN
#define BAR_TICKS   (PPQN * 4)
#define MIN_NOTE_TICKS 1

volatile uint32_t currentTick = 0;     // индекс последнего обработанного тика клока
volatile bool startArmed = false;      // после Start первый F8 = тик 0 (без инкремента)

// ===== BPM: СКОЛЬЗЯЩЕЕ СРЕДНЕЕ =====
#define BPM_WINDOW_TICKS 48        // 2 доли; 24 = отзывчивее, 96 = стабильнее
#define BPM_MIN_SAMPLES  24        // оценка начинается после 1 доли
const float BPM_MIN = 20.0f, BPM_MAX = 300.0f;
float activeBPM = 60000.0f / 545.0f;          // сглаженный BPM
uint32_t tickPeriodUs = 545000UL / PPQN;      // сглаженный период тика (мкс), для внутреннего клока
uint32_t tickTimesUs[BPM_WINDOW_TICKS];       // кольцевой буфер меток micros()
uint16_t tickTimeIdx = 0;                     // куда писать следующую метку
uint16_t tickTimeCount = 0;                   // сколько меток накоплено

// "Опорная" длительность доли для BLE-клиента (клиент ждёт миллисекунды).
uint32_t bleBeatMs = 545;

uint32_t recStartTick = 0, playStartTick = 0, loopLenTicks = 0;

inline uint32_t ticksSince(uint32_t base) {
  int32_t d = (int32_t)(currentTick - base);
  return d < 0 ? 0 : (uint32_t)d;
}
inline uint32_t ticksToBleMs(uint32_t t) { return (uint32_t)(((uint64_t)t * bleBeatMs) / PPQN); }

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

// --- PRE-ROLL: ноты, сыгранные чуть раньше первой доли, пока лупер ждёт старт записи ---
const long PREROLL_WINDOW_MS = 350;   // насколько "чуть раньше" (мс до первой доли)
const int  PREROLL_MAX = 32;
struct PreRollEv { byte type, d1, d2, ch; };
PreRollEv preRoll[PREROLL_MAX];
int preRollCount = 0;

// Сколько тиков осталось до следующей первой доли такта (0 = следующий тик и есть первая доля)
long ticksToDownbeat() {
  long n = (long)beatInBar * PPQN + (long)tickInBeat;   // индекс следующего тика внутри такта
  return (BAR_TICKS - n) % BAR_TICKS;
}
// Окно пре-ролла в тиках (по текущему сглаженному темпу)
long preRollWindowTicks() {
  uint32_t tp = (tickPeriodUs > 0) ? tickPeriodUs : 1;
  return (long)((PREROLL_WINDOW_MS * 1000L) / (long)tp);
}

void preRollAdd(byte t, byte d1, byte d2, byte ch, bool isNoteStart) {
  if (preRollCount >= PREROLL_MAX) return;
  if (isNoteStart) {
    if (ticksToDownbeat() > preRollWindowTicks()) return;   // слишком рано — игнорируем
  } else {
    // note-off берём только если в буфере есть открытая нота с таким же pitch
    bool open = false;
    for (int i = preRollCount - 1; i >= 0; i--) {
      if (preRoll[i].ch == ch && preRoll[i].d1 == d1) {
        open = (preRoll[i].type == 0x90 && preRoll[i].d2 > 0);
        break;
      }
    }
    if (!open) return;
  }
  preRoll[preRollCount++] = { t, d1, d2, ch };
}

struct MidiEvent {
  uint16_t timestamp;     // в ТИКАХ (PPQN = 24)
  byte type, d1, d2, ch, layer;
  bool played;
};
MidiEvent events[5000];
int eventCount = 0;
byte currentLayer = 0;
unsigned long btnTime = 0;
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
  sendBleNotify("/beatinfo " + String(bleBeatMs));
}

// --- Отслеживание "открытых" нот (note-on без note-off) при записи / квантовании ---
const int MAX_ACTIVE_NOTES = 64;
struct ActiveNote { bool used; byte ch; byte pitch; long raw; long q; };
ActiveNote activeNotes[MAX_ACTIVE_NOTES];

void resetActiveNoteTracking() {
  for (int i = 0; i < MAX_ACTIVE_NOTES; i++) activeNotes[i].used = false;
}
int findActiveNote(byte ch, byte pitch) {
  for (int i = 0; i < MAX_ACTIVE_NOTES; i++) {
    if (activeNotes[i].used && activeNotes[i].ch == ch && activeNotes[i].pitch == pitch) return i;
  }
  return -1;
}
void setActiveNote(byte ch, byte pitch, long raw, long q) {
  int idx = findActiveNote(ch, pitch);
  if (idx < 0) {
    for (int i = 0; i < MAX_ACTIVE_NOTES; i++) {
      if (!activeNotes[i].used) { idx = i; break; }
    }
  }
  if (idx < 0) idx = 0;   // таблица переполнена — вытесняем первую запись
  activeNotes[idx].used = true;
  activeNotes[idx].ch = ch;
  activeNotes[idx].pitch = pitch;
  activeNotes[idx].raw = raw;
  activeNotes[idx].q = q;
}
bool takeActiveNote(byte ch, byte pitch, long &raw, long &q) {
  int idx = findActiveNote(ch, pitch);
  if (idx < 0) return false;
  raw = activeNotes[idx].raw;
  q = activeNotes[idx].q;
  activeNotes[idx].used = false;
  return true;
}
void clearActiveNote(byte ch, byte pitch) {
  int idx = findActiveNote(ch, pitch);
  if (idx >= 0) activeNotes[idx].used = false;
}

// ===== CC В ОВЕРДАБЕ: ПЕРЕЗАПИСЬ ПОВЕРХ =====
// При движении ручки в овердабе стираются старые события этого ch/cc в диапазоне лупа,
// который "проехала" ручка, и пишутся новые. Пока овердаб включён, CC, которые трогали,
// не воспроизводятся (ни старые, ни новые); воспроизведение возвращается при выходе из овердаба.
// Компактно: битовая маска "трогали" + маленькая таблица активных "свипов".
// ВАЖНО: функции возвращают int-индексы (не указатели на структуру), иначе автопрототипы
// Arduino-препроцессора ломают компиляцию.
#define CC_OVERDUB_GAP_MS 1000     // пауза ручки, после которой перезапись останавливается
#define CC_OVERDUB_REDUMP 0
#define MAX_CC_SWEEPS 16
uint8_t ccTouched[17][16];         // бит (cc & 7) в байте (cc >> 3)
struct CcSweep { bool used; byte ch; byte cc; uint16_t pos; uint32_t ms; uint32_t tick; };
CcSweep ccSweeps[MAX_CC_SWEEPS];
bool ccOdErased = false;

bool ccRefreshActive = false;      // лёгкое обновление CC-меток в приложении
int ccRefreshIdx = 0;
unsigned long ccRefreshLastMs = 0;
#define CC_MARK_BUCKETS 512
uint8_t ccMarkSent[CC_MARK_BUCKETS / 8];

bool ccIsTouched(byte ch, byte cc) { return ((ccTouched[ch][cc >> 3] >> (cc & 7)) & 1) != 0; }
void ccSetTouched(byte ch, byte cc) { ccTouched[ch][cc >> 3] |= (uint8_t)(1 << (cc & 7)); }

int ccFindSweep(byte ch, byte cc) {
  for (int i = 0; i < MAX_CC_SWEEPS; i++)
    if (ccSweeps[i].used && ccSweeps[i].ch == ch && ccSweeps[i].cc == cc) return i;
  return -1;
}
int ccAllocSweep(byte ch, byte cc) {
  int idx = -1;
  for (int i = 0; i < MAX_CC_SWEEPS; i++) if (!ccSweeps[i].used) { idx = i; break; }
  if (idx < 0) {                       // таблица полна — вытесняем самый старый свип
    idx = 0;
    for (int i = 1; i < MAX_CC_SWEEPS; i++)
      if ((int32_t)(ccSweeps[idx].ms - ccSweeps[i].ms) > 0) idx = i;
  }
  ccSweeps[idx].used = true;
  ccSweeps[idx].ch = ch;
  ccSweeps[idx].cc = cc;
  ccSweeps[idx].pos = 0;
  ccSweeps[idx].ms = 0;
  ccSweeps[idx].tick = 0;
  return idx;
}
void ccResetSweeps() {                 // новые сегменты перезаписи (флаги "трогали" остаются)
  for (int i = 0; i < MAX_CC_SWEEPS; i++) ccSweeps[i].used = false;
}
void ccResetOverdubState() {           // полный сброс (выход из овердаба)
  memset(ccTouched, 0, sizeof(ccTouched));
  ccResetSweeps();
}

void startVisualDump() {
  ccRefreshActive = false;
  sendBleNotify("/reset 1");
  sendBleNotify("/state RECORDING");
  visualDumpIndex = 0;
  visualDumpLastPosSend = millis();
  visualDumpFinishStage = 0;
  visualDumpActive = true;
}

// Удаляет CC (ch,cc) в диапазоне тиков [start, start+count) по кругу лупа
void eraseCCRange(byte ch, byte cc, uint32_t start, uint32_t count) {
  if (loopLenTicks == 0 || count == 0) return;
  if (count > loopLenTicks) count = loopLenTicks;
  start %= loopLenTicks;
  int n = 0;
  for (int i = 0; i < eventCount; i++) {
    const MidiEvent &e = events[i];
    if (e.type == 0xB0 && e.ch == ch && e.d1 == cc) {
      uint32_t rel = ((uint32_t)(e.timestamp % loopLenTicks) + loopLenTicks - start) % loopLenTicks;
      if (rel < count) continue;               // удалить
    }
    if (n != i) events[n] = events[i];
    n++;
  }
  eventCount = n;
}

void startCcRefresh() {
  if (!deviceConnected || loopLenTicks == 0) return;
  ccRefreshActive = true;
  ccRefreshIdx = 0;
  ccRefreshLastMs = 0;
  memset(ccMarkSent, 0, sizeof(ccMarkSent));
  sendBleNotify("/ccclear 1");
}

// Вызывается из loop(): шлёт по одной CC-метке, не мешая /pos и состоянию
void serviceCcRefresh() {
  if (!ccRefreshActive) return;
  if (!deviceConnected || visualDumpActive || loopLenTicks == 0 ||
      looperState == STATE_IDLE || looperState == STATE_RECORDING) {
    ccRefreshActive = false;
    return;
  }
  if (looperState == STATE_OVERDUB) return;                 // пауза, пока снова в овердабе
  unsigned long now = millis();
  if (now - ccRefreshLastMs < 4) return;
  while (ccRefreshIdx < eventCount) {
    const MidiEvent &e = events[ccRefreshIdx++];
    if (e.type != 0xB0) continue;
    uint32_t b = ((uint32_t)e.timestamp * CC_MARK_BUCKETS) / loopLenTicks;
    if (b >= CC_MARK_BUCKETS) b = CC_MARK_BUCKETS - 1;
    if (ccMarkSent[b >> 3] & (1 << (b & 7))) continue;     // одна метка на корзину
    ccMarkSent[b >> 3] |= (uint8_t)(1 << (b & 7));
    ccRefreshLastMs = now;
    sendBleNotify("/ccmark " + String((float)e.timestamp / (float)loopLenTicks, 4));
    return;
  }
  ccRefreshActive = false;
}

// Вызывается при выходе из OVERDUB (в любую сторону)
void onOverdubExit() {
  ccResetOverdubState();
  if (ccOdErased) {
    ccOdErased = false;
    if (deviceConnected && !visualDumpActive && loopLenTicks > 0) startCcRefresh();
  }
}

// Находит ноты, у которых есть note-on, но нет note-off, и дописывает им note-off в момент endTs.
void repairOpenNotes(uint16_t endTs, bool markPlayed) {
  struct OpenNote { byte ch; byte pitch; int16_t idx; };
  OpenNote open[64];
  int n = 0;

  for (int i = 0; i < eventCount; i++) {
    byte type = events[i].type;
    if (type != 0x90 && type != 0x80) continue;
    byte ch = events[i].ch;
    byte p = events[i].d1;
    bool on = (type == 0x90 && events[i].d2 > 0);

    int k = -1;
    for (int j = 0; j < n; j++) {
      if (open[j].ch == ch && open[j].pitch == p) { k = j; break; }
    }
    if (on) {
      if (k >= 0) open[k].idx = (int16_t)i;
      else if (n < 64) { open[n].ch = ch; open[n].pitch = p; open[n].idx = (int16_t)i; n++; }
    } else if (k >= 0) {
      open[k] = open[n - 1];
      n--;
    }
  }

  for (int j = 0; j < n; j++) {
    if (eventCount >= 5000) break;
    MidiEvent ev;
    ev.timestamp = endTs;
    ev.type = 0x80;
    ev.d1 = open[j].pitch;
    ev.d2 = 0;
    ev.ch = open[j].ch;
    ev.layer = events[open[j].idx].layer;
    ev.played = markPlayed;
    events[eventCount++] = ev;
  }
}

// Выход из овердаба: закрываем ноты, которые игрок ещё держит, и сбрасываем трекинг
void endOverdub() {
  if (looperState != STATE_OVERDUB) return;
  if (loopLenTicks > 0) repairOpenNotes((uint16_t)((currentTick - playStartTick) % loopLenTicks), true);
  resetActiveNoteTracking();
  looperState = STATE_PLAYING;
  sendBleNotify("/state PLAYING");
}

void resetLooper() {
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) {
      midiNoteOff(events[i].d1, 0, events[i].ch);
    }
  }

  looperState = STATE_IDLE;
  eventCount = 0;
  currentLayer = 0;
  loopLenTicks = 0;
  recStartTick = 0;
  playStartTick = 0;
  pendingStart = false;
  pendingStop = false;
  pendingLoadSlot = -1;
  preRollCount = 0;
  ccOdErased = false;
  ccResetOverdubState();
  resetActiveNoteTracking();

  sendBleNotify("/reset 1");
}
void halfLoop() {
  if (loopLenTicks < BEAT_TICKS || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) midiNoteOff(events[i].d1, 0, events[i].ch);
  }
  loopLenTicks /= 2;
  int newCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].timestamp < loopLenTicks) {
      events[newCount++] = events[i];
    }
  }
  eventCount = newCount;
  // note-off из отрезанной половины пропали — закрываем оставшиеся открытые ноты
  repairOpenNotes((uint16_t)(loopLenTicks - 1), false);
  resetActiveNoteTracking();
  ccResetSweeps();
}
void doubleLoop() {
  if (loopLenTicks == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (loopLenTicks * 2 > 65000) return;
  int origCount = eventCount;
  unsigned long oldLen = loopLenTicks;
  for (int i = 0; i < origCount; i++) {
    if (eventCount < 5000) {
      MidiEvent newEv = events[i];
      newEv.timestamp = (uint16_t)(events[i].timestamp + oldLen);
      newEv.played = false;
      events[eventCount++] = newEv;
    }
  }
  loopLenTicks *= 2;
  resetActiveNoteTracking();
  ccResetSweeps();
}

// --- ТРАНСПОНИРОВАНИЕ ЛУПА НА ОКТАВУ (-12/+12) ---
void transposeLoop(int semitones) {
  if (loopLenTicks == 0 || (looperState != STATE_PLAYING && looperState != STATE_OVERDUB)) return;
  if (semitones == 0) return;

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) midiNoteOff(events[i].d1, 0, events[i].ch);
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90 || events[i].type == 0x80) {
      int newPitch = (int)events[i].d1 + semitones;
      if (newPitch < 0) newPitch = 0;
      if (newPitch > 127) newPitch = 127;
      events[i].d1 = (byte)newPitch;
    }
  }
  resetActiveNoteTracking();

  startVisualDump();
}

// --- UNDO: удаляет последний записанный слой овердаба ---
// Примечание: старые CC, стёртые перезаписью в этом слое, НЕ восстанавливаются.
void undoLastLayer() {
  if (loopLenTicks == 0 || currentLayer == 0) {
    sendBleNotify("/undo_err 1"); // нечего отменять
    return;
  }
  byte layerToRemove = currentLayer;

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90 && events[i].layer == layerToRemove) {
      midiNoteOff(events[i].d1, 0, events[i].ch);
    }
  }

  int newCount = 0;
  for (int i = 0; i < eventCount; i++) {
    if (events[i].layer != layerToRemove) {
      events[newCount++] = events[i];
    }
  }
  eventCount = newCount;
  currentLayer--;

  if (looperState == STATE_OVERDUB) {
    looperState = STATE_PLAYING;     // выход из овердаба обработает loop() -> onOverdubExit()
  }
  resetActiveNoteTracking();

  sendBleNotify("/undo_ok 1");
  startVisualDump();
}

// ===================== КВАНТИЗАЦИЯ (сетка в ТИКАХ) =====================

bool quantIsActive() {
  return (quantGrid != 0 && quantStrength > 0);
}

float gridBeatTicks(unsigned long loopLenLocal) {
  if (loopLenLocal > 0) {
    float n = roundf((float)loopLenLocal / (float)BEAT_TICKS);
    if (n < 1.0f) n = 1.0f;
    return (float)loopLenLocal / n;
  }
  return (float)BEAT_TICKS;
}

float quantUnitTicksFor(unsigned long loopLenLocal) {
  if (quantGrid == 0) return 0.0f;
  float b = gridBeatTicks(loopLenLocal);
  if (b < 1.0f) return 0.0f;
  switch (quantGrid) {
    case 1: return b / 8.0f;  // Q32 (3 тика)
    case 2: return b / 4.0f;  // Q16 (6)
    case 3: return b / 2.0f;  // Q8  (12)
    case 4: return b;         // Q4  (24)
    default: return 0.0f;
  }
}
float quantUnitTicks() {
  return quantUnitTicksFor(0);
}

long quantizeLogical(long raw, unsigned long loopLenLocal) {
  if (!quantIsActive()) return raw;
  float unit = quantUnitTicksFor(loopLenLocal);
  if (unit < 1.0f) return raw;
  float nearest = roundf((float)raw / unit) * unit;
  float blended = (float)raw + (nearest - (float)raw) * ((float)quantStrength / 100.0f);
  long r = (long)(blended + 0.5f);
  if (r < 0) r = 0;
  return r;
}

uint16_t wrapTs(long logical, unsigned long loopLenLocal) {
  if (logical < 0) logical = 0;
  if (loopLenLocal > 0) logical %= (long)loopLenLocal;
  if (logical > 65000) logical = 65000;
  return (uint16_t)logical;
}

// Квантование одиночного события (осиротевший note-off) с wrap. CC НЕ квантуем.
uint16_t quantizeTimestamp(uint16_t rawTs, unsigned long loopLenLocal) {
  return wrapTs(quantizeLogical((long)rawTs, loopLenLocal), loopLenLocal);
}

long resolveNoteEnd(long onRaw, long onQ, long offRaw, unsigned long loopLenLocal, bool doQuantEnd) {
  long off;
  if (doQuantEnd) {
    off = quantizeLogical(offRaw, loopLenLocal);
    if (quantIsActive()) {
      float unit = quantUnitTicksFor(loopLenLocal);
      long minLen = (unit >= 1.0f) ? (long)(unit + 0.5f) : (long)MIN_NOTE_TICKS;
      if (minLen < MIN_NOTE_TICKS) minLen = MIN_NOTE_TICKS;
      if (off < onQ + minLen) off = onQ + minLen;
    }
  } else {
    off = offRaw + (onQ - onRaw);
  }
  if (off < onQ + MIN_NOTE_TICKS) off = onQ + MIN_NOTE_TICKS;
  if (loopLenLocal > 0 && off > onQ + (long)loopLenLocal - 1) off = onQ + (long)loopLenLocal - 1;
  return off;
}

void applyQuantizeToLoop() {
  if (eventCount <= 0 || loopLenTicks == 0) {
    sendBleNotify("/qapply_err 1");
    return;
  }
  if (quantGrid == 0 || quantStrength <= 0) {
    sendBleNotify("/qapply_err 2");
    return;
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) midiNoteOff(events[i].d1, 0, events[i].ch);
  }

  bool qStart = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
  bool qEnd   = (quantTarget == QUANT_END   || quantTarget == QUANT_BOTH);

  resetActiveNoteTracking();

  for (int i = 0; i < eventCount; i++) {
    MidiEvent &e = events[i];
    bool isOn  = (e.type == 0x90 && e.d2 > 0);
    bool isOff = (e.type == 0x80 || (e.type == 0x90 && e.d2 == 0));
    long raw = (long)e.timestamp;

    if (e.type == 0xB0) {
      continue;   // CC не квантуем: иначе плавные свипы превращаются в ступеньки
    }
    else if (isOn) {
      long qL = qStart ? quantizeLogical(raw, loopLenTicks) : raw;
      setActiveNote(e.ch, e.d1, raw, qL);
      e.timestamp = wrapTs(qL, loopLenTicks);
    }
    else if (isOff) {
      long onRaw = 0, onQ = 0;
      if (takeActiveNote(e.ch, e.d1, onRaw, onQ)) {
        long offRaw = raw;
        if (offRaw < onRaw) offRaw += (long)loopLenTicks;       // нота перешла через границу лупа
        long offL = resolveNoteEnd(onRaw, onQ, offRaw, loopLenTicks, qEnd);
        e.timestamp = wrapTs(offL, loopLenTicks);
      } else if (qEnd) {
        e.timestamp = quantizeTimestamp(e.timestamp, loopLenTicks);   // "осиротевший" note-off
      }
    }
  }
  resetActiveNoteTracking();

  uint32_t pos = 0;
  if (loopLenTicks > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
    pos = (currentTick - playStartTick) % loopLenTicks;
  }
  for (int i = 0; i < eventCount; i++) events[i].played = ((uint32_t)events[i].timestamp <= pos);

  sendBleNotify("/qapply_ok 1");
  startVisualDump();
}

// Вызывается в момент остановки ПЕРВОЙ записи, когда уже известна loopLenTicks
void finalizeRecordedLoop() {
  if (loopLenTicks == 0) return;

  struct ShiftNote { byte ch; byte pitch; bool shifted; };
  ShiftNote sh[64];
  int n = 0;

  for (int i = 0; i < eventCount; i++) {
    MidiEvent &e = events[i];
    bool isOn  = (e.type == 0x90 && e.d2 > 0);
    bool isOff = (e.type == 0x80 || (e.type == 0x90 && e.d2 == 0));

    if (isOn) {
      bool s = (e.timestamp >= loopLenTicks);
      if (s) e.timestamp = (uint16_t)(e.timestamp % loopLenTicks);
      int k = -1;
      for (int j = 0; j < n; j++) if (sh[j].ch == e.ch && sh[j].pitch == e.d1) { k = j; break; }
      if (k >= 0) sh[k].shifted = s;
      else if (n < 64) { sh[n].ch = e.ch; sh[n].pitch = e.d1; sh[n].shifted = s; n++; }
    }
    else if (isOff) {
      int k = -1;
      for (int j = 0; j < n; j++) if (sh[j].ch == e.ch && sh[j].pitch == e.d1) { k = j; break; }
      bool s = (k >= 0) && sh[k].shifted;
      if (e.timestamp >= loopLenTicks) {
        e.timestamp = s ? (uint16_t)(e.timestamp % loopLenTicks) : (uint16_t)(loopLenTicks - 1);
      }
      if (k >= 0) { sh[k] = sh[n - 1]; n--; }
    }
    else if (e.timestamp >= loopLenTicks) {
      e.timestamp = (uint16_t)(e.timestamp % loopLenTicks);
    }
  }

  repairOpenNotes((uint16_t)(loopLenTicks - 1), false);
  resetActiveNoteTracking();
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
  if (eventCount <= 0 || loopLenTicks == 0 || looperState == STATE_IDLE || looperState == STATE_RECORDING) {
    sendBleNotify("/save_err " + String(slot));
    return false;
  }
  File f = LittleFS.open(presetPath(slot), "w");
  if (!f) { sendBleNotify("/save_err " + String(slot)); return false; }
  uint32_t magic = PRESET_MAGIC;
  uint32_t ll = (uint32_t)loopLenTicks;
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

  if (magic != PRESET_MAGIC || ec <= 0 || ec > 5000 || ll == 0 || ll > 65000) {
    f.close();
    sendBleNotify("/load_err " + String(slot));
    return false;
  }

  uint32_t prevPos = 0;
  bool hadPrevLoop = (loopLenTicks > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB));
  if (hadPrevLoop) {
    prevPos = (currentTick - playStartTick) % loopLenTicks;
  }

  for (int i = 0; i < eventCount; i++) {
    if (events[i].type == 0x90) midiNoteOff(events[i].d1, 0, events[i].ch);
  }

  size_t toRead = sizeof(MidiEvent) * (size_t)ec;
  size_t got = f.read((uint8_t*)events, toRead);
  f.close();
  if (got != toRead) { sendBleNotify("/load_err " + String(slot)); return false; }

  loopLenTicks = ll;
  eventCount = ec;
  currentLayer = 0;
  for (int i = 0; i < eventCount; i++) {
    events[i].played = false;
    if (events[i].layer > currentLayer) currentLayer = events[i].layer;
  }

  pendingStart = false;
  pendingStop = false;
  preRollCount = 0;
  ccOdErased = false;
  ccResetOverdubState();
  resetActiveNoteTracking();
  looperState = STATE_PLAYING;

  uint32_t newPos = prevPos % loopLenTicks;
  playStartTick = currentTick - newPos;
  bleBeatMs = loopBeatDuration;
  sendBeatInfo();

  startVisualDump();

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
        else if (cmd == "/undo") {
          undoLastLayer();
        }
        else if (cmd == "/rec") {
          if (looperState == STATE_IDLE) {
            pendingLoadSlot = -1;     // запись важнее отложенной загрузки
            preRollCount = 0;
            pendingStart = true;
          }
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
            endOverdub();
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
            if (looperState == STATE_RECORDING || (looperState != STATE_IDLE && loopLenTicks == 0)) {
              pendingLoadSlot = -1;
              loadPreset(slot);                       // как и раньше — сразу
            } else {
              // IDLE: ждём первую долю (triggerQuantizedAction)
              // PLAYING/OVERDUB: ждём конец лупа (playback)
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
          if (v < 0) v = 0;
          if (v > 100) v = 100;
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
const int buttonPins[11] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};   // S3-Zero: GPIO3..13
const int cc_list[11]    = {21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31};
const int cc_val[11]     = {127, 127, 127, 127, 127, 127, 127, 127, 127, 127, 127};

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
  // Загрузка пресета из IDLE — строго на первой доле
  if (pendingLoadSlot >= 1 && looperState == STATE_IDLE) {
    int slot = pendingLoadSlot;
    pendingLoadSlot = -1;
    pendingStart = false;
    pendingStop = false;
    preRollCount = 0;
    loadPreset(slot);      // в IDLE prevPos = 0, значит playStartTick = currentTick
    return;
  }

  if (pendingStart) {
    eventCount = 0; currentLayer = 0;
    recStartTick = currentTick;
    bleBeatMs = loopBeatDuration;                // опорный темп для BLE-клиента
    looperState = STATE_RECORDING;
    ccOdErased = false;
    ccResetOverdubState();
    resetActiveNoteTracking();
    pendingStart = false;
    sendBleNotify("/state RECORDING");

    // Ноты, сыгранные чуть раньше первой доли → пишем с начала лупа
    long offLen = (long)(quantUnitTicks() + 0.5f);
    if (offLen < MIN_NOTE_TICKS) offLen = MIN_NOTE_TICKS;
    for (int k = 0; k < preRollCount && eventCount < 5000; k++) {
      PreRollEv &p = preRoll[k];
      bool on = (p.type == 0x90 && p.d2 > 0);
      uint16_t ts = on ? 0 : (uint16_t)offLen;   // note-off до доли = короткая нота
      events[eventCount++] = { ts, p.type, p.d1, p.d2, p.ch, (byte)0, false };
      if (on) {
        setActiveNote(p.ch, p.d1, 0, 0);         // если нота ещё зажата — note-off обработается штатно
        sendBleNotify("/rec_noteon " + String(p.d1) + " 0");
      } else {
        clearActiveNote(p.ch, p.d1);
        sendBleNotify("/rec_noteoff " + String(p.d1) + " " + String(ticksToBleMs(ts)));
      }
    }
    preRollCount = 0;
  }
  else if (pendingStop) {
    uint32_t len = currentTick - recStartTick;
    if (len == 0) return;                        // защита от нулевого лупа
    if (len > 65000) len = 65000;
    loopLenTicks = len;
    playStartTick = currentTick;
    finalizeRecordedLoop();      // wrap событий за границей лупа + закрытие незакрытых нот
    looperState = STATE_PLAYING;
    pendingStop = false;
    sendBeatInfo();
    sendBleNotify("/state PLAYING");
  }
}

// ================== КЛОК / START / СГЛАЖИВАНИЕ BPM ==================

void silenceLooperNotes() {
  static uint8_t sent[16][16];
  memset(sent, 0, sizeof(sent));
  unsigned long now = millis();
  for (int i = 0; i < eventCount; i++) {
    const MidiEvent &e = events[i];
    if (e.type != 0x90 || e.d2 == 0 || !e.played || e.ch < 1 || e.ch > 16) continue;
    uint8_t c = e.ch - 1, byteIdx = e.d1 >> 3, bit = 1 << (e.d1 & 7);
    if (sent[c][byteIdx] & bit) continue;
    sent[c][byteIdx] |= bit;
    lastSentNoteTime[c][e.d1] = now;
    midiNoteOff(e.d1, 0, e.ch);
  }
}

void handleStart() {
  tickInBeat = 0;
  beatInBar = 0;
  currentTick = 0;
  startArmed = true;            // первый F8 после Start = тик 0
  lastBeatTime = millis();

  recStartTick = 0;
  playStartTick = 0;            // pos == 0

  silenceLooperNotes();         // до сброса played
  for (int i = 0; i < eventCount; i++) events[i].played = false;

  // looperState НЕ меняем.
  if (looperState == STATE_RECORDING) {
    // запись продолжается с нулевой позиции: старый дубль привязан к старой временной шкале
    eventCount = 0;
    currentLayer = 0;
    ccResetOverdubState();
    resetActiveNoteTracking();
    preRollCount = 0;
    sendBleNotify("/reset 1");
    sendBleNotify("/state RECORDING");
  }
  // OVERDUB: слои сохраняются, воспроизведение и запись идут с нуля.
  // Сегменты перезаписи CC начнутся заново (флаги "трогали" сохраняются — CC остаются заглушены)
  ccResetSweeps();

}

// Continue/Stop не пробрасываем обратно: источник клока — Zynthian, эхо создало бы петлю
void handleContinue() {
}

void handleStop() {
}

void updateTempo(uint32_t nowUs) {
  if (tickTimeCount >= BPM_MIN_SAMPLES) {
    uint32_t oldestUs = (tickTimeCount < BPM_WINDOW_TICKS) ? tickTimesUs[0] : tickTimesUs[tickTimeIdx];
    uint32_t span = nowUs - oldestUs;                   // tickTimeCount интервалов
    float tickUs = (float)span / (float)tickTimeCount;
    float bpm = 60000000.0f / (tickUs * (float)PPQN);

    if (bpm >= BPM_MIN && bpm <= BPM_MAX) {
      activeBPM = bpm;
      tickPeriodUs = (uint32_t)(tickUs + 0.5f);
      loopBeatDuration = (unsigned long)(tickUs * (float)PPQN / 1000.0f + 0.5f);
      beatDuration = loopBeatDuration;
    }
  }

  tickTimesUs[tickTimeIdx] = nowUs;
  tickTimeIdx = (tickTimeIdx + 1) % BPM_WINDOW_TICKS;
  if (tickTimeCount < BPM_WINDOW_TICKS) tickTimeCount++;
}

void advanceTick(unsigned long now) {
  if (startArmed) startArmed = false;
  else currentTick++;

  if (tickInBeat == 0) {
    lastBeatTime = now;
    if (beatInBar == 0) triggerQuantizedAction();
  }
  tickInBeat = tickInBeat + 1;
  if (tickInBeat >= PPQN) { tickInBeat = 0; beatInBar = (beatInBar + 1) % 4; }
}

void handleClock() {
  unsigned long now = millis();
  uint32_t nowUs = micros();

  if (now - lastClockTime > 300) tickTimeCount = 0;
  lastClockTime = now;

  updateTempo(nowUs);
  advanceTick(now);
}

void handleInternalClock() {
  static bool running = false;
  static uint32_t nextUs = 0;

  if (millis() - lastClockTime < 1200) { running = false; return; }   // внешний клок жив

  uint32_t nowUs = micros();
  uint32_t tickUs = tickPeriodUs;
  if (tickUs < 1000) tickUs = 1000;

  if (!running) { running = true; nextUs = nowUs + tickUs; return; }

  while ((int32_t)(nowUs - nextUs) >= 0) {   // догоняем пропущенные тики без дрейфа
    nextUs += tickUs;
    advanceTick(millis());
  }
}

void recordEvent(byte t, byte d1, byte d2, byte ch) {
  // Канал MIDICHANNEL (12) — канал управления: в лупер не пишем и обратно не отправляем
  // (иначе петля обратной связи через Zynthian)
  if (ch == MIDICHANNEL) return;
  if (ch == 11) return;   // 11-й канал — экспрессия, не пишем и не пробрасываем
  if (ch < 1 || ch > 16) return;
  unsigned long now = millis();

  if (t == 0xB0) {
    // фильтр эха от собственного воспроизведения CC
    if (now - lastSentCCTime[ch-1][d1] < 15) return;
    // живой перехват работает только вне овердаба (в овердабе CC глушится флагом ccTouched)
    if (looperState != STATE_OVERDUB) ccTakeover[ch][d1] = now;
  }
  if (t == 0xD0 || t == 0xA0) if (now - lastSentATTime[ch-1] < 15) return;

  bool isNoteStart = (t == 0x90 && d2 > 0);
  bool isNoteEnd   = (t == 0x80 || (t == 0x90 && d2 == 0));
  bool isRecOrOver = (looperState == STATE_RECORDING || looperState == STATE_OVERDUB);

  // Ждём старт записи — копим ноты, сыгранные чуть раньше первой доли
  if (looperState == STATE_IDLE && pendingStart && (isNoteStart || isNoteEnd)) {
    preRollAdd(t, d1, d2, ch, isNoteStart);
  }

  // Фильтр эха от собственного воспроизведения — ДО трекинга нот
  if (isRecOrOver && (t == 0x90 || t == 0x80) && (now - lastSentNoteTime[ch-1][d1] < 20)) return;

  uint16_t finalTs = 0;
  long rawL = 0;
  if (isRecOrOver) {
    bool isRec = (looperState == STATE_RECORDING);
    unsigned long loopLenLocal = isRec ? 0 : loopLenTicks;
    rawL = isRec ? (long)ticksSince(recStartTick) : (long)((currentTick - playStartTick) % loopLenTicks);
    if (rawL > 65000) rawL = 65000;

    bool qStart = (quantTarget == QUANT_START || quantTarget == QUANT_BOTH);
    bool qEnd   = (quantTarget == QUANT_END   || quantTarget == QUANT_BOTH);

    if (t == 0xB0) {
      finalTs = (uint16_t)rawL;      // CC НЕ квантуем: иначе плавный свип превращается в ступеньки
    }
    else if (isNoteStart) {
      long qL = qStart ? quantizeLogical(rawL, loopLenLocal) : rawL;
      setActiveNote(ch, d1, rawL, qL);
      finalTs = wrapTs(qL, loopLenLocal);
    }
    else if (isNoteEnd) {
      long onRaw = 0, onQ = 0;
      if (takeActiveNote(ch, d1, onRaw, onQ)) {
        long offRaw = rawL;
        if (!isRec && offRaw < onRaw) offRaw += (long)loopLenTicks;   // нота перешла через границу лупа
        long offL = resolveNoteEnd(onRaw, onQ, offRaw, loopLenLocal, qEnd);
        finalTs = wrapTs(offL, loopLenLocal);
      } else {
        finalTs = qEnd ? quantizeTimestamp((uint16_t)rawL, loopLenLocal) : (uint16_t)rawL;
      }
    }
    else {
      finalTs = (uint16_t)rawL;
    }

    // ---- CC в овердабе: стираем старое там, где прошла ручка ----
    if (t == 0xB0 && looperState == STATE_OVERDUB && loopLenTicks > 0) {
      uint32_t pos = (uint32_t)finalTs % loopLenTicks;
      int si = ccFindSweep(ch, d1);
      int32_t dt = (si >= 0) ? (int32_t)(currentTick - ccSweeps[si].tick) : -1;
      bool cont = (si >= 0) && ((uint32_t)(now - ccSweeps[si].ms) <= (uint32_t)CC_OVERDUB_GAP_MS) && (dt >= 0);
      if (cont) {
        if (dt > 0) eraseCCRange(ch, d1, (uint32_t)ccSweeps[si].pos + 1, (uint32_t)dt);   // (lastPos .. pos]
        // dt == 0: тот же тик, уже стёрт — значение заменит дедупликация ниже
      } else {
        eraseCCRange(ch, d1, pos, 1);                                                     // новый сегмент
      }
      if (si < 0) si = ccAllocSweep(ch, d1);
      ccSweeps[si].ms = (uint32_t)now;
      ccSweeps[si].tick = currentTick;
      ccSweeps[si].pos = (uint16_t)pos;
      ccSetTouched(ch, d1);
      ccOdErased = true;
    }
  }

  // BLE: поток CC ограничиваем (иначе notify тормозит основной цикл и клок/приём MIDI)
  bool bleOk = true;
  if (t == 0xB0) {
    static unsigned long lastCcBle = 0;
    if (now - lastCcBle < 40) bleOk = false; else lastCcBle = now;
  }

  if (deviceConnected && bleOk) {
    if (looperState == STATE_RECORDING) {
      uint32_t ms = ticksToBleMs(finalTs);                 // клиент ждёт мс
      if (t == 0xB0) sendBleNotify("/rec_cc " + String(ms));
      else if (isNoteStart) sendBleNotify("/rec_noteon " + String(d1) + " " + String(ms));
      else if (isNoteEnd) sendBleNotify("/rec_noteoff " + String(d1) + " " + String(ms));
    }
    else {
      float posNorm = 0.0;
      if (loopLenTicks > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
        if (looperState == STATE_OVERDUB) {
          posNorm = (float)finalTs / (float)loopLenTicks;
        } else {
          posNorm = (float)((currentTick - playStartTick) % loopLenTicks) / (float)loopLenTicks;
        }
      }

      if (t == 0xB0) sendBleNotify("/cc " + String(posNorm, 4));
      else if (t == 0x90 && d2 > 0) sendBleNotify("/noteon " + String(d1) + " " + String(posNorm, 4));
      else if (t == 0x80 || (t == 0x90 && d2 == 0)) sendBleNotify("/noteoff " + String(d1) + " " + String(posNorm, 4));
    }
  }

  if (!isRecOrOver) return;

  // Событие, сыгранное вживую прямо сейчас в овердабе, НЕ должно сразу проигрываться лупером ещё раз.
  // CC — всегда "уже сыграно"; ноты — если их время уже не в будущем (квантованные вперёд сыграются позже).
  bool alreadySounded = false;
  if (looperState == STATE_OVERDUB) {
    alreadySounded = (t == 0xB0) || ((uint32_t)finalTs <= (uint32_t)rawL);
  }

  // CC: в одном тике остаётся одно значение (последнее) — иначе поток ручки забивает events[]
  if (t == 0xB0) {
    int lim = eventCount > 64 ? eventCount - 64 : 0;
    for (int i = eventCount - 1; i >= lim; i--) {
      MidiEvent &e = events[i];
      if (e.type == 0xB0 && e.ch == ch && e.d1 == d1 && e.timestamp == finalTs && e.layer == currentLayer) {
        e.d2 = d2;
        e.played = alreadySounded;
        return;
      }
    }
  }

  if (eventCount < 5000) {
    events[eventCount++] = {finalTs, t, d1, d2, ch, currentLayer, alreadySounded};
  }
}
// ===== MIDI-ВХОД: USB и/или DIN =====
void usbMidiPoll() {
#if USE_USB_MIDI
  for (int n = 0; n < 64; n++) {
    midiEventPacket_t p = {0, 0, 0, 0};
    usbMidi.readPacket(&p);
    if (p.header == 0) break;                    // пакетов больше нет
    byte cin = p.header & 0x0F;
    byte ch = (p.byte1 & 0x0F) + 1;
    switch (cin) {
      case 0x8: recordEvent(0x80, p.byte2, p.byte3, ch); break;
      case 0x9:
        if (p.byte3 == 0) recordEvent(0x80, p.byte2, 0, ch);   // note-on с velocity 0 = note-off
        else recordEvent(0x90, p.byte2, p.byte3, ch);
        break;
      case 0xB: recordEvent(0xB0, p.byte2, p.byte3, ch); break;
      case 0xF:                                  // однобайтовые realtime
        if (p.byte1 == 0xF8) handleClock();
        else if (p.byte1 == 0xFA) handleStart();
        else if (p.byte1 == 0xFB) handleContinue();
        else if (p.byte1 == 0xFC) handleStop();
        break;
      default: break;
    }
  }
#endif
}

void midiPollInput() {
#if USE_DIN_MIDI
  while (MIDI.read()) { }
#endif
  usbMidiPoll();
}

void playback() {
  if ((looperState != STATE_PLAYING && looperState != STATE_OVERDUB) || loopLenTicks == 0) return;
  unsigned long now = millis();                                   // только для CC-takeover и echo-guard
  uint32_t pos = (currentTick - playStartTick) % loopLenTicks;    // позиция следует за входящим клоком
  static uint32_t lastPos = 0;

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
        byte c = events[i].ch, n = events[i].d1;
        // в овердабе CC, которые трогали, молчат до выхода из овердаба
        if (looperState == STATE_OVERDUB && ccIsTouched(c, n)) { events[i].played = true; continue; }
        // живой перехват ручкой вне овердаба
        if (now - ccTakeover[c][n] < 1200) { events[i].played = true; continue; }
        lastSentCCTime[c-1][n] = now;
      }
      if (events[i].type == 0x90 || events[i].type == 0x80) lastSentNoteTime[events[i].ch-1][events[i].d1] = now;

      midiSendMsg((midi::MidiType)events[i].type, events[i].d1, events[i].d2, events[i].ch);
      events[i].played = true;
    }
  }
  lastPos = pos;
}

// Педаль: неблокирующий debounce
void handleButton() {
  byte btn = digitalRead(LOOPER_PIN); unsigned long now = millis();
  static byte lastReading = HIGH, stable = HIGH;
  static unsigned long lastChange = 0;

  if (btn != lastReading) {
    lastReading = btn;
    lastChange = now;
    if (btn == LOW) btnTime = now;     // btnTime используется и в updateLED()
  }

  if (btn != stable && now - lastChange >= 20) {
    stable = btn;
    if (stable == LOW) { btnTime = now; ignoreRel = false; }
    else if (!ignoreRel) {
      if (looperState == STATE_IDLE) { pendingLoadSlot = -1; preRollCount = 0; pendingStart = true; }
      else if (looperState == STATE_RECORDING) pendingStop = true;
      else if (looperState == STATE_PLAYING) {
        currentLayer++;
        looperState = STATE_OVERDUB;
        sendBleNotify("/state OVERDUB");
      }
      else if (looperState == STATE_OVERDUB) {
        endOverdub();
      }
    }
  }

  if (stable == LOW && btn == LOW && !ignoreRel && (now - btnTime > 1500)) {
    pendingStart = false; pendingStop = false; ignoreRel = true;
    resetLooper();
  }
}
void handleExpression() {
  if (millis() % 20 == 0) {
    int v1 = constrain(map(filter1(analogRead(EXPPIN1) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v1 - lastValExp) > 1 || (v1 == 0 && lastValExp != 0) || (v1 == 127 && lastValExp != 127)) { midiCC(30, v1, 11); lastValExp = v1; }

    int v2 = constrain(map(filter2(analogRead(EXPPIN2) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v2 - lastValExp2) > 1 || (v2 == 0 && lastValExp2 != 0) || (v2 == 127 && lastValExp2 != 127)) { midiCC(31, v2, 11); lastValExp2 = v2; }
  }
}
void handleFootswitchButtons() {
  unsigned long now = millis();
  for(int i=0; i<11; i++) {
    byte current_reading = digitalRead(buttonPins[i]);
    if (current_reading != last_btn_logical_state[i]) {
      if (now - last_debounce[i] > 50) {
        last_btn_logical_state[i] = current_reading;
        last_debounce[i] = now;
        if (current_reading == LOW) {
          midiCC(cc_list[i], cc_val[i], MIDICHANNEL);
        } else {
          // Отпускание: 0, чтобы приёмник (Audio Looper) мог отличить тап от удержания
          midiCC(cc_list[i], 0, MIDICHANNEL);
        }
      }
    }
  }
}

// --- PWM для LED: совместимость с ESP32 Arduino core 2.x и 3.x ---
#define LED_PWM_CHANNEL 0

void ledPwmInit() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(LED_PIN, pwmFreq, pwmRes);            // core 3.x
#else
  ledcSetup(LED_PWM_CHANNEL, pwmFreq, pwmRes);     // core 2.x
  ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
#endif
}

void ledPwmWrite(int duty) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(LED_PIN, duty);                        // core 3.x: по пину
#else
  ledcWrite(LED_PWM_CHANNEL, duty);                // core 2.x: по каналу
#endif
}

int softPulse(unsigned long now, unsigned long period, int minB, int maxB) {
  float ph = (float)(now % period) / (float)period;
  float s = 0.5f - 0.5f * cosf(ph * 6.2831853f);   // 0..1 синус
  s = s * s;                                       // гамма, чтобы низ был мягче
  return minB + (int)(s * (float)(maxB - minB));
}

void updateLED() {
  unsigned long now = millis(); unsigned long diff = now - lastBeatTime; int brightness = 0;

  if (digitalRead(LOOPER_PIN) == LOW && !ignoreRel && (now - btnTime > 500)) {
    brightness = softPulse(now, 320, 3, 100);
  }
  else if (pendingStart || pendingStop || pendingLoadSlot >= 1) {
    brightness = softPulse(now, 700, 2, 55);
  }
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
  ledPwmWrite(brightness);
}
void setup() {
  LittleFS.begin(true);
  resetActiveNoteTracking();
  ccResetOverdubState();
  ledPwmInit();
  pinMode(LOOPER_PIN, INPUT_PULLUP);
  for(int i=0; i<11; i++) pinMode(buttonPins[i], INPUT_PULLUP);

#if USE_DIN_MIDI
  MIDI_Serial.begin(31250, SERIAL_8N1, MIDI_RX_PIN, MIDI_TX_PIN);
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();

  MIDI.setHandleClock(handleClock);
  MIDI.setHandleStart(handleStart);
  MIDI.setHandleContinue(handleContinue);
  MIDI.setHandleStop(handleStop);

  MIDI.setHandleNoteOn([](byte ch, byte n, byte v) { recordEvent(0x90, n, v, ch); });
  MIDI.setHandleNoteOff([](byte ch, byte n, byte v) { recordEvent(0x80, n, v, ch); });
  MIDI.setHandleControlChange([](byte ch, byte n, byte v) { recordEvent(0xB0, n, v, ch); });
#endif
#if USE_USB_MIDI
  USB.productName("MIDI Footswitch");   // имя устройства в Zynthian (если строка не компилируется — удалите её)
  usbMidi.begin();
  USB.begin();
#endif
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
  midiPollInput();
  handleInternalClock();
  handleButton();
  handleFootswitchButtons();
  handleExpression();

  // Выход из овердаба (педаль, BLE, undo, reset, load): сбрасываем состояние перезаписи CC,
  // после этого CC снова воспроизводятся
  static LooperState prevLooperState = STATE_IDLE;
  if (prevLooperState == STATE_OVERDUB && looperState != STATE_OVERDUB) onOverdubExit();
  prevLooperState = looperState;

  playback();
  serviceCcRefresh();
  updateLED();

  // Темп для BLE-клиента (только когда нет лупа). Порог 0.5 BPM — защита от спама.
  static float lastReportedBpm = 0.0f;
  if (looperState == STATE_IDLE && loopLenTicks == 0 && fabsf(activeBPM - lastReportedBpm) >= 0.5f) {
    lastReportedBpm = activeBPM;
    bleBeatMs = loopBeatDuration;
    sendBeatInfo();
  }

  if (needSendPresetStatus && deviceConnected && millis() - connectStatusTime > 400) {
    if (looperState == STATE_IDLE && loopLenTicks == 0) bleBeatMs = loopBeatDuration;
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
        sendBleNotify("/rec_pos " + String(ticksToBleMs(events[visualDumpIndex - 1].timestamp + 1)));
      }

      if (nowDump - lastVisualDumpTime >= VISUAL_DUMP_EVENT_INTERVAL) {
        lastVisualDumpTime = nowDump;
        MidiEvent &e = events[visualDumpIndex];
        uint32_t evMs = ticksToBleMs(e.timestamp);
        if (e.type == 0x90 && e.d2 > 0) {
          sendBleNotify("/rec_noteon " + String(e.d1) + " " + String(evMs));
        } else if (e.type == 0x80 || (e.type == 0x90 && e.d2 == 0)) {
          sendBleNotify("/rec_noteoff " + String(e.d1) + " " + String(evMs));
        } else if (e.type == 0xB0) {
          sendBleNotify("/rec_cc " + String(evMs));
        }
        visualDumpIndex++;
      }
    }
    else if (visualDumpFinishStage == 0) {
      sendBleNotify("/rec_pos " + String(ticksToBleMs(loopLenTicks)));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 1;
      visualDumpFinishTime = nowDump;
    }
    else if (visualDumpFinishStage == 1 && nowDump - visualDumpFinishTime >= 20) {
      sendBleNotify("/rec_pos " + String(ticksToBleMs(loopLenTicks)));
      sendBleNotify("/state " + stateName());
      visualDumpFinishStage = 0;
      visualDumpActive = false;
    }
  }

  if (deviceConnected && !visualDumpActive && (millis() - lastBleUpdate >= BLE_INTERVAL)) {
    lastBleUpdate = millis();
    if (loopLenTicks > 0 && (looperState == STATE_PLAYING || looperState == STATE_OVERDUB)) {
      uint32_t currentPos = (currentTick - playStartTick) % loopLenTicks;
      float posNorm = (float)currentPos / (float)loopLenTicks;
      sendBleNotify("/pos " + String(posNorm, 4));
    }
    else if (looperState == STATE_RECORDING) {
      sendBleNotify("/rec_pos " + String(ticksToBleMs(ticksSince(recStartTick))));
    }
    else if (looperState == STATE_IDLE) {
      float idlePosNorm = (float)(beatInBar * PPQN + tickInBeat) / (float)BAR_TICKS;
      sendBleNotify("/pos " + String(idlePosNorm, 4));
    }
  }
}