// v26 + comments
#include <HardwareSerial.h>
#include <MIDI.h>

// --- НАСТРОЙКИ ПИНОВ И ПАРАМЕТРОВ ---
#define LOOPER_PIN 18      // Пин основной кнопки управления MIDI лупером
#define LED_PIN 2          // Пин светодиода состояния / ритма (ШИМ)
#define MIDICHANNEL 12     // MIDI-канал для команд контроллера Aeros
#define EXPPIN1 34         // АЦП пин первой педали экспрессии
#define EXPPIN2 35         // АЦП пин второй педали экспрессии

// Инициализация второго аппаратного UART для работы с MIDI (31250 baud)
HardwareSerial MIDI_Serial(2);
MIDI_CREATE_INSTANCE(HardwareSerial, MIDI_Serial, MIDI); 

// Параметры генерации ШИМ для светодиода (ESP32 LEDC)
const int pwmFreq = 5000;  // Частота ШИМ: 5 кГц
const int pwmRes = 8;      // Разрешение: 8 бит (значения от 0 до 255)

// --- ПЕРЕМЕННЫЕ КОНТРОЛЛЕРА ---
// Пины 11 кнопок (от нижней правой к верхней левой, за исключением кнопки лупера)
const int buttonPins[11] = {19, 23, 5, 13, 12, 14, 27, 25, 26, 32, 33}; 
// Номера MIDI CC для каждой из 11 кнопок
const int cc_list[11]    = {46, 47, 38, 41, 41, 41, 42, 35, 35, 37, 40};
// Отправляемые значения CC для каждой из 11 кнопок
const int cc_val[11]     = {0, 1, 100, 100, 0, 104, 0, 1, 5, 0, 0};

// Массивы для программной защиты от дребезга контактов (Debounce)
unsigned long last_debounce[11] = {0};
bool last_btn_logical_state[11] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; 

// --- ПЕРЕМЕННЫЕ И ФИЛЬТРЫ ПЕДАЛЕЙ ЭКСПРЕССИИ ---
int lastValExp = 0, lastValExp2 = 0; // Последние отправленные значения CC педалей

// Настройки адаптивного фильтра Калмана для первой педали
float errmeasure = 40, errestimate = 40, q = 0.5;
float currentestimate = 0.0, lastestimate = 0.0, kalmangain = 0.0;

// Настройки адаптивного фильтра Калмана для второй педали
float errmeasure2 = 40, errestimate2 = 40, q2 = 0.5;
float currentestimate2 = 0.0, lastestimate2 = 0.0, kalmangain2 = 0.0;

// Вычисление фильтра Калмана для первого входа (убирает шумы АЦП)
float filter1(int value) {
  kalmangain = errestimate / (errestimate + errmeasure);
  currentestimate = lastestimate + kalmangain * (value - lastestimate);
  errestimate = (1.0 - kalmangain) * errestimate + fabs(lastestimate - currentestimate) * q;
  lastestimate = currentestimate; return currentestimate;
}

// Вычисление фильтра Калмана для второго входа
float filter2(int value2) {
  kalmangain2 = errestimate2 / (errestimate2 + errmeasure2);
  currentestimate2 = lastestimate2 + kalmangain2 * (value2 - lastestimate2);
  errestimate2 = (1.0 - kalmangain2) * errestimate2 + fabs(lastestimate2 - currentestimate2) * q2;
  lastestimate2 = currentestimate2; return currentestimate2;
}

// --- ДВИЖОК ЛУПЕРА ---
// Состояния автомата лупера: Покой, Запись, Воспроизведение, Наложение
enum LooperState { STATE_IDLE, STATE_RECORDING, STATE_PLAYING, STATE_OVERDUB };
LooperState looperState = STATE_IDLE;

// Структура записанного MIDI-события (размер подогнан под экономию ОЗУ)
struct MidiEvent { uint16_t timestamp; byte type, d1, d2, ch, layer; bool played; };
MidiEvent events[5000]; // Буфер памяти максимум на 5000 MIDI-событий

int eventCount = 0;         // Текущее количество записанных событий
byte currentLayer = 0;      // Индекс текущего слоя наложения (Overdub)
byte ccMaxLayer[17][128];   // Фиксация максимального слоя для контроллеров

// Временные метки и флаги работы лупера
unsigned long loopLen = 0, recStart = 0, playStart = 0, btnTime = 0;
bool ignoreRel = false;

// Защитные массивы от «залипания» и конфликтов сообщений (для каждого канала/ноты)
unsigned long lastSentNoteTime[16][128], lastSentCCTime[16][128], lastSentATTime[16], ccTakeover[17][128]; 

// --- СИНХРОНИЗАЦИЯ С MIDI CLOCK ---
volatile bool pendingStart = false; // Флаг ожидания квантованного старта записи
volatile bool pendingStop = false;  // Флаг ожидания квантованного завершения записи
volatile unsigned long lastBeatTime = 0;   // Время последнего удара (Beat)
volatile unsigned long lastClockTime = 0;  // Время последнего входящего тика MIDI Clock
volatile int beatInBar = 0;       // Доля в такте (0..3 для размера 4/4)
volatile int tickInBeat = 0;      // Тик внутри доли (24 тика на одну долю)
unsigned long beatDuration = 545; // Длительность доли в мс (по умолчанию ~110 BPM)

// Выполнение квантованных действий, привязанных к началу такта (Beat 0)
void triggerQuantizedAction() {
  unsigned long now = millis();
  if (pendingStart) {
    // Переход в режим первой записи петли
    eventCount = 0; currentLayer = 0; recStart = now; 
    looperState = STATE_RECORDING;
    for(int i=0; i<17; i++) for(int j=0; j<128; j++) ccMaxLayer[i][j] = 0;
    pendingStart = false;
  } 
  else if (pendingStop) {
    // Завершение первой записи и переход к воспроизведению
    loopLen = now - recStart; playStart = now; 
    looperState = STATE_PLAYING;
    pendingStop = false;
  }
}

// Обработчик команды MIDI Start/Continue
void handleStart() { tickInBeat = 0; beatInBar = 0; lastBeatTime = millis(); }

// Обработчик входящего тика MIDI Clock (24 тика на 1 долю)
void handleClock() { 
  unsigned long now = millis();
  lastClockTime = now;
  if (tickInBeat == 0) { 
    beatDuration = now - lastBeatTime; 
    lastBeatTime = now; 
    if (beatInBar == 0) triggerQuantizedAction(); // Старт/стоп ровно на первую долю такта
  }
  tickInBeat++;
  if (tickInBeat >= 24) { tickInBeat = 0; beatInBar = (beatInBar + 1) % 4; }
}

// Запись входящих MIDI-событий в буфер лупера
void recordEvent(byte t, byte d1, byte d2, byte ch) {
  if (ch == 11 || ch == 12) return; // Игнорируем служебные каналы контроллера
  unsigned long now = millis();
  
  // Защита от дублирования CC и конфликтных перезаписей
  if (t == 0xB0) { if (now - lastSentCCTime[ch-1][d1] < 15) return; ccTakeover[ch][d1] = now; ccMaxLayer[ch][d1] = currentLayer; }
  if (t == 0xD0 || t == 0xA0) if (now - lastSentATTime[ch-1] < 15) return;
  if (looperState != STATE_RECORDING && looperState != STATE_OVERDUB) return;
  if ((t == 0x90 || t == 0x80) && (now - lastSentNoteTime[ch-1][d1] < 20)) return;
  
  // Сохранение события с временным штампом
  if (eventCount < 5000) {
    unsigned long rawTs = (looperState == STATE_RECORDING) ? (now - recStart) : ((now - playStart) % loopLen);
    events[eventCount++] = {(uint16_t)(rawTs > 65000 ? 65000 : rawTs), t, d1, d2, ch, currentLayer, false};
  }
}

// Воспроизведение записанных MIDI-событий по кругу
void playback() {
  if (looperState != STATE_PLAYING && looperState != STATE_OVERDUB) return;
  unsigned long now = millis();
  unsigned long pos = (now - playStart) % loopLen; // Текущая позиция внутри цикла
  static unsigned long lastPos = 0;
  
  // Сброс флагов проигрывания при переходе на новый круг (зацикливание)
  if (pos < lastPos) for (int i = 0; i < eventCount; i++) events[i].played = false;
  
  for (int i = 0; i < eventCount; i++) {
    if (events[i].played) continue;
    if (pos >= events[i].timestamp) {
      // Проверки приоритетов слоев для CC команд
      if (events[i].type == 0xB0) {
        if (now - ccTakeover[events[i].ch][events[i].d1] < 1200) { events[i].played = true; continue; }
        if (events[i].layer < ccMaxLayer[events[i].ch][events[i].d1]) { events[i].played = true; continue; }
        lastSentCCTime[events[i].ch-1][events[i].d1] = now;
      }
      if (events[i].type == 0x90 || events[i].type == 0x80) lastSentNoteTime[events[i].ch-1][events[i].d1] = now;
      
      // Отправка записанной MIDI ноты / CC на выход
      MIDI.send((midi::MidiType)events[i].type, events[i].d1, events[i].d2, events[i].ch);
      events[i].played = true;
    }
  }
  lastPos = pos;
}

// Обработка единственной кнопки управления MIDI-лупером (GPIO 18)
void handleButton() {
  byte btn = digitalRead(LOOPER_PIN); unsigned long now = millis(); static byte lastBtn = HIGH;
  
  // Внутренний генератор такта (работает, если нет внешнего MIDI Clock > 1.2 сек)
  if (now - lastClockTime >= 1200) { if (now - lastBeatTime >= 545) { lastBeatTime = now; if (beatInBar == 0) triggerQuantizedAction(); beatInBar = (beatInBar + 1) % 4; } }
  
  // Фиксация нажатий/отпусканий кнопки
  if (btn != lastBtn) {
    delay(20); // Задержка от дребезга
    if (btn == LOW) { btnTime = now; ignoreRel = false; }
    else if (!ignoreRel) {
      // Переключение состояний по короткому клику
      if (looperState == STATE_IDLE) pendingStart = true;
      else if (looperState == STATE_RECORDING) pendingStop = true;
      else if (looperState == STATE_PLAYING) { currentLayer++; looperState = STATE_OVERDUB; }
      else if (looperState == STATE_OVERDUB) looperState = STATE_PLAYING;
    }
    lastBtn = btn;
  }
  
  // Долгое удержание (> 1.5 сек) — полная очистка памяти и сброс лупера
  if (btn == LOW && !ignoreRel && (now - btnTime > 1500)) {
    for (int i = 0; i < eventCount; i++) if (events[i].type == 0x90) MIDI.sendNoteOff(events[i].d1, 0, events[i].ch);
    looperState = STATE_IDLE; eventCount = 0; currentLayer = 0; pendingStart = false; pendingStop = false; ignoreRel = true;
  }
}

// Считывание и фильтрация аналоговых педалей экспрессии
void handleExpression() {
  if (millis() % 20 == 0) { // Опрос каждые 20 мс
    // Педаль 1
    int v1 = constrain(map(filter1(analogRead(34) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v1 - lastValExp) > 1 || (v1 == 0 && lastValExp != 0) || (v1 == 127 && lastValExp != 127)) { MIDI.sendControlChange(30, v1, 11); lastValExp = v1; }
    
    // Педаль 2
    int v2 = constrain(map(filter2(analogRead(35) >> 2), 0, 1022, 0, 127), 0, 127);
    if (abs(v2 - lastValExp2) > 1 || (v2 == 0 && lastValExp2 != 0) || (v2 == 127 && lastValExp2 != 127)) { MIDI.sendControlChange(31, v2, 11); lastValExp2 = v2; }
  }
}

// Обработка 11 кнопок контроллера Aeros
void handleAerosController() {
  unsigned long now = millis();
  for(int i=0; i<11; i++) {
    byte current_reading = digitalRead(buttonPins[i]);
    
    // Если состояние кнопки изменилось (физически)
    if (current_reading != last_btn_logical_state[i]) {
      // И прошло достаточно времени с прошлого изменения (антидребезг)
      if (now - last_debounce[i] > 50) { 
        last_btn_logical_state[i] = current_reading;
        last_debounce[i] = now;

        // Отправляем MIDI только в момент нажатия (LOW)
        if (current_reading == LOW) {
          if (cc_list[i] == 42) MIDI.sendControlChange(43, 127, MIDICHANNEL);
          MIDI.sendControlChange(cc_list[i], cc_val[i], MIDICHANNEL);
          if (cc_list[i] == 42) MIDI.sendControlChange(43, 1, MIDICHANNEL);
        }
      }
    }
  }
}

// Индикация состояний лупера и такта пульсацией светодиода (ШИМ)
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

// Инициализация периферии микроконтроллера
void setup() {
  ledcAttach(LED_PIN, pwmFreq, pwmRes); // Настройка генератора ШИМ для светодиода
  pinMode(LOOPER_PIN, INPUT_PULLUP);
  for(int i=0; i<11; i++) pinMode(buttonPins[i], INPUT_PULLUP);
  
  // Запуск MIDI на пинах 16 (RX) и 17 (TX)
  MIDI_Serial.begin(31250, SERIAL_8N1, 16, 17);
  MIDI.begin(MIDI_CHANNEL_OMNI); MIDI.turnThruOff();
  
  // Регистрация коллбэков для обработки входящего MIDI-потока
  MIDI.setHandleClock(handleClock); MIDI.setHandleStart(handleStart); MIDI.setHandleContinue(handleStart);
  MIDI.setHandleNoteOn([](byte ch, byte n, byte v) { recordEvent(0x90, n, v, ch); });
  MIDI.setHandleNoteOff([](byte ch, byte n, byte v) { recordEvent(0x80, n, v, ch); });
  MIDI.setHandleControlChange([](byte ch, byte n, byte v) { recordEvent(0xB0, n, v, ch); });
}

// Главный бесконечный цикл программы
void loop() {
  while (MIDI.read()) { } // Вычитка входящих MIDI сообщений
  handleButton();          // Обработка кнопки лупера
  handleAerosController(); // Обработка 11 кнопок Aeros
  handleExpression();      // Обработка педалей экспрессии
  playback();              // Проигрывание MIDI-цикла
  updateLED();             // Обновление состояния светодиода
}
