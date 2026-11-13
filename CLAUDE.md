# Footswitch (ESP32-S3-Zero)

DIY MIDI-футсвитч и MIDI-лупер. Управляет Zynthian по MIDI. Прошивка на PlatformIO (Arduino framework,
платформа pioarduino 55.03.39, Arduino core 3.x). Старый проект (D1 R32, Aeros) в архиве: github.com/dmtry3/_old_footswitch.

## Сборка и прошивка
- Сборка: `pio run`
- Прошивка: `pio run -t upload` (первый раз зажать BOOT на плате, подключить USB, потом Upload)
- Зависимость: только `fortyseveneffects/MIDI Library` (lib_deps), BLE и LittleFS берутся из ядра
- Режим USB: `ARDUINO_USB_MODE=0` (TinyUSB), нужен для USB MIDI; не менять без причины

## Плата и пины (Waveshare ESP32-S3-Zero)
- Кнопки 1–11: GPIO3..13, CC 21–31, канал 12 (нажатие 127, отпускание 0)
- Педаль лупера: GPIO14
- Экспрессия 1/2: GPIO1/GPIO2 (CC 30/31, канал 11)
- Светодиод: GPIO15 (внешний)
- DIN MIDI: RX GPIO44, TX GPIO43 (UART1)
- GPIO19/20 заняты под USB, GPIO0 — кнопка BOOT

## Код
- Весь код в `src/main.cpp`. Версию веди комментарием в шапке файла (vNN) и добавляй строку при каждом изменении.
- Любая отправка MIDI идёт через `midiSendMsg`, `midiCC`, `midiNoteOff`: они шлют одновременно в USB и DIN.
  Не вызывай `MIDI.send...` напрямую.
- Флаги `USE_USB_MIDI` и `USE_DIN_MIDI` в начале файла включают и выключают каналы.
- Канал 12 — канал управления: в лупер не пишется и обратно не отправляется.
  Клок и Start/Stop/Continue обратно в Zynthian не пробрасываем. Иначе будет петля обратной связи.
- Пресеты хранятся в LittleFS, BLE-сервер нужен для приложения лупера.

## Как работать
- Перед правкой убедись, что рабочее состояние закоммичено.
- После правки запусти `pio run` и исправь ошибки сборки. Прошивает плату и проверяет на железе человек.
- Отвечай по-русски.
