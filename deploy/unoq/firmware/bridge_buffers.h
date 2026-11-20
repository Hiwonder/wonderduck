#pragma once

// Arduino's serial objects are compiled with the sketch. A complete HX
// broadcast reply and its MessagePack envelope exceed the default 64 bytes.
#undef CONFIG_ARDUINO_API_SERIAL_BUFFER_SIZE
#define CONFIG_ARDUINO_API_SERIAL_BUFFER_SIZE 2048
