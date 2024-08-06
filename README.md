(c) 2024 M10tech

# Quatt ModBus monitor tool

use the LCM4ESP32 concept to run this

under developement so don't expect stability or anything

## Version History

### 0.0.2 Add CRC check
- if OK only print CRC_OK
- if failed show received != calculated

### 0.0.1 initial version
- displays received ModBus commands and responses
- uses UDPlogger to observe remotely
- initial sdkconfig for esp32c3

