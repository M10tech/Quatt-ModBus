(c) 2024 M10tech

# Quatt ModBus monitor tool

use the LCM4ESP32 concept to run this

under developement so don't expect stability or anything

## Version History


### 0.0.4 bad logic
- memcmp match is 0, not 1

### 0.0.3 compress known items 
- all known items go on one line
- all unknown items get reported separately

### 0.0.2 Add CRC check
- if OK only print CRC_OK
- if failed show received != calculated

### 0.0.1 initial version
- displays received ModBus commands and responses
- uses UDPlogger to observe remotely
- initial sdkconfig for esp32c3

