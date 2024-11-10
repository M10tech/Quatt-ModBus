(c) 2024 M10tech

# Quatt ModBus monitor tool

use the LCM4ESP32 concept to run this

under developement so don't expect stability or anything

## Version History

### 0.1.0 must feed WatchDog more often
- increasing ping interval will trigger WDT all the time
- now ping is sent every second
- no point in reporting good pings anymore, only failed ones

### 0.0.9 RTC watchdog enable
- to force restart when hard crash
- ping-response feeds the watchdog

### 0.0.8 ping guard
- tends to crash when Quatt starts to do real work
- when no more ping response, resets

### 0.0.7 add time stamps
- fix first register report

### 0.0.6 read match tracking
- must link read command and answer

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

