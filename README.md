# LoRa_SSDV

This repository contains the firmware for three different parts of a LoRa based high altitude balloon tracking system which also allows transmitting [SSDV](https://ukhas.org.uk/guides:ssdv) images. Included is a Python based GUI Gateway to [Habitat](http://habitat.habhub.org/) and [SSDV](https://ssdv.habhub.org/newindex.php) servers. A detailed description can be found in [this](http://tt7hab.blogspot.cz/2018/04/the-lora-ssdv.html) blog post.

##### Payload
<img src="/docs/IMG_2156_payload_testing.JPG" height="270" width="360">

##### Handheld
<img src="/docs/IMG_2075_handheld_gps_lock.JPG" height="270" width="360">

##### Station
<img src="/docs/IMG_2173_station_hardware.JPG" height="270" width="360">

##### Gateway
<img src="/docs/gateway_06_full_03.png" height="167" width="289">|
<img src="/docs/gateway_06_full_02.jpg" height="167" width="289">
|:---:|:---:|

### Installation
- The three pieces of hardware are all Arduino ProMini based, so the firmware is installed via the Arduino IDE.
- The LoRa_Gateway, aside from all the imported Python modules, expects a compiled command line app SSDV.exe in the same folder. The source code for the app can be found in Philip Heron's Github [repository](https://github.com/fsphil/ssdv).
