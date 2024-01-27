These are the project files for the Sagittarius wireless headtracker.

A bit of background:
This project started off as a way to find a cheap high performance headtracker, but it didn't take me long to find out that a cheap and simple wireless headtracker would be more suited for my needs.
I started off with some cheap IMU modules and an ESP32, however most of these sensors were fakes because the real ones weren't in production anymore. They worked, but they had issues with drift and low dynamic accuracy.
The first prototype that actually worked well used a BNO085 sensor breakout board from Adafruit and an ESP32-S2 Wemos board to process and transmit the data over WiFi to Opentrack that was using the UDP input.

Do not that everything you will find here is not a complete and ready to go device you can just build. Knowledge about electronics, embedded software and hardware design is REQUIRED to get anything out of this. After all, this more of a dump of my knowledge, findings and what I've made than a complete project.
The following thins might be of interest if you are looking to make something like this:
- PCB Schematics
- Sagittarius S1 Firmware
  - Data transmit function (packet construction)
  - Quaternion processing
