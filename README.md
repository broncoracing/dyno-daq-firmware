Bronco Racing Dyno zDAQ

Info:
  - HX711 2 channel scale amplifier input (80sps)
  - Output multiplies HX711 reading by 1000 (keeps 3 decimal places)
  - CAN bus alive frame generation (50ms)
  - CAN bus data interpretation
  - Data transfer to MATLAB over serial
  - Serial routed to Nucleo L432KC onboard micro USB port @ 921600 baud
  - Current serial output rate is 80sps clocked by the HX711 chip.
  - Send 0-180 degrees over serial as uint8 to control servo.

Pins:
  - Servo: A2
  - Solenoid: D1
  - HX711 (SCK, DT): D11, D9
  - CAN transciever (RD, TD): D10, D2
    - RD -> CANRX, TD -> CANTX on transciever
    - These do not switch like serial

How to use:
    - Install VSCode and PlatformIO extension
    - Ensure STM32 L432KC onboard ST-Link has been reflashed with J-Link
        - https://www.segger.com/products/debug-probes/j-link/models/other-j-links/st-link-on-board/
    - Install J-Link software and documentation pack
        - https://www.segger.com/downloads/jlink/#J-LinkSoftwareAndDocumentationPack
