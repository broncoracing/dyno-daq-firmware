/*

Bronco Racing 2020 Dyno zDAQ

What this does:
  - HX711 2 channel scale amplifier input (80sps)
  - Servo motor control
  - CAN bus alive frame generation (50ms)
  - CAN bus data interpretation
  - Data transfer to MATLAB over serial

Info:
  - Current output rate is 80sps clocked by the HX711 chip.
  - Serial routed to Nucleo L432KC onboard micro USB port
  - Output multiplies HX711 reading by 1000 (keeps 3 decimal places)
*/

#include <Hx711.h>
#include <mbed.h>
#include <servo.h>

CAN can0(PA_11, PA_12, 250000);

Hx711 scale1 = Hx711(D11, D9);

Serial usb = Serial(USBTX, USBRX, 921600);

DigitalOut led(LED3);
DigitalOut testPin(D12); // used for timing tests
DigitalOut solenoidPin(D1);

volatile uint16_t rpm = 0;
volatile int16_t waterTemp = 0;
volatile double scaleValue = 0;
volatile uint32_t scaleInt = 0;

CANMessage inMsg;
CANMessage outMsg;

Timer canTimer;
Servo servo(A1);

void CANCallback();

int main() {
  canTimer.start();

  led.write(0);

  outMsg.id = 0;
  outMsg.len = 8;

  for (int i = 0; i < outMsg.len; i++) {
    outMsg.data[i] = 0;
  }

  scale1.setMultiplierA(1.f);
  scale1.set_scale(0.001f);

  wait_ms(100);

  scale1.tareA(15, .2); // Tare 15 times, .2 seconds each, and average them

  wait_ms(100);
  led.write(1);

  can0.filter(0x0CFFF048, 0x0CFFF548, CANExtended, 0);
  can0.attach(CANCallback);

  while (1) {
    // Read scale and remove decimal point. Remove negative vals.
    scaleInt = (uint32_t)((abs(scale1.readTaredA())) * 1000);

    // Print CAN alive frame
    if (canTimer.read_ms() > 50) {
      can0.write(outMsg);
      canTimer.reset();
    }

    // Send Data
    usb.printf("r%d", rpm);
    usb.printf("l%ld\n", scaleInt);

    if (usb.readable()) {
      // solenoidPin.write(usb.getc());
      servo.position((float)(uint8_t)usb.getc());
    }
  }
}

void CANCallback() {
  if (can0.read(inMsg)) {
    // Read in ECU frames
    // PE1
    if (inMsg.id == 0x0CFFF048) {
      rpm = ((inMsg.data[1] << 8) + inMsg.data[0]);
      led = !led;
    }
    // PE6
    else if (inMsg.id == 0x0CFFF548) {
      uint16_t newTemp = ((inMsg.data[5] << 8) + inMsg.data[4]);
      if (newTemp > 32767) {
        newTemp = newTemp - 65536;
      }
      waterTemp = ((newTemp / 10.0) * 1.8) + 32;
    }
  }
}