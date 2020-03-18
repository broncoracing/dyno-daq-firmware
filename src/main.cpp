/*

Bronco Racing 2020 Dyno zDAQ - See README for details

*/


#include <Hx711.h>
#include <PID.h>
#include <mbed.h>
#include <servo.h>

CAN can0(PA_11, PA_12, 250000);

PID controller(10, 0, 10, 80);

Hx711 scale1 = Hx711(D11, D9);

Serial usb = Serial(USBTX, USBRX, 921600);

DigitalOut led(LED3);
DigitalOut testPin(D12); // used for timing tests
DigitalOut solenoidPin(D1);

volatile uint16_t rpm = 0;
volatile int16_t waterTemp = 0;
volatile double scaleValue = 0;
volatile uint32_t scaleInt = 0;

volatile float servoVal = 95;
float RPMSet = 4500;

CANMessage inMsg;
CANMessage outMsg;

Timer canTimer;
Servo servo(A2);

void CANCallback();

int main() {

  controller.setInputLimits(0.0, 14000);
  controller.setOutputLimits(0, 140);
  controller.setMode(0);
  controller.setSetPoint(RPMSet);

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

  scale1.tareA(5, .2); // Tare 5 times, .2 seconds each, and average them

  wait_ms(100);
  led.write(1);

  can0.filter(0x0CFFF048, 0x0CFFF548, CANExtended, 0);
  can0.attach(CANCallback);

  while (1) {
    // Read scale and remove decimal point. Remove negative vals.
    scaleInt = (uint32_t)((abs(scale1.readTaredA())) * 1000);
    controller.setProcessValue((float)rpm);

    if (rpm > (RPMSet - 1500)) {
      //servoVal = 140 - controller.compute();
      servoVal = 95 ; //change the niumber here to change open angle (0 --> 140 = 0 --> 90)
    } else {
      servoVal = 0;
    }
servoVal = 1;
    servo.write(servoVal);

    // Send CAN alive frame
    if (canTimer.read_ms() > 50) {
      can0.write(outMsg);
      canTimer.reset();
    }

    // Send Data for MATLAB
    //usb.printf("r%d", rpm);
    //usb.printf("l%ld\n", scaleInt);

    // For terminal viewing
    usb.printf("%f\t", servoVal);
    usb.printf("%d\t", rpm);
    usb.printf("%ld\n", scaleInt);
    // usb.printf("%d\n", waterTemp);

    // if (usb.readable()) {
    // solenoidPin.write(usb.getc());
    // servo.write((uint8_t)usb.getc());
    // }
  }
}

// Fires on incoming CAN frame interrupt
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
      waterTemp = (uint16_t)((newTemp / 10) * 1.8) + 32;
    }
  }
}