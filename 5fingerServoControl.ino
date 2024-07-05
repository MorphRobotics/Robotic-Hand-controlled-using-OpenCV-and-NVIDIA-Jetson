#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Firmata.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // Minimum pulse length count (out of 4096)
#define SERVOMAX  600 // Maximum pulse length count (out of 4096)

void setup()
{
  Serial.begin(57600);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop()
{
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    int servoPos[5];
    int index = 0;
    char *token = strtok(data.c_str(), ",");

    while (token != NULL) {
      servoPos[index] = atoi(token);
      index++;
      token = strtok(NULL, ",");
    }

    for (int i = 0; i < 5; i++) {
      int pulseLength = map(servoPos[i], 0, 180, SERVOMIN, SERVOMAX);
      pwm.setPWM(i, 0, pulseLength);
    }
  }
}
