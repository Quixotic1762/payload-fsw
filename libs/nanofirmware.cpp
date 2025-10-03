#include <Servo.h>
#include <Arduino_LPS22HB.h>
#include <Wire.h>
#include <Arduino_LSM9DS1.h>
#include <Arduino.h>

#define ALPHA 0.05
#define UART_BAUD 115200

unsigned long lastSensorMillis = 0;
unsigned long previousMillis = 0;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float roll, pitch, yaw;
float pressure, voltage;
float pitchMapped, rollMapped;
float temperature = 28.00;
float altitude = 10;

int fin_servo_flag = 0;
int parachute_servo_flag = 0;

int servopin0 = 6;
int servopin1 = 7;
int servopin2 = 8;
int servopin3 = 9;
int servopin4 = 10;

int pos = 0;

Servo servo0, servo1, servo2, servo3, servo4;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(UART_BAUD);

  if (!IMU.begin())
  {
    Serial.println("Failed to initialize IMU!");
    while (1)
      ;
  }

  //  barometer
  /*
  if (!BARO.begin()) {
    Serial.println("Failed to initialize Barometric sensor!");
    while (1);
  }
  */
  servo0.attach(servopin0);
  servo1.attach(servopin1);
  servo2.attach(servopin2);
  servo3.attach(servopin3);
  servo4.attach(servopin4);

  servo1.write(0);
  servo2.write(0);
}

void loop()
{
  if (Serial1.available())
  {
    char buff;
    String input_string;
    while (Serial1.available())
    {
      buff = Serial1.read();
      if (buff == '\n')
      {
        break;
      }
      else
      {
        input_string += buff;
      }
    }

    int input_num = input_string.toInt();
    switch (input_num)
    {
    case 1:
      fin_servo_flag = 1;
      break;
    case 2:
      parachute_servo_flag = 1;
      break;
    case 3:
      int alive = 128;
      Serial1.println(alive);
      break;
    }
  }
  unsigned long currentMillis = millis(); // vsas data
  int sensorValue = analogRead(A7);
  voltage = (sensorValue / 4095.0) * 6.6;

  char vBuf[20];
  snprintf(vBuf, sizeof(vBuf), "<V,%lu,%.4f>", currentMillis, voltage);
  Serial1.println(vBuf);

  if (fin_servo_flag)
  {
    servo1.write(90);
    servo2.write(90);
    servo3.write(90);
    servo0.write(90);
    fin_servo_flag = 0;
  }
  if (parachute_servo_flag)
  {
    servo4.write(90);
    parachute_servo_flag = 0;
  }

  if (millis() - lastSensorMillis >= 50)
  {
    lastSensorMillis = millis();
    float dt = (currentMillis - previousMillis) / 1000.0;

    if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable() && IMU.magneticFieldAvailable() && dt > 0)
    {
      previousMillis = currentMillis;

      IMU.readGyroscope(gx, gy, gz);
      IMU.readAcceleration(ax, ay, az);
      IMU.readMagneticField(mx, my, mz);

      float accRoll = atan2(ay, az) * 180 / M_PI;
      float accPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / M_PI;
      float magYaw = atan2(my, mx) * 180 / M_PI;

      roll = ALPHA * (roll + gx * dt) + (1 - ALPHA) * accRoll;
      pitch = ALPHA * (pitch + gy * dt) + (1 - ALPHA) * accPitch;
      yaw = ALPHA * (yaw + gz * dt) + (1 - ALPHA) * magYaw;

      // dummy barometer value
      float pressure1 = 1013.25;
      temperature = 29.0;
      altitude = 600;

      // servo code
      float limitedRoll = constrain(roll, -85, 85);
      float limitedPitch = constrain(pitch, -85, 85);

      if (roll < 0)
      {
        // if (roll < -85) roll = -85;
        rollMapped = limitedRoll + 85;
        servo1.write(rollMapped);
      }
      else
      {
        // if (roll > 85) roll = 85;
        rollMapped = 85 - limitedRoll;
        servo2.write(rollMapped);
      }
      if (pitch < 0)
      {
        // if (pitch < -85) pitch = -85;
        pitchMapped = limitedPitch + 85;
        servo3.write(pitchMapped);
      }
      else
      {
        // if (pitch > 85) pitch = 85;
        pitchMapped = 85 - limitedPitch;
        servo4.write(pitchMapped);
      }

      char buf[128];
      snprintf(buf, sizeof(buf),
               "<S,%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f>",
               currentMillis, temperature, roll, pitch, yaw,
               pressure1, altitude, ax, ay, az, gx, gy, gz, mx, my, mz);

      Serial1.println(buf);
    }
  }
  // delay(1000);
  // delay(10);// just added because of garbage value did not test it after removing it
}