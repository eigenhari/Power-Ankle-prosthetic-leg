#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

BNO08x myIMU;

#define BNO08X_INT  A4
#define BNO08X_RST  A5
#define BNO08X_ADDR 0x4A 



void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10); // Wait for Serial to become available

  Wire.begin();

  if (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    while (1); // Freeze if the sensor is not detected
  }

  setReports();
  delay(100);
}

void setReports(void) {
  if (myIMU.enableGyro() == false) {
    //Serial.println("Could not enable gyro");
  }


}

void loop() {
  delay(10);

  if (myIMU.wasReset()) {
    setReports();
  }

  if (myIMU.getSensorEvent() == true) {
    if (myIMU.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {

      float x = myIMU.getGyroX();
      float y = myIMU.getGyroY();
      float z = myIMU.getGyroZ();
      unsigned long currentTime = millis();  // Get time in milliseconds

      Serial.print(currentTime / 1000.0, 3); // Convert to seconds
      Serial.print(F(","));
      Serial.print(x, 5);
      Serial.print(F(","));
      Serial.print(y, 5);
      Serial.print(F(","));
      Serial.print(z, 5);
      Serial.println();
    }
  }
}
