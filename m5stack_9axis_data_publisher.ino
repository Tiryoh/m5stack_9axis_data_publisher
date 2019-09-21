// Copyright (c) 2018 Tiryoh
// Licensed under the MIT license.
// https://tiryoh.mit-license.org/2018

// This code is based on Angle3DMonitor,
// which is licensed under the MIT license.
// Copyright (c) 2018 katsumin
// https://github.com/katsumin/Angle3DMonitor

#include <M5Stack.h> // tested on 0.2.9
#include "utility/MPU9250.h"
#include "utility/quaternionFilters.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Flags
boolean beepEnable = false;
boolean sendEnable = false;
boolean bleEnable = false;

// Devices
MPU9250 IMU;

// GRAPH Constants
#define LCD_WIDTH 320
#define LCD_HEIGHT 240
#define GRAPH_OFFSET 5
#define GRAPH_MARK_SIZE 5
#define GRAPH_CENTER_X  (LCD_WIDTH - LCD_HEIGHT / 2 + GRAPH_OFFSET)
#define GRAPH_CENTER_Y  (LCD_HEIGHT / 2 - GRAPH_OFFSET)
#define GRAPH_FS        (LCD_HEIGHT / 2 - GRAPH_MARK_SIZE - GRAPH_OFFSET)
#define GRAPH_COLOR TFT_WHITE

// GUID
#define SERVICE_UUID           "bc6da0e6-3cbe-11e8-b467-0ed5f89f718b" // UART service UUID
#define CHARACTERISTIC_UUID_RX "482d5c81-607c-4e34-b42a-bff1f76607c0"
#define CHARACTERISTIC_UUID_TX "cd0a93d6-07c3-4c1a-b16d-b38694b2a715"

// BLE
BLECharacteristic *pCharacteristic;
bool deviceConnected = false;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

boolean resetMPU9250 = false;
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
        if (rxValue == "RST") {
          resetMPU9250 =  true;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);

  M5.begin();
  Wire.begin();
  BleInit();

  M5.Lcd.fillScreen(BLACK);
  char* logo = "/WinDesignLogo.jpg";
  if ( SD.exists(logo) ) {
    // Logo Jpeg
    M5.Lcd.drawJpgFile(SD, logo, (LCD_WIDTH - 243) / 2, (LCD_HEIGHT - 84) / 2 );
  } else {
    M5.Lcd.setTextColor(WHITE , BLACK); // Set pixel color; 1 on the monochrome screen
    //    M5.Lcd.setTextSize(2);
    //    M5.Lcd.setCursor( 0, 0); M5.Lcd.print("WinDesign");
    M5.Lcd.drawCentreString( "WinDesign", LCD_WIDTH / 2, LCD_HEIGHT / 2, 2);
  }

  MPU9250_init();

  //  delay(2000);
  for (int i = 255; i >= 0; i--) {
    M5.Lcd.setBrightness(i);
    delay(8);
  }
  M5.Lcd.setBrightness(100);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN , BLACK);
  M5.Lcd.setTextSize(1);
  drawScale();
  dispFlags();
}

void BleInit()
{
  // Create the BLE Device
  BLEDevice::init("Angle Meter");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}

void dispFlags()
{
  M5.Lcd.setCursor( 40, 230); M5.Lcd.printf("beep:%s", (beepEnable ? " on" : "off"));
  M5.Lcd.setCursor(140, 230); M5.Lcd.printf("send:%s", (sendEnable ? " on" : "off"));
  if ( deviceConnected ) {
    M5.Lcd.setCursor(220, 230); M5.Lcd.printf(" ble:%s", (bleEnable ? " on" : "off"));
  } else {
    M5.Lcd.setCursor(220, 230); M5.Lcd.print("        ");
  }
}

void MPU9250_init()
{
  byte c = IMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
  Serial.print(" I should be "); Serial.println(0x71, HEX);

  if (c == 0x71) { // WHO_AM_I should always be 0x68
    Serial.println("MPU9250 is online...");

    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[0], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[1], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[2], 1); Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[3], 1); Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[4], 1); Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[5], 1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);

    IMU.initMPU9250();

    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
    Serial.print(" I should be "); Serial.println(0x48, HEX);
    if ( d == 0x48 ) {
      Serial.println("AK8963 is online...");

      IMU.initAK8963(IMU.magCalibration);
    }
  }
}

void drawScale()
{
  M5.Lcd.drawLine( GRAPH_CENTER_X - GRAPH_FS, GRAPH_CENTER_Y           , GRAPH_CENTER_X + GRAPH_FS, GRAPH_CENTER_Y           , GRAPH_COLOR);
  M5.Lcd.drawLine( GRAPH_CENTER_X           , GRAPH_CENTER_Y - GRAPH_FS, GRAPH_CENTER_X           , GRAPH_CENTER_Y + GRAPH_FS, GRAPH_COLOR);
  M5.Lcd.drawCircle( GRAPH_CENTER_X, GRAPH_CENTER_Y, GRAPH_FS, GRAPH_COLOR);
}

float angle_x = 0.0;
float angle_y = 0.0;
float offset_x = 0.0;
float offset_y = 0.0;
float expansion = 1.0; // -90 - +90
//    float expansion = 9.0; // -10 - +10
int16_t cur_x = GRAPH_CENTER_X;
int16_t cur_y = GRAPH_CENTER_Y;
int16_t pre_x;
int16_t pre_y;
void updateAngle()
{
  pre_x = cur_x;
  pre_y = cur_y;

  float ax = IMU.ax;
  float ay = IMU.ay;
  float az = IMU.az;
  float norm = sqrt(ax * ax + ay * ay + az * az);
  if ( norm != 0.0f ) {
    ax /= norm;
    ay /= norm;
    az /= norm;
    angle_x = -1.0 * asin(ax) / 3.14;
    angle_y = -1.0 * asin(ay) / 3.14;
  }
  cur_x =  (int16_t)( (angle_x + offset_x) * GRAPH_FS * expansion * 2) + GRAPH_CENTER_X;
  cur_x = (cur_x > GRAPH_CENTER_X + GRAPH_FS) ? GRAPH_CENTER_X + GRAPH_FS : (cur_x <= GRAPH_CENTER_X - GRAPH_FS) ? GRAPH_CENTER_X - GRAPH_FS : cur_x;
  cur_y = -(int16_t)( (angle_y + offset_y) * GRAPH_FS * expansion * 2) + GRAPH_CENTER_Y;
  cur_y = (cur_y > GRAPH_CENTER_Y + GRAPH_FS) ? GRAPH_CENTER_Y + GRAPH_FS : (cur_y <= GRAPH_CENTER_Y - GRAPH_FS) ? GRAPH_CENTER_Y - GRAPH_FS : cur_y;

  if ( cur_x != pre_x || cur_y != pre_y ) {
    fillMark();
    drawScale();
    drawMark();
    if ( beepEnable && cur_x == GRAPH_CENTER_X && cur_y == GRAPH_CENTER_Y ) {
      M5.Speaker.setVolume(10);
      M5.Speaker.beep();
    }
  }
}
void drawMark()
{
  M5.Lcd.drawLine( cur_x - GRAPH_MARK_SIZE, cur_y - 1, cur_x + GRAPH_MARK_SIZE, cur_y - 1, TFT_YELLOW);
  M5.Lcd.drawLine( cur_x - GRAPH_MARK_SIZE, cur_y + 1, cur_x + GRAPH_MARK_SIZE, cur_y + 1, TFT_YELLOW);
  M5.Lcd.drawLine( cur_x - 1, cur_y - GRAPH_MARK_SIZE, cur_x - 1, cur_y + GRAPH_MARK_SIZE, TFT_YELLOW);
  M5.Lcd.drawLine( cur_x + 1, cur_y - GRAPH_MARK_SIZE, cur_x + 1, cur_y + GRAPH_MARK_SIZE, TFT_YELLOW);
}
void fillMark()
{
  M5.Lcd.fillRect( pre_x - GRAPH_MARK_SIZE, pre_y - GRAPH_MARK_SIZE, GRAPH_MARK_SIZE * 2 + 1, GRAPH_MARK_SIZE * 2 + 1, TFT_BLACK);
}

void loop() {
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);  // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount);  // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.readMagData(IMU.magCount);  // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be
    // automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];

    // Must be called before updating quaternions!
    IMU.updateTime();

    // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
    // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
    // (+ up) of accelerometer and gyro! We have to make some allowance for this
    // orientationmismatch in feeding the output to the quaternion filter. For the
    // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
    // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
    // modified to allow any convenient orientation convention. This is ok by
    // aircraft orientation standards! Pass gyro rate as rad/s
    MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az,
                           IMU.gx * DEG_TO_RAD, IMU.gy * DEG_TO_RAD, IMU.gz * DEG_TO_RAD,
                           IMU.mx, IMU.my, IMU.mz, IMU.deltat);

    // Serial print and/or display at 0.5 s rate independent of data rates
    IMU.delt_t = millis() - IMU.count;

    // update LCD once per half-second independent of read rate
    // if (IMU.delt_t > 500)
    if (IMU.delt_t > 100)
    {
      M5.Lcd.setCursor( 0, 50);  M5.Lcd.printf(" x(mg): %+6d", (int)(1000 * IMU.ax));
      M5.Lcd.setCursor( 0, 60);  M5.Lcd.printf(" y(mg): %+6d", (int)(1000 * IMU.ay));
      M5.Lcd.setCursor( 0, 70);  M5.Lcd.printf(" z(mg): %+6d", (int)(1000 * IMU.az));

      M5.Lcd.setCursor( 0, 80); M5.Lcd.printf("x(o/s): %+6d", (int)(IMU.gx));
      M5.Lcd.setCursor( 0, 90); M5.Lcd.printf("y(o/s): %+6d", (int)(IMU.gy));
      M5.Lcd.setCursor( 0, 100); M5.Lcd.printf("z(o/s): %+6d", (int)(IMU.gz));

      M5.Lcd.setCursor( 0, 110); M5.Lcd.printf(" x(mG): %+6d", (int)(IMU.mx));
      M5.Lcd.setCursor( 0, 120); M5.Lcd.printf(" y(mG): %+6d", (int)(IMU.my));
      M5.Lcd.setCursor( 0, 130); M5.Lcd.printf(" z(mG): %+6d", (int)(IMU.mz));

      // Define output variables from updated quaternion---these are Tait-Bryan
      // angles, commonly used in aircraft orientation. In this coordinate system,
      // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
      // x-axis and Earth magnetic North (or true North if corrected for local
      // declination, looking down on the sensor positive yaw is counterclockwise.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // For more see
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.
      float qw = getQ()[0];
      float qx = getQ()[1];
      float qy = getQ()[2];
      float qz = getQ()[3];
      //      // original
      //      IMU.yaw   = atan2(
      //        2.0f * (qx * qy + qw * qz),
      //        qw * qw + qx * qx - qy * qy - qz * qz);
      //      IMU.pitch = -asin(2.0f * qx * qz - qw * qy);
      //      IMU.roll  = atan2(
      //        2.0f * (qw * qx + qy * qz),
      //        qw * qw - qx * qx - qy * qy + qz * qz);
      //      // original
      // wikipedia
      float sinr = 2.0 * ( qw * qx + qy * qz );
      float cosr = 1.0 - 2.0 * ( qx * qx + qy * qy );
      IMU.roll = atan2( sinr, cosr );

      float sinp = 2.0 * ( qw * qy - qz * qx );
      if ( fabs( sinp ) >= 1.0 )
        //        IMU.pitch = copysign( M_PI / 2, sinp );
        IMU.pitch = (sinp > 0) ? (M_PI / 2) : -(M_PI / 2);
      else
        IMU.pitch = asin( sinp );

      float siny = 2.0 * ( qw * qz + qx * qy );
      float cosy = 1.0 - 2.0 * ( qy * qy + qz * qz );
      IMU.yaw = atan2( siny, cosy );
      // wikipedia

      IMU.pitch *= RAD_TO_DEG;
      IMU.yaw   *= RAD_TO_DEG;
      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      // Declination of Tokyo (35°40'59.0"N 139°48'32.0"E) is
      //   7° 29' W  ± 0° 18' changing by 0° 4' W on 2018-04-19
      //      IMU.yaw   -= 8.5;
      IMU.yaw   += 7.3;
      IMU.roll  *= RAD_TO_DEG;

      M5.Lcd.setCursor( 0, 140);  M5.Lcd.printf("   yaw: %+6.1f", (IMU.yaw));
      M5.Lcd.setCursor( 0, 150);  M5.Lcd.printf(" pitch: %+6.1f", (IMU.pitch));
      M5.Lcd.setCursor( 0, 160);  M5.Lcd.printf("  roll: %+6.1f", (IMU.roll));

      // With these settings the filter is updating at a ~145 Hz rate using the
      // Madgwick scheme and >200 Hz using the Mahony scheme even though the
      // display refreshes at only 2 Hz. The filter update rate is determined
      // mostly by the mathematical steps in the respective algorithms, the
      // processor speed (8 MHz for the 3.3V Pro Mini), and the magnetometer ODR:
      // an ODR of 10 Hz for the magnetometer produce the above rates, maximum
      // magnetometer ODR of 100 Hz produces filter update rates of 36 - 145 and
      // ~38 Hz for the Madgwick and Mahony schemes, respectively. This is
      // presumably because the magnetometer read takes longer than the gyro or
      // accelerometer reads. This filter update rate should be fast enough to
      // maintain accurate platform orientation for stabilization control of a
      // fast-moving robot or quadcopter. Compare to the update rate of 200 Hz
      // produced by the on-board Digital Motion Processor of Invensense's MPU6050
      // 6 DoF and MPU9150 9DoF sensors. The 3.3 V 8 MHz Pro Mini is doing pretty
      // well!

      // M5.Lcd.setCursor(0, 60);
      // M5.Lcd.printf("yaw:%6.2f   pitch:%6.2f   roll:%6.2f  ypr \r\n",(IMU.yaw), (IMU.pitch), (IMU.roll));
      M5.Lcd.setCursor( 0, 170);  M5.Lcd.printf("rt(Hz): %6.1f", (float) IMU.sumCount / IMU.sum);

      IMU.count = millis();
      IMU.sumCount = 0;
      IMU.sum = 0;

      // Serial send
      char buf[6][20 + 1];
      snprintf( buf[0], sizeof(buf[0]), "a=%+5d,%+5d,%+5d", (int)(1000 * IMU.ax), (int)(1000 * IMU.ay), (int)(1000 * IMU.az));
      snprintf( buf[1], sizeof(buf[1]), "g=%+5d,%+5d,%+5d", (int)(IMU.gx), (int)(IMU.gy), (int)(IMU.gz));
      snprintf( buf[2], sizeof(buf[2]), "m=%+5d,%+5d,%+5d", (int)(IMU.mx), (int)(IMU.my), (int)(IMU.mz));
      snprintf( buf[3], sizeof(buf[3]), "qw=%+6.1f,qx=%+6.1f", qw, qx);
      snprintf( buf[4], sizeof(buf[4]), "qy=%+6.1f,qz=%+6.1f", qy, qz);
      snprintf( buf[5], sizeof(buf[4]), "%+6.1f,%+6.1f,%+6.1f", IMU.roll, IMU.pitch, IMU.yaw);

      if ( sendEnable ) {
        /*
        for ( int i = 0; i < 6; i++ ) {
          Serial.println(buf[i]);
        }
        */
        static int timestamp = 0;
        Serial.print(timestamp);
        Serial.print(",");
        if (timestamp > 254) {
          timestamp = 0;
        }
        else {
          timestamp++;
        }

        Serial.print(IMU.gx * DEG_TO_RAD, 6); // X-gyro rate [rad/sec] (degree/s to rad/s)
        Serial.print(",");
        Serial.print(IMU.gy * DEG_TO_RAD, 6);
        Serial.print(",");
        Serial.print(IMU.gz * DEG_TO_RAD, 6);
        Serial.print(",");
        Serial.print(IMU.ax, 6); // X-acceleration [G]
        Serial.print(",");
        Serial.print(IMU.ay, 6);
        Serial.print(",");
        Serial.print(IMU.az, 6);
        Serial.print(",");
        Serial.print(IMU.mx * 0.1, 6); // X-mag field [uT]  (mG to uT)
        Serial.print(",");
        Serial.print(IMU.my * 0.1, 6);
        Serial.print(",");
        Serial.print(IMU.mz * 0.1, 6);
        Serial.print(",");
        IMU.tempCount = IMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        IMU.temperature = ((float) IMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print(IMU.temperature, 6);
        Serial.println("");
      }


      // BLE send
      if ( bleEnable && deviceConnected ) {
        for ( int i = 0; i < 6; i++ ) {
          int len = strlen(buf[i]);
          pCharacteristic->setValue( (uint8_t *)buf[i], len );
          pCharacteristic->notify();
        }
      }
    } // if (IMU.delt_t > 500)

    // display refresh
    if ( M5.BtnA.wasPressed() ) {
      beepEnable = !beepEnable;
    }
    if ( M5.BtnB.wasPressed() ) {
      sendEnable = !sendEnable;
    }
    if ( M5.BtnC.wasPressed() ) {
      bleEnable = !bleEnable;
    }
    dispFlags();
    updateAngle();
    M5.Lcd.setCursor( 0, 0);  M5.Lcd.printf("x(deg.): %6.1f", angle_x * 180.0);
    M5.Lcd.setCursor( 0, 10);  M5.Lcd.printf("y(deg.): %6.1f", angle_y * 180.0);

    if ( resetMPU9250 ) {
      Serial.println("reset MPU !");
      MPU9250_init();
      resetMPU9250 = false;
    }
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  M5.update();
}
