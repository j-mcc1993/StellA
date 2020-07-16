/*
 * Author: James McCullough
 * Date: 07/14/2020
 */
#include <CurieBLE.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>


// CurieBLE Custom Service
const int DATA_LENGTH = 8;
BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)
BLEService azAltService("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE LED Service
BLECharacteristic azCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", 
                                    BLERead | BLENotify, 
                                    DATA_LENGTH);


// For Madgewick algorithm
int aix, aiy, aiz;
int gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
float accelScale, gyroScale;
float azAlt[2]{0.0, 0.0};
const int SAMPLE_RATE = 50;
unsigned long microsNow, microsPerReading, microsPrevious;


// Convert accelerometer data to terms of g
float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0;
  return a;
}


// Convert gyro data to deg/s/s
float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
 
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}


void setup() {
  // Set advertised local name and service UUID:
  blePeripheral.setLocalName("azAlt");
  blePeripheral.setAdvertisedServiceUuid(azAltService.uuid());

  // Add service and characteristic:
  blePeripheral.addAttribute(azAltService);
  blePeripheral.addAttribute(azCharacteristic);

  // Set the initial value for the characeristic:
  azCharacteristic.setValue((unsigned char *)azAlt, DATA_LENGTH);

  // Begin advertising BLE service:
  blePeripheral.begin();
}


void loop() {
  // Listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // If a central is connected to peripheral:
  if (central) {
    
    // Start the IMU and filter
    Madgwick filter;
    CurieIMU.begin();
    CurieIMU.setGyroRate(SAMPLE_RATE);
    CurieIMU.setAccelerometerRate(SAMPLE_RATE);
    filter.begin(SAMPLE_RATE);

    // Set the accelerometer + gryo range and calibrate
    CurieIMU.setAccelerometerRange(2);
    CurieIMU.setGyroRange(250);
    CurieIMU.autoCalibrateGyroOffset();

    // Initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / SAMPLE_RATE;
    microsPrevious = micros();

    // While the central is still connected to peripheral:
    while (central.connected()) {
      // Check if it's time to read data and update the filter
      microsNow = micros();
      if (microsNow - microsPrevious >= microsPerReading) {

        // Read raw data from CurieIMU
        noInterrupts();
        CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);
        interrupts();

        // Convert from raw data to gravity and degrees/second units
        ax = convertRawAcceleration(aix);
        ay = convertRawAcceleration(aiy);
        az = convertRawAcceleration(aiz);
        gx = convertRawGyro(gix);
        gy = convertRawGyro(giy);
        gz = convertRawGyro(giz);

        // Update the filter
        filter.updateIMU(gx, gy, gz, ax, ay, az);

        // If sensor data has changed significantly, update and notify central
        if (abs(azAlt[0] - filter.getYaw()) > 1.0 || abs(azAlt[1] + filter.getPitch()) > 1.0) {
          azAlt[0] = filter.getYaw();
          azAlt[1] = -filter.getPitch();
          noInterrupts();
          azCharacteristic.setValue((unsigned char *)azAlt, DATA_LENGTH);
          interrupts();
        }
        
        // Increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
      }
    }
  }
}
