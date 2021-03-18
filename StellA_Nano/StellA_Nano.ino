/* 
 *  Authors: Kris Winer (original sketch here: https://github.com/kriswiner/LSM9DS1/commit/4c35dd7b5c2a470bcc0ec913ce7a54a56ae4ee8b) & James McCullough
 *  Date: 07.20.2020
 *  Description: This code interacts with the LSM9DS1 IMU chip
 *  on the Nano 33 BLE board to implement the Madgwick algorithm
 *  and send orientation data to a BLE service.
 */
#include <ArduinoBLE.h>
#include <Arduino_HTS221.h>
#include "Wire.h"

// Accelerometer and Gyroscope registers
#define LSM9DS1XG_ACT_THS      0x04
#define LSM9DS1XG_ACT_DUR     0x05
#define LSM9DS1XG_INT_GEN_CFG_XL    0x06
#define LSM9DS1XG_INT_GEN_THS_X_XL  0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL  0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL  0x09
#define LSM9DS1XG_INT_GEN_DUR_XL    0x0A
#define LSM9DS1XG_REFERENCE_G       0x0B
#define LSM9DS1XG_INT1_CTRL         0x0C
#define LSM9DS1XG_INT2_CTRL         0x0D
#define LSM9DS1XG_WHO_AM_I          0x0F  // should return 0x68
#define LSM9DS1XG_CTRL_REG1_G       0x10
#define LSM9DS1XG_CTRL_REG2_G       0x11
#define LSM9DS1XG_CTRL_REG3_G       0x12
#define LSM9DS1XG_ORIENT_CFG_G      0x13
#define LSM9DS1XG_INT_GEN_SRC_G     0x14
#define LSM9DS1XG_OUT_TEMP_L        0x15
#define LSM9DS1XG_OUT_TEMP_H        0x16
#define LSM9DS1XG_STATUS_REG        0x17
#define LSM9DS1XG_OUT_X_L_G         0x18
#define LSM9DS1XG_OUT_X_H_G         0x19
#define LSM9DS1XG_OUT_Y_L_G         0x1A
#define LSM9DS1XG_OUT_Y_H_G         0x1B
#define LSM9DS1XG_OUT_Z_L_G         0x1C
#define LSM9DS1XG_OUT_Z_H_G         0x1D
#define LSM9DS1XG_CTRL_REG4         0x1E
#define LSM9DS1XG_CTRL_REG5_XL      0x1F
#define LSM9DS1XG_CTRL_REG6_XL      0x20
#define LSM9DS1XG_CTRL_REG7_XL      0x21
#define LSM9DS1XG_CTRL_REG8         0x22
#define LSM9DS1XG_CTRL_REG9         0x23
#define LSM9DS1XG_CTRL_REG10        0x24
#define LSM9DS1XG_INT_GEN_SRC_XL    0x26
//#define LSM9DS1XG_STATUS_REG        0x27 // duplicate of 0x17!
#define LSM9DS1XG_OUT_X_L_XL        0x28
#define LSM9DS1XG_OUT_X_H_XL        0x29
#define LSM9DS1XG_OUT_Y_L_XL        0x2A
#define LSM9DS1XG_OUT_Y_H_XL        0x2B
#define LSM9DS1XG_OUT_Z_L_XL        0x2C
#define LSM9DS1XG_OUT_Z_H_XL        0x2D
#define LSM9DS1XG_FIFO_CTRL         0x2E
#define LSM9DS1XG_FIFO_SRC          0x2F
#define LSM9DS1XG_INT_GEN_CFG_G     0x30
#define LSM9DS1XG_INT_GEN_THS_XH_G  0x31
#define LSM9DS1XG_INT_GEN_THS_XL_G  0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G  0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G  0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G  0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G  0x36
#define LSM9DS1XG_INT_GEN_DUR_G     0x37
//
// Magnetometer registers
#define LSM9DS1M_OFFSET_X_REG_L_M   0x05
#define LSM9DS1M_OFFSET_X_REG_H_M   0x06
#define LSM9DS1M_OFFSET_Y_REG_L_M   0x07
#define LSM9DS1M_OFFSET_Y_REG_H_M   0x08
#define LSM9DS1M_OFFSET_Z_REG_L_M   0x09
#define LSM9DS1M_OFFSET_Z_REG_H_M   0x0A
#define LSM9DS1M_WHO_AM_I           0x0F  // should be 0x3D
#define LSM9DS1M_CTRL_REG1_M        0x20
#define LSM9DS1M_CTRL_REG2_M        0x21
#define LSM9DS1M_CTRL_REG3_M        0x22
#define LSM9DS1M_CTRL_REG4_M        0x23
#define LSM9DS1M_CTRL_REG5_M        0x24
#define LSM9DS1M_STATUS_REG_M       0x27
#define LSM9DS1M_OUT_X_L_M          0x28
#define LSM9DS1M_OUT_X_H_M          0x29
#define LSM9DS1M_OUT_Y_L_M          0x2A
#define LSM9DS1M_OUT_Y_H_M          0x2B
#define LSM9DS1M_OUT_Z_L_M          0x2C
#define LSM9DS1M_OUT_Z_H_M          0x2D
#define LSM9DS1M_INT_CFG_M          0x30
#define LSM9DS1M_INT_SRC_M          0x31
#define LSM9DS1M_INT_THS_L_M        0x32
#define LSM9DS1M_INT_THS_H_M        0x33

#define LSM9DS1XG_ADDRESS 0x6B  //  Device address 
#define LSM9DS1M_ADDRESS  0x1E  //  Address of magnetometer

// Set initial input parameters
enum Ascale {  // set of allowable accel full scale settings
    AFS_2G = 0,
    AFS_16G,
    AFS_4G,
    AFS_8G
};

enum Aodr {  // set of allowable gyro sample rates
    AODR_PowerDown = 0,
    AODR_10Hz,
    AODR_50Hz,
    AODR_119Hz,
    AODR_238Hz,
    AODR_476Hz,
    AODR_952Hz
};

enum Abw {  // set of allowable accewl bandwidths
    ABW_408Hz = 0,
    ABW_211Hz,
    ABW_105Hz,
    ABW_50Hz
};

enum Gscale {  // set of allowable gyro full scale settings
    GFS_245DPS = 0,
    GFS_500DPS,
    GFS_NoOp,
    GFS_2000DPS
};

enum Godr {  // set of allowable gyro sample rates
    GODR_PowerDown = 0,
    GODR_14_9Hz,
    GODR_59_5Hz,
    GODR_119Hz,
    GODR_238Hz,
    GODR_476Hz,
    GODR_952Hz
};

enum Gbw {   // set of allowable gyro data bandwidths
    GBW_low = 0,  // 14 Hz at Godr = 238 Hz,  33 Hz at Godr = 952 Hz
    GBW_med,      // 29 Hz at Godr = 238 Hz,  40 Hz at Godr = 952 Hz
    GBW_high,     // 63 Hz at Godr = 238 Hz,  58 Hz at Godr = 952 Hz
    GBW_highest   // 78 Hz at Godr = 238 Hz, 100 Hz at Godr = 952 Hz
};

enum Mscale {  // set of allowable mag full scale settings
    MFS_4G = 0,
    MFS_8G,
    MFS_12G,
    MFS_16G
};

enum Mmode {
    MMode_LowPower = 0, 
    MMode_MedPerformance,
    MMode_HighPerformance,
    MMode_UltraHighPerformance
};

enum Modr {  // set of allowable mag sample rates
    MODR_0_625Hz = 0,
    MODR_1_25Hz,
    MODR_2_5Hz,
    MODR_5Hz,
    MODR_10Hz,
    MODR_20Hz,
    MODR_80Hz
};

// Specify sensor full scale
uint8_t Gscale = GFS_245DPS; // gyro full scale
uint8_t Godr = GODR_238Hz;   // gyro data sample rate
uint8_t Gbw = GBW_med;       // gyro data bandwidth
uint8_t Ascale = AFS_2G;     // accel full scale
uint8_t Aodr = AODR_238Hz;   // accel data sample rate
uint8_t Abw = ABW_50Hz;      // accel data bandwidth
uint8_t Mscale = MFS_4G;     // mag full scale
uint8_t Modr = MODR_80Hz;    // mag data sample rate
uint8_t Mmode = MMode_UltraHighPerformance;  // magnetometer operation mode
float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors

int16_t accelCount[3], gyroCount[3], magCount[3];  // Stores the 16-bit signed accelerometer, gyro, and mag sensor output
float gyroBias[3] = {.00106, -.00076, .00001}, accelBias[3] = {-.02264, -.02533, .02625},  magBias[3] = {.44006, .01257, .01990}; // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempCount;            // temperature raw count output
float   temperature;          // Stores the LSM9DS1gyro internal chip temperature in degrees Celsius

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float beta = .9f;

float pitch, yaw, roll;
float deltat = 0.0f;                      // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// ArduinoBLE Custom Service
float azAlt[]{0.0, 0.0};
float tempHumidity[]{0.0, 0.0, 0.0};
unsigned long count = 0;
unsigned long delta = 0;
const float a = 6.112;
const float b = 17.62;
const float c = 243.12;
const float e = 2.71828;
const float DECLINATION = 13.22f;
const int TMP_LENGTH = 12;
const int DATA_LENGTH = 8;
int temp_interval = 60;
bool ud;

// ArduinoBLE
BLEService azAltService("19B10000-E8F2-537E-4F6C-D104768A1214"); // Altitude + Azimuth
BLECharacteristic azCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", 
        BLERead | BLENotify, 
        DATA_LENGTH);
BLECharacteristic thCharacteristic("19B10002-E8F2-537E-4F6C-D104768A1214", 
        BLERead, 
        TMP_LENGTH);

void setup()
{
    // Turn PWR led off and turn red led on
    digitalWrite(LED_PWR, LOW);
    digitalWrite(LEDR, LOW);

    // Start serial communication with LSM9DS1
    Wire1.begin();

    // reset LSM9DS1
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x05);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, 0x0c);
    delay(4000);

    // Read the WHO_AM_I registers, this is a good test of communication
    byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 accel/gyro
    byte d = readByte(LSM9DS1M_ADDRESS, LSM9DS1M_WHO_AM_I);  // Read WHO_AM_I register for LSM9DS1 magnetometer

    if (c == 0x68 && d == 0x3D) // WHO_AM_I should always be 0x0E for the accel/gyro and 0x3C for the mag
    {  
        // get sensor resolutions, only need to do this once
        getAres();
        getGres();
        getMres();
        magcalLSM9DS1(magBias);

        delay(2000);
        initLSM9DS1(); 

        // Begin advertising BLE service:
        if (!BLE.begin()){
            while(1) ledBlink(250, LEDR);
        }

        // Begin reading thermo/humidity sensor
        if (!HTS.begin()) {
            while (1) ledBlink(250, LEDR);
        }

        // Set advertised local name and service UUID:
        BLE.setLocalName("Nano33");
        BLE.setAdvertisedServiceUuid(azAltService.uuid());

        azAltService.addCharacteristic(azCharacteristic);
        azAltService.addCharacteristic(thCharacteristic);
        BLE.addService(azAltService);
        BLE.advertise();

        // Set the initial value for the characeristic:
        azCharacteristic.setValue((unsigned char *)azAlt, DATA_LENGTH);

        // Set initial temp/humidity values
        tempHumidity[0] = HTS.readTemperature(FAHRENHEIT);
        tempHumidity[1] = HTS.readHumidity();
        setDewPoint();
        thCharacteristic.setValue((unsigned char *)tempHumidity, TMP_LENGTH);

        digitalWrite(LEDR, HIGH);
    }
    else
    {
        while(1) ledBlink(250, LEDR);
    }
}

void loop()
{ 
    // Listen for BLE peripherals to connect:
    BLEDevice central = BLE.central();

    if (central) {
        while (central.connected()) {
            if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x01) {  // check if new accel data is ready  
                readAccelData(accelCount);  // Read the x/y/z adc values

                // Now we'll calculate the accleration value into actual g's
                ax = (float)accelCount[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
                ay = (float)accelCount[1]*aRes - accelBias[1];   
                az = (float)accelCount[2]*aRes - accelBias[2]; 
            } 

            if (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_STATUS_REG) & 0x02) {  // check if new gyro data is ready  
                readGyroData(gyroCount);  // Read the x/y/z adc values

                // Calculate the gyro value into actual degrees per second
                gx = (float)gyroCount[0]*gRes - gyroBias[0];  // get actual gyro value, this depends on scale being set
                gy = (float)gyroCount[1]*gRes - gyroBias[1];  
                gz = (float)gyroCount[2]*gRes - gyroBias[2];   
            }

            if (readByte(LSM9DS1M_ADDRESS, LSM9DS1M_STATUS_REG_M) & 0x08) {  // check if new mag data is ready  
                readMagData(magCount);  // Read the x/y/z adc values

                // Calculate the magnetometer values in milliGauss
                // Include factory calibration per data sheet and user environmental corrections
                mx = (float)magCount[0]*mRes;// - magBias[0];  // get actual magnetometer value, this depends on scale being set
                my = (float)magCount[1]*mRes;// - magBias[1];  
                mz = (float)magCount[2]*mRes;// - magBias[2];   
            }

            // Integration delta
            Now = micros();
            deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
            lastUpdate = Now;

            // Update fusion algorithm
            MadgwickQuaternionUpdate(-ax, ay, az, -gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f, mx, my, mz);

            // Gyro chip temp
            tempCount = readTempData();  // Read the gyro adc values
            temperature = ((float) tempCount/256. + 25.0); // Gyro chip temperature in degrees Centigrade

            // Euler Angles in Deg.
            yaw   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]); 
            pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
            roll  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
            pitch *= 180.0f / PI;
            yaw   *= 180.0f / PI; 
            roll  *= 180.0f / PI;

            // Declination adjustment
            yaw   -= DECLINATION; // Declination at Los Altos, California is ~13 degrees 13 minutes on 2020-07-19
            if (yaw < 0) {
                yaw += 360.0f;
            }

            // Update BLE characteristic
            ud = false;
            if (abs(azAlt[0] - yaw) > 1.0f) {
                ud = true;
                azAlt[0] = yaw;
            }
            if (abs(azAlt[1] - pitch) > 1.0f) {
                ud = true;
                azAlt[1] = pitch;
            }

            if (ud) {
                azCharacteristic.setValue((unsigned char *)azAlt, DATA_LENGTH);
            }

            // Check if it's time to update temp/humidity
            delta = millis() - count;
            if (delta > temp_interval*1000) {
                tempHumidity[0] = HTS.readTemperature(FAHRENHEIT);
                tempHumidity[1] = HTS.readHumidity();
                setDewPoint();
                checkDewStatus();
                thCharacteristic.setValue((unsigned char *)tempHumidity, TMP_LENGTH);
                count = millis();
            }
        }
    }
}

//===================================================================================================================
//====== Set of useful function to access acceleration. gyroscope, magnetometer, and temperature data
//===================================================================================================================

// Blink RGB LED on Nano 33 boards
void ledBlink(int wait, int pin) {
    digitalWrite(pin, LOW);
    delay(wait);
    digitalWrite(pin, HIGH);
    delay(wait);
}

// Calculate dewpoint using Magnus formula
void setDewPoint() {
    // Convert to celsius
    float temp = (tempHumidity[0] - 32.0)/1.8;
    float humidity = tempHumidity[1];

    // Avoid repeated calculations
    float lnplusbt = float(log(humidity/100.0)) + b*temp/(c + temp);
    float dewpoint = c*lnplusbt/(b - lnplusbt);

    // Convert to Fahrenheit
    tempHumidity[2] = 1.8*dewpoint + 32.0;
}

// Check if temp is near dewpoint and turn on indicator if so
void checkDewStatus() {
    if (tempHumidity[0] - tempHumidity[2] <= 5.0) {
        digitalWrite(LEDR, LOW);
    }
}

void getMres() {
    switch (Mscale)
    {
        // Possible magnetometer scales (and their register bit settings) are:
        // 4 Gauss (00), 8 Gauss (01), 12 Gauss (10) and 16 Gauss (11)
        case MFS_4G:
            mRes = 4.0/32768.0;
            break;
        case MFS_8G:
            mRes = 8.0/32768.0;
            break;
        case MFS_12G:
            mRes = 12.0/32768.0;
            break;
        case MFS_16G:
            mRes = 16.0/32768.0;
            break;
    }
}

void getGres() {
    switch (Gscale)
    {
        // Possible gyro scales (and their register bit settings) are:
        // 245 DPS (00), 500 DPS (01), and 2000 DPS  (11). 
        case GFS_245DPS:
            gRes = 245.0/32768.0;
            break;
        case GFS_500DPS:
            gRes = 500.0/32768.0;
            break;
        case GFS_2000DPS:
            gRes = 2000.0/32768.0;
            break;
    }
}

void getAres() {
    switch (Ascale)
    {
        // Possible accelerometer scales (and their register bit settings) are:
        // 2 Gs (00), 16 Gs (01), 4 Gs (10), and 8 Gs  (11). 
        case AFS_2G:
            aRes = 2.0/32768.0;
            break;
        case AFS_16G:
            aRes = 16.0/32768.0;
            break;
        case AFS_4G:
            aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            aRes = 8.0/32768.0;
            break;
    }
}


void readAccelData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &rawData[0]);  // Read the six raw data registers into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}


void readGyroData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

void readMagData(int16_t * destination)
{
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;  // Turn the MSB and LSB into a signed 16-bit value
    destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  // Data stored as little Endian
    destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int16_t readTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_TEMP_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array 
    return (((int16_t)rawData[1] << 8) | rawData[0]);  // Turn the MSB and LSB into a 16-bit signed value
}


void initLSM9DS1()
{  
    // enable the 3-axes of the gyroscope
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
    // configure the gyroscope
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
    delay(200);
    // enable the three axes of the accelerometer 
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
    // configure the accelerometer-specify bandwidth selection with Abw
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
    delay(200);
    // enable block data update, allow auto-increment during multiple byte read
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);
    // configure the magnetometer-enable temperature compensation of mag data
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode
}


void selftestLSM9DS1()
{
    float accel_noST[3] = {0., 0., 0.}, accel_ST[3] = {0., 0., 0.};
    float gyro_noST[3] = {0., 0., 0.}, gyro_ST[3] = {0., 0., 0.};

    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
    accelgyrocalLSM9DS1(gyro_noST, accel_noST);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x05); // enable gyro/accel self test
    accelgyrocalLSM9DS1(gyro_ST, accel_ST);

    float gyrodx = (gyro_ST[0] - gyro_noST[0]);
    float gyrody = (gyro_ST[1] - gyro_noST[1]);
    float gyrodz = (gyro_ST[2] - gyro_noST[2]);

    Serial.println("Gyro self-test results: ");
    Serial.print("x-axis = "); Serial.print(gyrodx); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
    Serial.print("y-axis = "); Serial.print(gyrody); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");
    Serial.print("z-axis = "); Serial.print(gyrodz); Serial.print(" dps"); Serial.println(" should be between 20 and 250 dps");

    float accdx = 1000.*(accel_ST[0] - accel_noST[0]);
    float accdy = 1000.*(accel_ST[1] - accel_noST[1]);
    float accdz = 1000.*(accel_ST[2] - accel_noST[2]);

    Serial.println("Accelerometer self-test results: ");
    Serial.print("x-axis = "); Serial.print(accdx); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
    Serial.print("y-axis = "); Serial.print(accdy); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");
    Serial.print("z-axis = "); Serial.print(accdz); Serial.print(" mg"); Serial.println(" should be between 60 and 1700 mg");

    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG10,   0x00); // disable self test
    delay(200);
}
// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void accelgyrocalLSM9DS1(float * dest1, float * dest2)
{  
    uint8_t data[6] = {0, 0, 0, 0, 0, 0};
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
    uint16_t samples, ii;

    // enable the 3-axes of the gyroscope
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG4, 0x38);
    // configure the gyroscope
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG1_G, Godr << 5 | Gscale << 3 | Gbw);
    delay(200);
    // enable the three axes of the accelerometer 
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG5_XL, 0x38);
    // configure the accelerometer-specify bandwidth selection with Abw
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG6_XL, Aodr << 5 | Ascale << 3 | 0x04 |Abw);
    delay(200);
    // enable block data update, allow auto-increment during multiple byte read
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG8, 0x44);

    // First get gyro bias
    byte c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable gyro FIFO  
    delay(50);                                                       // Wait for change to take effect
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable gyro FIFO stream mode and set watermark at 32 samples
    delay(1000);  // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

    for(ii = 0; ii < samples ; ii++) {            // Read the gyro data stored in the FIFO
        int16_t gyro_temp[3] = {0, 0, 0};
        readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_G, 6, &data[0]);
        gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
        gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

        gyro_bias[0] += (int32_t) gyro_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        gyro_bias[1] += (int32_t) gyro_temp[1]; 
        gyro_bias[2] += (int32_t) gyro_temp[2]; 
    }  

    gyro_bias[0] /= samples; // average the data
    gyro_bias[1] /= samples; 
    gyro_bias[2] /= samples; 

    dest1[0] = (float)gyro_bias[0]*gRes;  // Properly scale the data to get deg/s
    dest1[1] = (float)gyro_bias[1]*gRes;
    dest1[2] = (float)gyro_bias[2]*gRes;

    c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable gyro FIFO  
    delay(50);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable gyro bypass mode

    // now get the accelerometer bias
    c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c | 0x02);     // Enable accel FIFO  
    delay(50);                                                       // Wait for change to take effect
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x20 | 0x1F);  // Enable accel FIFO stream mode and set watermark at 32 samples
    delay(1000);  // delay 1000 milliseconds to collect FIFO samples

    samples = (readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_SRC) & 0x2F); // Read number of stored samples

    for(ii = 0; ii < samples ; ii++) {            // Read the accel data stored in the FIFO
        int16_t accel_temp[3] = {0, 0, 0};
        readBytes(LSM9DS1XG_ADDRESS, LSM9DS1XG_OUT_X_L_XL, 6, &data[0]);
        accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]);
        accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1]; 
        accel_bias[2] += (int32_t) accel_temp[2]; 
    }  

    accel_bias[0] /= samples; // average the data
    accel_bias[1] /= samples; 
    accel_bias[2] /= samples; 

    if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) (1.0/aRes);}  // Remove gravity from the z-axis accelerometer bias calculation
    else {accel_bias[2] += (int32_t) (1.0/aRes);}

    dest2[0] = (float)accel_bias[0]*aRes;  // Properly scale the data to get g
    dest2[1] = (float)accel_bias[1]*aRes;
    dest2[2] = (float)accel_bias[2]*aRes;

    c = readByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_CTRL_REG9, c & ~0x02);   //Disable accel FIFO  
    delay(50);
    writeByte(LSM9DS1XG_ADDRESS, LSM9DS1XG_FIFO_CTRL, 0x00);  // Enable accel bypass mode
}

void magcalLSM9DS1(float * dest1) 
{

    uint8_t data[6]; // data array to hold mag x, y, z, data
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

    // configure the magnetometer-enable temperature compensation of mag data
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG1_M, 0x80 | Mmode << 5 | Modr << 2); // select x,y-axis mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG2_M, Mscale << 5 ); // select mag full scale
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG3_M, 0x00 ); // continuous conversion mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG4_M, Mmode << 2 ); // select z-axis mode
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_CTRL_REG5_M, 0x40 ); // select block update mode

    //Serial.println("Mag Calibration: Wave device in a figure eight until done!");
    digitalWrite(LEDR, HIGH);
    delay(100);
    digitalWrite(LEDR, LOW);
    delay(4000);

    sample_count = 128;
    for(ii = 0; ii < sample_count; ii++) {
        int16_t mag_temp[3] = {0, 0, 0};
        readBytes(LSM9DS1M_ADDRESS, LSM9DS1M_OUT_X_L_M, 6, &data[0]);  // Read the six raw data registers into data array
        mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
        mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
        mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
        for (int jj = 0; jj < 3; jj++) {
            if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
            if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
        }
        delay(105);  // at 20 Hz ODR, new mag data is available every 50 ms
    }

    //    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
    //    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
    //    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
    mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
    mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

    dest1[0] = (float) mag_bias[0]*mRes;  // save mag biases in G for main program
    dest1[1] = (float) mag_bias[1]*mRes;   
    dest1[2] = (float) mag_bias[2]*mRes;          

    //write biases to accelerometermagnetometer offset registers as counts);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_L_M, (int16_t) mag_bias[0]  & 0xFF);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_X_REG_H_M, ((int16_t)mag_bias[0] >> 8) & 0xFF);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_L_M, (int16_t) mag_bias[1] & 0xFF);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Y_REG_H_M, ((int16_t)mag_bias[1] >> 8) & 0xFF);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_L_M, (int16_t) mag_bias[2] & 0xFF);
    writeByte(LSM9DS1M_ADDRESS, LSM9DS1M_OFFSET_Z_REG_H_M, ((int16_t)mag_bias[2] >> 8) & 0xFF);

    //Serial.println("Mag Calibration done!");
    digitalWrite(LEDR, HIGH);
}

// I2C read/write functions for the LSM9DS1and AK8963 sensors

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    Wire1.beginTransmission(address);  // Initialize the Tx buffer
    Wire1.write(subAddress);           // Put slave register address in Tx buffer
    Wire1.write(data);                 // Put data in Tx buffer
    Wire1.endTransmission();           // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    uint8_t data; // `data` will store the register data   
    Wire1.beginTransmission(address);         // Initialize the Tx buffer
    Wire1.write(subAddress);                  // Put slave register address in Tx buffer
    //  Wire.endTransmission(I2C_NOSTOP);        // Send the Tx buffer, but send a restart to keep connection alive
    Wire1.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
    //  Wire.requestFrom(address, 1);  // Read one byte from slave register address 
    Wire1.requestFrom(address, (size_t) 1);   // Read one byte from slave register address 
    data = Wire1.read();                      // Fill Rx buffer with result
    return data;                             // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire1.beginTransmission(address);   // Initialize the Tx buffer
    Wire1.write(subAddress);            // Put slave register address in Tx buffer
    //  Wire.endTransmission(I2C_NOSTOP);  // Send the Tx buffer, but send a restart to keep connection alive
    Wire1.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire1.requestFrom(address, count);  // Read bytes from slave register address 
    //        Wire.requestFrom(address, (size_t) count);  // Read bytes from slave register address 
    while (Wire1.available()) {
        dest[i++] = Wire1.read(); }         // Put read results in the Rx buffer
}

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];   // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;

}
