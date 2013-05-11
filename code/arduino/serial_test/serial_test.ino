/********************************************

Redundant Array of Independent Gyroscopes Arduino Code

Collects data from 4 LSM330s and 4 MPU6050s and sends to a host for processing
Sends Accelerometer, Gyroscope, and Temperature data over serial to host computer
Message format: 3 message types: LSM330, MPU6050, and Timestamp
Message starts with message type byte.
Sensor messages are followed by a sensor ID byte, then 6 bytes gyro data (XYZ), then 6 bytes accelerometer
data (XYZ), then 1 or 2 bytes temperature data.
LSM330 messages have little-endian data, MPU6050 messages have big-endian data
Timestamp message is followed with 4 byte timestamp in milliseconds, and a checksum of entire message blob.
Data is associated with the next timestamp.

Author(s): Paul Doerr

Date: 5/6/2013

********************************************/

#include <Wire.h>
#include <SPI.h>

// Register addresses and commands for the sensors
// LSM330 Accelerometer
const byte LSM330_CTRL_REG1_A = 0x20; // Change data rate to 400Hz
const byte LSM330_CTRL_REG2_A = 0x21;
const byte LSM330_CTRL_REG3_A = 0x22;
const byte LSM330_CTRL_REG4_A = 0x23; // Turn on high resolution mode
const byte LSM330_CTRL_REG5_A = 0x24;
const byte LSM330_CTRL_REG6_A = 0x25;
const byte LSM330_STATUS_REG_A = 0x27; // XYZ data overwritten, new data available?
const byte LSM330_FIFO_CTRL_REG_A = 0x2E; // Change FIFO mode
const byte LSM330_FIFO_SRC_REG_A = 0x2F; // FIFO status
const byte LSM330_OUT_X_L_A = 0x28; // X low
const byte LSM330_OUT_X_H_A = 0x29; // X high
const byte LSM330_OUT_Y_L_A = 0x2A; // Y low
const byte LSM330_OUT_Y_H_A = 0x2B; // Y high
const byte LSM330_OUT_Z_L_A = 0x2C; // Z low
const byte LSM330_OUT_Z_H_A = 0x2D; // Z high
// LSM330 Gyroscope
const byte LSM330_CTRL_REG1_G = 0x20; // Change to normal mode, DRBW=1111
const byte LSM330_CTRL_REG2_G = 0x21;
const byte LSM330_CTRL_REG3_G = 0x22;
const byte LSM330_CTRL_REG4_G = 0x23;
const byte LSM330_CTRL_REG5_G = 0x24;
const byte LSM330_OUT_TEMP_G = 0x26; // Read temperature data
const byte LSM330_STATUS_REG_G = 0x27; // XYZ data overwritten, new data available?
const byte LSM330_FIFO_CTRL_REG_G = 0x2E; // Change FIFO mode
const byte LSM330_FIFO_SRC_REG_G = 0x2F; // FIFO status
const byte LSM330_OUT_X_L_G = 0x28; // X low
const byte LSM330_OUT_X_H_G = 0x29; // X high
const byte LSM330_OUT_Y_L_G = 0x2A; // Y low
const byte LSM330_OUT_Y_H_G = 0x2B; // Y high
const byte LSM330_OUT_Z_L_G = 0x2C; // Z low
const byte LSM330_OUT_Z_H_G = 0x2D; // Z high
// MPU6050
const byte MPU6050_SMPRT_DIV = 0x19; // Sample rate divider - set to 0
const byte MPU6050_CONFIG = 0x1A; // Set DLPF_CFG to 0x2
const byte MPU6050_GYRO_CONFIG = 0x1B; // Set FS_SEL to 0 (250 deg/s range)
const byte MPU6050_ACCEL_CONFIG = 0x1C; // Set AFS_SEL to 0 (2g)
const byte MPU6050_ACCEL_XOUT_H = 0x3B; // X high
const byte MPU6050_ACCEL_XOUT_L = 0x3C; // X low
const byte MPU6050_ACCEL_YOUT_H = 0x3D; // Y high
const byte MPU6050_ACCEL_YOUT_L = 0x3E; // Y low
const byte MPU6050_ACCEL_ZOUT_H = 0x3F; // Z high
const byte MPU6050_ACCEL_ZOUT_L = 0x40; // Z low
const byte MPU6050_TEMP_OUT_H = 0x41; // Temperature high
const byte MPU6050_TEMP_OUT_L = 0x42; // Temperature low
const byte MPU6050_GYRO_XOUT_H = 0x43; // X high
const byte MPU6050_GYRO_XOUT_L = 0x44; // X low
const byte MPU6050_GYRO_YOUT_H = 0x45; // Y high
const byte MPU6050_GYRO_YOUT_L = 0x46; // Y low
const byte MPU6050_GYRO_ZOUT_H = 0x47; // Z high
const byte MPU6050_GYRO_ZOUT_L = 0x48; // Z low
const byte MPU6050_FIFO_EN = 0x23; // Disable FIFO
const byte MPU6050_USER_CTRL = 0x6A; // Disable FIFO

// SPI Commands
const byte LSM330_WRITE = B00000000;
const byte LSM330_READ_MULT = B11000000;
const byte LSM330_READ_SING = B10000000;


// SPI chip select lines for the LSM330s
const byte NUM_LSM330 = 4;
const byte cs_g_pin[4] = {14, 16, 18, 2};
const byte cs_a_pin[4] = {15, 17, 19, 3};

// I2C Addresses
const byte PCA9546A_ADDR = B1110000;
const byte MPU6050_0_ADDR = B1101000;
const byte MPU6050_1_ADDR = B1101001;

// I2C Commands
const byte CHANNEL_0_EN = B01;
const byte CHANNEL_1_EN = B10;

// Message types
const byte MSG_LSM330 = 'L';
const byte MSG_MPU6050 = 'M';
const byte MSG_TIMESTAMP = 'T';


byte data_buffer[256];
byte data_size;
byte checksum;

void setup() 
{
  delay(1000);
  
  Serial.begin(115200);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE3);
  
  Wire.begin();

  // initalize chip select pins:
  for (byte i = 0; i < NUM_LSM330; i++) {
    pinMode(cs_g_pin[i], OUTPUT);
    pinMode(cs_a_pin[i], OUTPUT);
  }

  // LSM330 configuration
  for (byte i = 0; i < NUM_LSM330; i++) {
    // Accelerometers
    writeLSM330(LSM330_CTRL_REG1_A, B01110111, cs_a_pin[i]); // Set data rate
    writeLSM330(LSM330_CTRL_REG4_A, B000010, cs_a_pin[i]);  // High resolution mode
    // Gyroscopes
    writeLSM330(LSM330_CTRL_REG1_G, B11111111, cs_g_pin[i]); // Turn on, set data rate
  }
  
  // MPU6050 configuration
  selectChannel0();
  writeMPU6050(MPU6050_0_ADDR, MPU6050_SMPRT_DIV, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_CONFIG, 0x2);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_GYRO_CONFIG, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_ACCEL_CONFIG, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_FIFO_EN, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_USER_CTRL, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_SMPRT_DIV, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_CONFIG, 0x2);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_GYRO_CONFIG, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_ACCEL_CONFIG, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_FIFO_EN, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_USER_CTRL, 0x0);
  selectChannel1();
  writeMPU6050(MPU6050_0_ADDR, MPU6050_SMPRT_DIV, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_CONFIG, 0x2);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_GYRO_CONFIG, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_ACCEL_CONFIG, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_FIFO_EN, 0x0);
  writeMPU6050(MPU6050_0_ADDR, MPU6050_USER_CTRL, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_SMPRT_DIV, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_CONFIG, 0x2);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_GYRO_CONFIG, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_ACCEL_CONFIG, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_FIFO_EN, 0x0);
  writeMPU6050(MPU6050_1_ADDR, MPU6050_USER_CTRL, 0x0);
  
  // setup delay
  delay(10);
  
} // setup()

void loop() 
{ 
  data_size = 0;
  
  // Poll all sensors, then send data blob
  // LSM330s
  for (byte i = 0; i < NUM_LSM330; i++) {
    // Check for Gyro and Accel data
    if((readLSM330(LSM330_STATUS_REG_G, cs_g_pin[i]) & B1000) &&
        readLSM330(LSM330_STATUS_REG_A, cs_a_pin[i]) & B1000) {
      
      // Add Gyroscope message 
      addLSM330Msg(i);
    }
  }
  // MPU6050
  selectChannel0();
  addMPU6050Msg(4, MPU6050_0_ADDR);
  addMPU6050Msg(5, MPU6050_1_ADDR);
  selectChannel1();
  addMPU6050Msg(6, MPU6050_0_ADDR);
  addMPU6050Msg(7, MPU6050_1_ADDR);
  
  // Timestamp + checksum:w
  
  addTimestampeMsg();
  addChecksumMsg();
  
  // Send data to host
  Serial.write(data_buffer, data_size);
  
} // loop()

/***************** BUFFER COMMANDS ************************/

void addLSM330Msg(byte i)
{
  // Add message type and ID bytes
  data_buffer[data_size++] = MSG_LSM330;
  data_buffer[data_size++] = i;
  
  // Add 6 bytes of gyroscope data
  byte temp[6];
  readLSM330(LSM330_OUT_X_L_G, cs_g_pin[i], 6, data_buffer+data_size);
  data_size += 6;

  // Add 6 bytes of accelerometer data
  readLSM330(LSM330_OUT_X_L_A, cs_a_pin[i], 6, data_buffer+data_size);
  data_size += 6;
  
  // Add 2 bytes temperature data
  readLSM330(LSM330_OUT_TEMP_G, cs_g_pin[i], 1, data_buffer+data_size);
  data_size++;
}

void addMPU6050Msg(byte i, byte addr)
{
  // Add message type and ID bytes
  data_buffer[data_size++] = MSG_MPU6050;
  data_buffer[data_size++] = i;
  
  // Add 6 bytes gyro data
  readMPU6050(addr, MPU6050_GYRO_XOUT_H, 6, data_buffer+data_size);
  data_size += 6; 
  
  // Add 6 bytes :waccelerometer data
  readMPU6050(addr, MPU6050_ACCEL_XOUT_H, 6, data_buffer+data_size);
  data_size += 6; 
  
  // Add 2 bytes temperature data
  readMPU6050(addr, MPU6050_TEMP_OUT_H, 2, data_buffer+data_size);
  data_size += 2; 
}

void addTimestampMsg()
{
  data_buffer[data_size++] = MSG_TIMESTAMP;
  unsigned long time_millis = millis();
  data_buffer[data_size++] = (byte)((time_millis >> 24) & 0xFF);
  data_buffer[data_size++] = (byte)((time_millis >> 16) & 0xFF);
  data_buffer[data_size++] = (byte)((time_millis >> 8) & 0xFF);
  data_buffer[data_size++] = (byte)(time_millis & 0xFF);
}

void addChecksumMsg()
{
  byte checksum = 0;
  
  for (int i = 0; i < data_size; i++) {
    checksum ^= data_buffer[i];
  }
  
  data_buffer[data_size++] = checksum;
}

/***************** SPI COMMANDS ************************/

// Read from LSM330
// Needs: read register address, arduino chip select line, num bytes to read, array to store data
void readLSM330(byte regAddr, byte chipSelect, byte numBytes, byte* data) 
{ 
  // Conbine register address and command
  byte cmd_addr = regAddr | LSM330_READ_MULT;
  // select device
  digitalWrite(chipSelect, LOW);
  SPI.transfer(cmd_addr); // Send register address
  for (int i = 0; i < numBytes; i++) {
    data[i] = SPI.transfer(0x00); // Read next byte
  }
  
  // de-select device
  digitalWrite(chipSelect, HIGH);
}

// Read from LSM330
// Needs: read register address, arduino chip select line, num bytes to read, array to store data
byte readLSM330(byte regAddr, byte chipSelect) 
{ 
  // Conbine register address and command
  byte cmd_addr = regAddr | LSM330_READ_SING;
  // select device
  digitalWrite(chipSelect, LOW);
  SPI.transfer(cmd_addr); // Send register address
  byte result = SPI.transfer(0x00); // Read byte
  
  // de-select device
  digitalWrite(chipSelect, HIGH);

  return result;
}

// Sends a write command to LSM330
// Needs: write register address, value to write, arduino chip select line
void writeLSM330(byte regAddr, byte value, byte chipSelect) 
{
  // now combine the register address and the command into one byte:
  byte cmd_addr = regAddr | LSM330_WRITE;

  // take the chip select low to select the device:
  digitalWrite(chipSelect, LOW);

  SPI.transfer(cmd_addr); // Send register address
  SPI.transfer(value);  //Send value to record into register

  // take the chip select high to de-select:
  digitalWrite(chipSelect, HIGH);
}


/***************** I2C COMMANDS ************************/

// Connect PCA9546A to channel 0 sensors
void selectChannel0()
{
  Wire.beginTransmission(PCA9546A_ADDR);
  Wire.write(CHANNEL_0_EN);
  Wire.endTransmission(true);
}

// Connect PCA9546A to channel 1 sensors
void selectChannel1()
{
  Wire.beginTransmission(PCA9546A_ADDR);
  Wire.write(CHANNEL_1_EN);
  Wire.endTransmission(true);
}

// Read from MPU6050
// Needs: MPU6050 address, register address, num bytes, pointer to store data
void readMPU6050(byte devAddr, byte regAddr, byte numBytes, byte* data) 
{
  // Send starting register address to read
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.endTransmission(false);
  // Read the registers
  Wire.requestFrom(devAddr, numBytes, (byte)true);
  for (int i = 0; Wire.available(); i++) {
    data[i] = Wire.read();
  }
}

// Write to MPU6050
// Needs: MPU6050 address, register address, data to write
void writeMPU6050(byte devAddr, byte regAddr, byte data)
{
  Wire.beginTransmission(devAddr);
  Wire.write(regAddr);
  Wire.write(data);
  Wire.endTransmission(true);
}
