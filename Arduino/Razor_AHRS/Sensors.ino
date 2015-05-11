/* This file is part of the Razor AHRS Firmware */

// I2C code to read the sensors
//
// Sensor I2C addresses
#define ACCEL_ADDRESS ((int) 0x1D) // 
#define MAGN_ADDRESS  ((int) 0x1D) // 
#define GYRO_ADDRESS  ((int) 0x6B) // 

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif


void I2C_Init()
{
  Wire.begin();
}

void Accel_Init()
{
  
  //CTRL_REG0_XM 
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x1F); 
  WIRE_SEND(0x00); 
  Wire.endTransmission();
  delay(5);
  //CTRL_REG1_XM
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x20);
  WIRE_SEND(0x57);
  Wire.endTransmission();
  delay(5);
  //CTRL_REG2_XM
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x21);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
  //CTRL_REG3_XM 
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x22);
  WIRE_SEND(0x04);
  Wire.endTransmission();
  delay(5);
  
  uint8_t temp;
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x20);
  Wire.endTransmission(false);
  Wire.requestFrom(ACCEL_ADDRESS, 1); 
  temp = Wire.read();  
  // Then mask out the gyro ODR bits:
  temp &= 0xFF^(0xF << 4);
  // Then shift in our new ODR bits:
  uint8_t aRate = 0x5;
  temp |= (aRate << 4);
  //Set the acell ODR
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x20);
  WIRE_SEND(temp);
  Wire.endTransmission();
  delay(5);
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x21);
  Wire.endTransmission(false);
  Wire.requestFrom(ACCEL_ADDRESS, 1); 
  temp = Wire.read();  
  // Then mask out the accel scale bits:
  temp &= 0xFF^(0x3 << 3);
  // Then shift in our new scale bits:
  uint8_t aScl = 0x0;
  temp |= aScl << 3;
  // And write the new register value back into CTRL_REG2_XM:
  //Set the acell scale
  Wire.beginTransmission(ACCEL_ADDRESS);
  WIRE_SEND(0x21);
  WIRE_SEND(temp);
  Wire.endTransmission();
  delay(5);
  

  
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(ACCEL_ADDRESS); 
  WIRE_SEND(0x28);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.requestFrom(ACCEL_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    // No multiply by -1 for coordinate system transformation here, because of double negation:
    // We want the gravity vector, which is negated acceleration vector.
    accel[0] = (((int) buff[1]) << 8) | buff[0];  // X axis (internal sensor y axis)
    accel[1] = (((int) buff[3]) << 8) | buff[2];  // Y axis (internal sensor x axis)
    accel[2] = (((int) buff[5]) << 8) | buff[4];  // Z axis (internal sensor z axis)
  }
  else
  {
    num_accel_errors++;
    if (output_errors) Serial.println("!ERR: reading accelerometer");
  }
}

void Magn_Init()
{
  //CTRL_REG5_XM enables temp sensor,
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x24); 
  WIRE_SEND(0x94);  
  Wire.endTransmission();
  delay(5);
  
  //CTRL_REG6_XM sets the magnetometer full-scale
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x25);
  WIRE_SEND(0x00);  
  Wire.endTransmission();
  delay(5);
  
  //CTRL_REG7_XM sets magnetic sensor mode, low power mode, and filters  
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x26);
  WIRE_SEND(0x00); 
  Wire.endTransmission();
  delay(5);
 
  //CTRL_REG4_XM is used to set interrupt generators on INT2_XM
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x23);
  WIRE_SEND(0x04);
  Wire.endTransmission();
  delay(5);
  
  //INT_CTRL_REG_M to set push-pull/open drain, and active-low/high
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x12);
  WIRE_SEND(0x09);  
  Wire.endTransmission();
  delay(5);
  
  //Mad ODR
  uint8_t temp;
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x24);
  Wire.endTransmission(false);
  Wire.requestFrom(MAGN_ADDRESS, 1); 
  temp = Wire.read();  
  // Then mask out the mag ODR bits:
  temp &= 0xFF^(0x7 << 2);
  // Then shift in our new ODR bits:
  uint8_t mRate = 0x04;
  temp |= (mRate << 2);
  // And write the new register value back into CTRL_REG5_XM:
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x24);
  WIRE_SEND(temp);  
  Wire.endTransmission();
  delay(5);
  
  
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x25);
  Wire.endTransmission(false);
  Wire.requestFrom(MAGN_ADDRESS, 1); 
  temp = Wire.read();  
  // Then mask out the mag scale bits:
  temp &= 0xFF^(0x3 << 5);
  // Then shift in our new scale bits:
  uint8_t mScl = 0x00;
  temp |= mScl << 5;
  // And write the new register value back into CTRL_REG6_XM:
  Wire.beginTransmission(MAGN_ADDRESS);
  WIRE_SEND(0x25);
  WIRE_SEND(temp);  
  Wire.endTransmission();
  delay(5);
  
  
  
  
}

void Read_Magn()
{
  int i = 0;
  byte buff[6];
 
  Wire.beginTransmission(MAGN_ADDRESS); 
  WIRE_SEND(0x08);  // Send address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(MAGN_ADDRESS); 
  Wire.requestFrom(MAGN_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
// 9DOF Razor IMU SEN-10125 using HMC5843 magnetometer
#if HW__VERSION_CODE == 10125
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
// 9DOF Razor IMU SEN-10736 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10736
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // X axis (internal sensor -y axis)
    magnetom[1] = -1 * ((((int) buff[0]) << 8) | buff[1]);  // Y axis (internal sensor -x axis)
    magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
#elif (HW__VERSION_CODE == 10183) || (HW__VERSION_CODE == 10321)
    // MSB byte first, then LSB; X, Y, Z
    magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Z axis (internal sensor -z axis)
// 9DOF Sensor Stick SEN-10724 using HMC5883L magnetometer
#elif HW__VERSION_CODE == 10724
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (((int) buff[0]) << 8) | buff[1];         // X axis (internal sensor x axis)
    magnetom[1] = -1 * ((((int) buff[4]) << 8) | buff[5]);  // Y axis (internal sensor -y axis)
    magnetom[2] = -1 * ((((int) buff[2]) << 8) | buff[3]);  // Z axis (internal sensor -z axis)
#elif HW__VERSION_CODE == 9999
    // MSB byte first, then LSB; Y and Z reversed: X, Z, Y
    magnetom[0] = (((int) buff[1]) << 8) | buff[0];         // X axis (internal sensor x axis)
    magnetom[1] = ((((int) buff[3]) << 8) | buff[2]);  // Y axis (internal sensor -y axis)
    magnetom[2] = ((((int) buff[5]) << 8) | buff[4]);  // Z axis (internal sensor -z axis)
#endif
  }
  else
  {
    num_magn_errors++;
    if (output_errors) Serial.println("!ERR: reading magnetometer");
  }
}

void Gyro_Init()
{
  // CTRL_REG1_G sets output data rate, bandwidth, power-down and enables
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x20);
  WIRE_SEND(0x0F);
  Wire.endTransmission();
  delay(5);
  
  // CTRL_REG2_G sets up the HPF
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x21);
  WIRE_SEND(0x00);  // DLPF_CFG = 3, FS_SEL = 3
  Wire.endTransmission();
  delay(5);
  
  //  CTRL_REG3_G sets up interrupt and DRDY_G pins
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x22);
  WIRE_SEND(0x88);  //  SMPLRT_DIV = 10 (50Hz)
  Wire.endTransmission();
  delay(5);

  // CTRL_REG4_G sets the scale, update mode
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x23);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);  
  
  // CTRL_REG5_G sets up the FIFO, HPF, and INT1
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x23);
  WIRE_SEND(0x00);
  Wire.endTransmission();
  delay(5);
  
  //setGyroODR
  uint8_t temp;
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x20);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO_ADDRESS, 1); 
  temp = Wire.read();  
  // Then mask out the gyro ODR bits:
  temp &= 0xFF^(0xF << 4);
  // Then shift in our new ODR bits:
  uint8_t gRate = 0x00;
  temp |= (gRate << 4);
  // And write the new register value back into CTRL_REG1_G:
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x20);
  WIRE_SEND(temp);
  Wire.endTransmission();
  
  
  //setGyroScale
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x23);
  Wire.endTransmission(false);
  Wire.requestFrom(GYRO_ADDRESS, 1); 
  temp = Wire.read(); 
  // Then mask out the gyro scale bits:
  temp &= 0xFF^(0x3 << 4);
  // Then shift in our new scale bits:
  uint8_t gScl = 0x00;
  temp |= gScl << 4;
  // And write the new register value back into CTRL_REG4_G:
  Wire.beginTransmission(GYRO_ADDRESS);
  WIRE_SEND(0x23);
  WIRE_SEND(temp);
  Wire.endTransmission();
  
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  int i = 0;
  byte buff[6];
  
  Wire.beginTransmission(GYRO_ADDRESS); 
  WIRE_SEND(0x28);  // Sends address to read from
  Wire.endTransmission();
  
  Wire.beginTransmission(GYRO_ADDRESS);
  Wire.requestFrom(GYRO_ADDRESS, 6);  // Request 6 bytes
  while(Wire.available())  // ((Wire.available())&&(i<6))
  { 
    buff[i] = WIRE_RECEIVE();  // Read one byte
    i++;
  }
  Wire.endTransmission();
  
  if (i == 6)  // All bytes received?
  {
    gyro[0] = -1 * ((((int) buff[1]) << 8) | buff[0]);    // X axis (internal sensor -y axis)
    gyro[1] = -1 * ((((int) buff[3]) << 8) | buff[2]);    // Y axis (internal sensor -x axis)
    gyro[2] = -1 * ((((int) buff[5]) << 8) | buff[4]);    // Z axis (internal sensor -z axis)
  }
  else
  {
    num_gyro_errors++;
    if (output_errors) Serial.println("!ERR: reading gyroscope");
  }
}
