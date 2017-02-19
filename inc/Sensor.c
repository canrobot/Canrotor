#include "Board.h"

uint8_t rawADC[12];
float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};
float SelfTest[6] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accBias[3] = {0, 0, 0};

float magBias[3], magScale[3];
float gyro_cal[3] = {0, 0, 0};
float mag_cal[3] = {0, 0, 0};
int count = 16;
extern int Flight_Status;
imu_t imu;

//#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = (int16_t)X; imu.accADC[PITCH]  = (int16_t)Y; imu.accADC[YAW]  =  (int16_t)Z;}
//#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = (int16_t)X; imu.gyroADC[PITCH] = (int16_t)Y; imu.gyroADC[YAW] = (int16_t)Z;}
//#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = (int16_t)X; imu.magADC[PITCH]  = (int16_t)Y; imu.magADC[YAW]  =  (int16_t)Z;}

#define AMult 8.0f / 32768.0f
#define GMult 2000.0f / 32768.0f
#define MMult 10.0f * 4912.0f / 32768.0f

void MPU9250_Init(void)
{
	//  MPU9250SelfTest(SelfTest);
	//  calibrateMPU9250(gyroBias, accBias);
	// ACC Gyro_Init
    //calibrateMPU9250(gyroBias, accBias);
    I2C_BitWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, DISABLE); // Clear sleep mode bit (6), enable all sensors
    HAL_Delay(100); // for stability
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x01);// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001;
    HAL_Delay(200); // for stability
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_CONFIG, MPU9250_CFG_DLPF_CFG_BIT, MPU9250_CFG_DLPF_CFG_LENGTH, MPU9250_DLPF_BW_42); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
    //I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, 0x1B, 1, 2, 0x03);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x04);  //SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, MPU9250_GCONFIG_FS_SEL_LENGTH, MPU9250_GYRO_FS_2000); //GYRO_CONFIG   -- FS_SEL = 2: Full scale set to 1000 deg/sec
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, MPU9250_ACONFIG_AFS_SEL_LENGTH, MPU9250_ACCEL_FS_8); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, 0x1D, 3, 4, 0x03); //0x03
    //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
    //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

    I2C_BitWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, ENABLE);  // enable I2C bypass for AUX I2C

    //MPU9150_I2C_BitWrite(MPU9150_Address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
    //MPU9150_I2C_BitWrite(MPU9150_Address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
	HAL_Delay(100); // for stability

	//Compass_Init
	initAK8963(magCalibration);
}
void Calibration(void)
{
	int cal_int = 0;
	uint8_t axis = 0;

	for (cal_int = 0; cal_int < 2000; cal_int ++){
		if(cal_int % 125 == 0){
		count--;
		}
		for(axis=0; axis<3; axis++)
		{
			Gyro_getADC();
			gyro_cal[axis] += (float)imu.gyroADC[axis];
			Mag_getADC();
			mag_cal[axis] += (float)imu.magADC[axis];
		}

	}
	PrintData(2);
	for(axis=0; axis<3; axis++)
	{
		Gyro_getADC();
		gyro_cal[axis] /= 2000.0f;
		Mag_getADC();
		mag_cal[axis] /= 2000.0f;
	}
	PrintData(2);
}
void GYRO_Common(void)
{
	uint8_t axis =0;
	for(axis=0; axis<3; axis++)
	{
		imu.gyroRaw[axis] = ((float)imu.gyroADC[axis] - gyro_cal[axis]) * GMult;
	}
}

void ACC_Common(void)
{
	uint8_t axis = 0;
	for(axis=0;axis<3;axis++)
	{
		imu.accRaw[axis] = (float)imu.accADC[axis] * AMult;
	}
}

void MAG_Common(void)
{
	uint8_t axis = 0;
	for(axis=0;axis<3;axis++)
	{
		imu.magRaw[axis] = (float)imu.magADC[axis] * MMult ; // * magCalibration[axis] - magBias[axis];
	}
}

void Gyro_getADC(void)
{
	I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6);
	 /* Get Angular rate */
	imu.gyroADC[0] = ((int16_t)rawADC[0]<<8) | rawADC[1];
	imu.gyroADC[1] = ((int16_t)rawADC[2]<<8) | rawADC[3];
	imu.gyroADC[2] = ((int16_t)rawADC[4]<<8) | rawADC[5];
	if(Flight_Status >= 1) GYRO_Common();
}

void ACC_getADC(void)
{
	I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6);
	/* Get acceleration */
	imu.accADC[0] = ((int16_t)rawADC[0]<<8) | rawADC[1];
	imu.accADC[1] = ((int16_t)rawADC[2]<<8) | rawADC[3];
	imu.accADC[2] = ((int16_t)rawADC[4]<<8) | rawADC[5];
	if(Flight_Status >= 1) ACC_Common();
}

void Mag_getADC(void)
{
	I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, 0x02, 1, rawADC, 1);
	if( rawADC[0] & 0x01)
	{
	I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, 0x03, 1, rawADC, 7);
	uint8_t c = rawADC[6];
	if(!(c & 0x08)){
	imu.magADC[0] = ((int16_t)rawADC[1]<<8) | rawADC[0];
	imu.magADC[1] = ((int16_t)rawADC[3]<<8) | rawADC[2];
	imu.magADC[2] = ((int16_t)rawADC[5]<<8) | rawADC[4];
	//I2C_BitWrite(MPU9250_RA_MAG_ADDRESS, 0x0A, 0, ENABLE);
	if(Flight_Status >= 1) MAG_Common();
		}
	}
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x80);// Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100); // for stability
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x01);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_2, 7, 8, 0x00);
  HAL_Delay(200); // for stability

// Configure device for bias calculation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_INT_ENABLE, 7, 8, 0x00);// Disable all interrupts
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_FIFO_EN, 7, 8, 0x00);// Disable FIFO
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x00);// Turn on internal clock source
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_I2C_MST_CTRL, 7, 8, 0x00);// Disable I2C master
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_USER_CTRL, 7, 8, 0x00);// Disable FIFO and I2C master modes
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_USER_CTRL, 7, 8, 0x0C);// Reset FIFO and DMP
  HAL_Delay(15); // for stability

// Configure MPU9250 gyro and accelerometer for bias calculation.

  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_CONFIG, 7, 8, 0x01); // Set low-pass filter to 188 Hz
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x00);// Set sample rate to 1 kHz
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_CONFIG, 7, 8, 0x00);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0x00);// Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_USER_CTRL, 7, 8, 0x40); // Enable FIFO
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_FIFO_EN, 7, 8, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_FIFO_EN, 7, 8, 0x00);// Disable gyro and accelerometer sensors for FIFO
  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_FIFO_COUNTH, 1, rawADC, 2);// read FIFO sample count
  fifo_count = ((uint16_t)rawADC[0] << 8) | rawADC[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW,MPU9250_RA_FIFO_R_W, 1, rawADC, 12);// read data for averaging

    accel_temp[0] = (int16_t) (((int16_t)rawADC[0] << 8) | rawADC[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)rawADC[2] << 8) | rawADC[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)rawADC[4] << 8) | rawADC[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)rawADC[6] << 8) | rawADC[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)rawADC[8] << 8) | rawADC[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)rawADC[10] << 8) | rawADC[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
*/
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_XA_OFFSET_H, 1, rawADC, 2);// Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];
  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_YA_OFFSET_H, 1, rawADC, 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];
  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ZA_OFFSET_H, 1, rawADC, 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
*/
// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
void initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, 0x0A, 7, 8, 0x00);// Power down magnetometer
	  HAL_Delay(10);
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, 0x0A, 7, 8, 0x0F);// Enter Fuse ROM access mode
	  HAL_Delay(10);
	  I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, 0x10, 1, rawADC, 3);// Read the x-, y-, and z-axis calibration values
	  destination[0] =  (float)(rawADC[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	  destination[1] =  (float)(rawADC[1] - 128)/256.0f + 1.0f;
	  destination[2] =  (float)(rawADC[2] - 128)/256.0f + 1.0f;
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, 0x0A, 7, 8, 0x00);// Power down magnetometer
	  HAL_Delay(10);
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, 0x0A, 7, 8, 0x16); // Set magnetometer data resolution and sample ODR
	  HAL_Delay(10);

	  //magcalMPU9250(magBias, magScale);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t selfTest[6] = {0, 0, 0, 0, 0, 0};
   uint16_t i, ii;
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;

   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x00);// Set gyro sample rate to 1 kHz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_CONFIG, 7, 8, 0x02);// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_CONFIG, 7, 8, 0x00);// Set full scale range for the gyro to 250 dps
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, 0x1D, 7, 8, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0x00);// Set full scale range for the accelerometer to 2 g

  for(ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6);  // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;

  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6); // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_CONFIG, 7, 8, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(25); // Delay a while to let the device stabilize

  for(ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6); // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;

  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6); // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0x00);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_LOW, MPU9250_RA_GYRO_CONFIG, 7, 8, 0x00);
  HAL_Delay(25); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, 0x0D, 1, rawADC, 3);
   selfTest[0] = rawADC[0]; // X-axis accel self-test results
   selfTest[1] = rawADC[1]; // Y-axis accel self-test results
   selfTest[2] = rawADC[2]; // Z-axis accel self-test results
   I2C_ByteRead(MPU9250_ADDRESS_AD0_LOW, 0x00, 1, rawADC, 3);
   selfTest[3] = rawADC[0]; // X-axis gyro self-test results
   selfTest[4] = rawADC[1]; // Y-axis gyro self-test results
   selfTest[5] = rawADC[2]; // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (i = 0; i < 3; i++) {
     destination[i] = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i]; // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
   }

}

void magcalMPU9250(float * dest1, float * dest2)
 {
 uint16_t ii = 0, sample_count = 0;
 int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
 int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

 sample_count = 128;
 for(ii = 0; ii < sample_count; ii++) {
  Mag_getADC();  // Read the mag data
  mag_temp[0] = imu.magADC[0];
  mag_temp[1] = imu.magADC[1];
  mag_temp[2] = imu.magADC[2];
 for (int jj = 0; jj < 3; jj++) {
  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
 }
 HAL_Delay(135);  // at 8 Hz ODR, new mag data is available every 125 ms
 }

// Get hard iron correction
 mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

 dest1[0] = (float) mag_bias[0]*MMult*magCalibration[0];  // save mag biases in G for main program
 dest1[1] = (float) mag_bias[1]*MMult*magCalibration[1];
 dest1[2] = (float) mag_bias[2]*MMult*magCalibration[2];

// Get soft iron correction estimate
 mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
 mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
 mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

 float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 avg_rad /= 3.0;

 dest2[0] = avg_rad/((float)mag_scale[0]);
 dest2[1] = avg_rad/((float)mag_scale[1]);
 dest2[2] = avg_rad/((float)mag_scale[2]);


 }
