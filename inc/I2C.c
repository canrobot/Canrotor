#include <Board.h>

I2C_HandleTypeDef hi2c2;

void MX_I2C2_Init(void)
{
	__I2C2_CLK_ENABLE();
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;//0x68;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
  HAL_I2C_Init(&hi2c2);

}

void I2C_ByteWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	 // 010 value to write
	// 76543210 bit numbers
	// xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 6, 1);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
}

void I2C_BitWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
}

//void I2C_ByteRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t *data)
//{
//	// 01101001 read byte
//	// 76543210 bit numbers
//	//    xxx   args: bitStart=4, length=3
//	//    010   masked
//	//   -> 010 shifted
//	uint8_t tmp;
//	HAL_I2C_Mem_Read(&hi2c3, DevAddress, MemAddress, 1, &tmp, 1, 1);
//	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//	tmp &= mask;
//	tmp >>= (bitStart - length + 1);
//	*data = tmp;
//}

void I2C_ByteRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, MemAddSize, pData, Size, 1);
}

void I2C_BitRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
	*data = tmp & (1 << bitNum);
}
