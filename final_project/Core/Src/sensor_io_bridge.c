/**
  ******************************************************************************
  * @file           : sensor_io_bridge.c
  * @brief          : Bridge for SENSOR_IO functions required by old BSP drivers
  ******************************************************************************
  */
#include "main.h"
#include "stm32l4xx_hal.h"

/* External handle from main.c */
extern I2C_HandleTypeDef hi2c2;

/**
  * @brief  Configures the I2C interface.
  */
void SENSOR_IO_Init(void)
{
  /* I2C2 is initialized in main.c MX_I2C2_Init() */
}

/**
  * @brief  DeInit the I2C interface.
  */
void SENSOR_IO_DeInit(void)
{
  /* Optional: HAL_I2C_DeInit(&hi2c2); */
}

/**
  * @brief  Write register value.
  * @param  Addr: Device address
  * @param  Reg: Register address
  * @param  Value: Data to write
  */
void SENSOR_IO_Write(uint8_t Addr, uint8_t Reg, uint8_t Value)
{
  HAL_I2C_Mem_Write(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);
}

/**
  * @brief  Read register value.
  * @param  Addr: Device address
  * @param  Reg: Register address
  * @retval Data read
  */
uint8_t SENSOR_IO_Read(uint8_t Addr, uint8_t Reg)
{
  uint8_t Value = 0;
  HAL_I2C_Mem_Read(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, &Value, 1, 1000);
  return Value;
}

/**
  * @brief  Read multiple register values.
  * @param  Addr: Device address
  * @param  Reg: Register address
  * @param  Buffer: Pointer to data buffer
  * @param  Length: Number of bytes to read
  * @retval 0 ON Success
  */
uint16_t SENSOR_IO_ReadMultiple(uint8_t Addr, uint8_t Reg, uint8_t *Buffer, uint16_t Length)
{
  return HAL_I2C_Mem_Read(&hi2c2, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Buffer, Length, 1000);
}
