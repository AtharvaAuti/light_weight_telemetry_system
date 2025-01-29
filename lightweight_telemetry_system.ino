#include "stm32f4xx_hal.h"
#include <string.h>

// Pin Definitions
#define LORA_NSS_PIN    GPIO_PIN_4
#define LORA_NSS_PORT   GPIOA
#define LORA_RESET_PIN  GPIO_PIN_0
#define LORA_RESET_PORT GPIOB
#define LORA_DIO0_PIN   GPIO_PIN_1
#define LORA_DIO0_PORT  GPIOB

// SPI and UART Handles
SPI_HandleTc:\Users\naam\Downloads\string.hypeDef hspi1;
UART_HandleTypeDef huart2;
I2C_HandleTypeDef hi2c1;

// BMP280 Address (I2C)
#define BMP280_ADDR 0x76

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);

// LoRa Initialization
void LoRa_Init() {
  HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);
  HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
  HAL_Delay(100);

  // Configure LoRa Registers (Example: Set Frequency to 868 MHz)
  uint8_t reg;
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
  reg = 0x80; // RegOpMode (Sleep Mode)
  HAL_SPI_Transmit(&hspi1, &reg, 1, 100);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

// Read BMP280 Data
float BMP280_ReadTemperature() {
  uint8_t data[3];
  uint8_t reg = 0xFA; // Temperature MSB Register
  HAL_I2C_Master_Transmit(&hi2c1, BMP280_ADDR, &reg, 1, 100);
  HAL_I2C_Master_Receive(&hi2c1, BMP280_ADDR, data, 3, 100);
  // Convert raw data to temperature (see BMP280 datasheet for calibration)
  return 25.0; // Simplified example
}

// Transmit Data via LoRa
void LoRa_SendPacket(float lat, float lon, float alt, float temp) {
  char packet[64];
  sprintf(packet, "%.6f,%.6f,%.2f,%.2f", lat, lon, alt, temp);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, (uint8_t*)packet, strlen(packet), 100);
  HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
}

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();

  LoRa_Init();

  while (1) {
    // Read GPS Data (Example: UART Receive)
    float lat = 0.0, lon = 0.0; // Replace with actual UART parsing
    float altitude = BMP280_ReadTemperature(); // Simplified
    float temp = BMP280_ReadTemperature();

    LoRa_SendPacket(lat, lon, altitude, temp);
    HAL_Delay(1000); // 1 Hz update
  }
}

// Auto-generated HAL configurations (STM32CubeMX)
void MX_SPI1_Init(void) {
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  HAL_SPI_Init(&hspi1);
}

void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  HAL_I2C_Init(&hi2c1);
}