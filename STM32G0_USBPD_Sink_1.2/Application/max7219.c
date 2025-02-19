/*
 * max7219.c
 *
 *  Created on: May 11, 2019
 *      Author: tabur
 */

#include "../Application/max7219.h"

#define CS_SET() 	HAL_GPIO_WritePin(CS_MAX7219_GPIO_Port, CS_MAX7219_Pin, GPIO_PIN_RESET)
#define CS_RESET() 	HAL_GPIO_WritePin(CS_MAX7219_GPIO_Port, CS_MAX7219_Pin, GPIO_PIN_SET)

static uint8_t decodeMode = 0x00;

static uint8_t SYMBOLS[] = {
		0x7E,	// numeric 0
		0x30,	// numeric 1
		0x6D,	// numeric 2
		0x79,	// numeric 3
		0x33,	// numeric 4
		0x5B,	// numeric 5
		0x5F,	// numeric 6
		0x70,	// numeric 7
		0x7F,	// numeric 8
		0x7B,	// numeric 9
		0x01,	// minus
		0x4F,	// letter E
		0x37,	// letter H
		0x0E,	// letter L
		0x67,	// letter P
		0x00	// blank
};

static uint16_t getSymbol(uint8_t number);
static uint32_t lcdPow10(uint8_t n);
static MAX7219_Digits mapPosition(MAX7219_Digits newPosition, MAX7219_Segments segment);

void max7219_Init(uint8_t intensivity)
{
	max7219_Turn_On();
	max7219_SendData(REG_SCAN_LIMIT, NUMBER_OF_DIGITS - 1);
	max7219_SetIntensivity(intensivity);
	max7219_Clean();
}

void max7219_SetIntensivity(uint8_t intensivity)
{
	if (intensivity > 0x0F)
	{
		return;
	}

	max7219_SendData(REG_INTENSITY, intensivity);
}

void max7219_Clean()
{
	uint8_t clear = 0x00;

	if(decodeMode == 0xFF)
	{
		clear = BLANK;
	}

	for (int i = 0; i < 8; ++i)
	{
		max7219_SendData(i + 1, clear);
	}
}

void max7219_SendData(uint8_t addr, uint8_t data)
{
	CS_SET();
	HAL_SPI_Transmit(&SPI_PORT, &addr, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&SPI_PORT, &data, 1, HAL_MAX_DELAY);
	CS_RESET();
}

void max7219_Turn_On(void)
{
	max7219_SendData(REG_SHUTDOWN, 0x01);
}

void max7219_Turn_Off(void)
{
	max7219_SendData(REG_SHUTDOWN, 0x00);
}

void max7219_Decode_On(void)
{
	decodeMode = 0xFF;
	max7219_SendData(REG_DECODE_MODE, decodeMode);
}

void max7219_Decode_Off(void)
{
	decodeMode = 0x00;
	max7219_SendData(REG_DECODE_MODE, decodeMode);
}

/**
  * @brief  Function to map positions based on segment (for segment 2 map 4->8, 3->7 etc.)
  * @param  segment: Specify which segment (1/2) should be used
  * @param  newPosition: Starting position of printing
  * @retval MAX7219_Digits: return new mapped Position
  */

static MAX7219_Digits mapPosition(MAX7219_Digits newPosition, MAX7219_Segments segment)
{
	if (segment == 1)
	{
		return newPosition;
	}
	else if (segment == 2)
	{
		return newPosition +4;
	}
	return 0; // In case of invalid position
}

/**
  * @brief  Function to print a single digit value
  * @param  segment: Specify which segment (1/2) should be used
  * @param  position: Position of printing
  * @param  numeric: Digit value to be displayed
  * @param  point: Specify if decimal point should be displayed or not
  * @retval None
  */

void max7219_PrintDigit(MAX7219_Segments segment, MAX7219_Digits position, MAX7219_Numeric numeric, bool point)
{
	MAX7219_Digits mappedPosition = mapPosition(position, segment);
	if(mappedPosition > NUMBER_OF_DIGITS)
	{
		return;
	}

	if(point)
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(mappedPosition, getSymbol(numeric) | (1 << 7));
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(mappedPosition, numeric | (1 << 7));
		}
	}
	else
	{
		if(decodeMode == 0x00)
		{
			max7219_SendData(mappedPosition, getSymbol(numeric) & (~(1 << 7)));
		}
		else if(decodeMode == 0xFF)
		{
			max7219_SendData(mappedPosition, numeric & (~(1 << 7)));
		}
	}
}

/**
  * @brief  Function to display integer value with possibility to print decimal point
  * @param  segment: Specify which segment (1/2) should be used
  * @param  position: Starting position of printing
  * @param  value: Numeric value to be displayed
  * @param  decimal_position: Place of decimal point
  * @retval MAX7219_Digits: current cursor position
  */

MAX7219_Digits max7219_PrintItos(MAX7219_Segments segment, MAX7219_Digits position, int value, uint8_t decimal_position)
{
	max7219_SendData(REG_DECODE_MODE, 0xFF);

	int32_t i;
    int8_t num_digits = 0;

	if (value < 0)
	{
		if(position > 0)
		{
			max7219_SendData(position, MINUS);
			position--;
		}
		value = -value;
	}

	i = 1;

	//Get number of non-zero digits
	while ((value / i) > 9)
	{
		i *= 10;
		num_digits++;
	}
	num_digits++;

/*
	if(position > 0)
	{
		max7219_SendData(position, value/i);
		position--;
	}

	i /= 10;
*/

	//Print leading zeros and check for decimal point
	for (int j= 4; j > num_digits; j--) {
		if(j == decimal_position) {
			max7219_PrintDigit(segment, j, 0, true);
		}
		else {
			max7219_PrintDigit(segment, j, 0, false);
		}
	}


	//Print each number and decimal point
	while (i > 0)
	{
		if(position > 0)
		{	//If current number position is decimal point, print also decimal point
			if(position == decimal_position) {
				max7219_PrintDigit(segment, position, (value % (i * 10)) / i, true);
			}
			else {
				max7219_PrintDigit(segment, position, (value % (i * 10)) / i, false);
			}
			position--;
		}

		i /= 10;

	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}
/**
  * @brief  Function to print number of INT type with a fixed number of digits
  * @param  segment: Specify which segment (1/2) should be used
  * @param  position: Starting position of printing (4-1)
  * @param  value: Number to be printed
  * @param  n: Number of digits to be printed
  * @retval MAX7219_Digits: current cursor position
  */

MAX7219_Digits max7219_PrintNtos(MAX7219_Segments segment, MAX7219_Digits position, uint32_t value, uint8_t n)
{
	max7219_SendData(REG_DECODE_MODE, 0xFF);

	if (n > 0u)
	{
		uint32_t i = lcdPow10(n - 1u);

		while (i > 0u)	/* Display at least one symbol */
		{
			if(position > 0u)
			{
				max7219_PrintDigit(segment, position, (value / i) % 10u, false);
				position--;
			}

			i /= 10u;
		}
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}

/**
  * @brief  Function to print floating point numbers
  * @param  segment: Specify which segment (1/2) should be used
  * @param  position: Starting position of printing (4-1)
  * @param  value: Float numerical value to be displayed
  * @param  n: Number of characters after comma
  */

MAX7219_Digits max7219_PrintFtos(MAX7219_Segments segment, MAX7219_Digits position, float value, uint8_t n)
{
	if(n > 4)
	{
		n = 4;
	}

	max7219_SendData(REG_DECODE_MODE, 0xFF);

	if (value < 0.0)
	{
		if(position > 0)
		{
			max7219_PrintDigit(segment, position, MINUS, false);
			position--;
		}

		value = -value;
	}

	position = max7219_PrintItos(segment, position, (int32_t) value, 0);

	if (n > 0u)
	{
		max7219_PrintDigit(segment, position + 1, ((int32_t) value) % 10, true);

		position = max7219_PrintNtos(segment, position, (uint32_t) (value * (float) lcdPow10(n)), n);
	}

	max7219_SendData(REG_DECODE_MODE, decodeMode);

	return position;
}
/**
  * @brief  Function to blink a digit on specific position
  * @param  segment: Specify which segment (1/2) should be used
  * @param  valuePtr: Address of the value
  * @param  n: Digit which should be blinked
  * @param  ms: Blink delay in milliseconds
  * @param  decimal_position: place of decimal point
  * @retval None
  */

void max7219_BlinkDigit(MAX7219_Segments segment, int *valuePtr, uint8_t n, uint32_t ms, uint8_t decimal_position) {
	uint32_t blinkDelay = ms; // Delay in milliseconds (adjust as needed)
	uint16_t blinkDigit = n; // Digit to blink (0-3)
	uint8_t digit;

	//Print the BLANK and also decimal point
	if (blinkDigit == decimal_position) {
		max7219_PrintDigit(segment, blinkDigit, BLANK, true);
	}
	//Print the BLANK without decimal point
	else {
		max7219_PrintDigit(segment, blinkDigit, BLANK, false);
	}

	//Delay
	uint32_t blinkTimer = HAL_GetTick() + blinkDelay;
	while (HAL_GetTick() < blinkTimer);

	// Get the specific digit value at n position
    digit = (*valuePtr / lcdPow10(blinkDigit-1)) % 10;

	//Print back the original digit and also decimal point
	if (blinkDigit == decimal_position) {
		max7219_PrintDigit(segment, blinkDigit, digit, true);
	}
	//Print back the original digit without a decimal point
	else {
		max7219_PrintDigit(segment, blinkDigit, digit, false);
	}

	//Delay
	blinkTimer = HAL_GetTick() + blinkDelay;
	while (HAL_GetTick() < blinkTimer);
}

static uint16_t getSymbol(uint8_t number)
{
	return SYMBOLS[number];
}

static uint32_t lcdPow10(uint8_t n)
{
	uint32_t retval = 1u;

	while (n > 0u)
	{
		retval *= 10u;
		n--;
	}

	return retval;
}
