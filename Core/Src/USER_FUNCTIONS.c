/*
 * USER_FUNCTIONS.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */


#include "USER_FUNCTIONS.h"
/*
 * @brief	counts the number of non-null data in given array
 *
 * @param	buff	pointer to the array of data to be counted
 * @retval	int		number of non-null values in the array
 */
int buffersize(char *buff) {
	int i = 0;
	while (*buff++ != '\0')
		i++;
	return i;
}


/*
 * @brief	Outputs the string data to the PC, via USB CDC
 *
 * @param	fmt	pointer the array of characters data to be transmitted
 *
 * @retval	none
 */
void myprintf(const char *fmt, ...) {
	static char temp[100];
	va_list args;
	va_start(args, fmt);
	vsnprintf(temp, sizeof(temp), fmt, args);
	va_end(args);
	int len = buffersize(temp);
#ifdef CDC_USB_DEBUG
	CDC_Transmit_FS((uint8_t*) temp, len);

#endif
#ifdef UART_DEBUG
	uint32_t tick = uwTick;
	while (DEBUG_DATA_TX_FLAG != 0) {
		HAL_Delay(1);
		if (uwTick - tick > 5000) {
			DEBUG_DATA_TX_FLAG = 0;
		}
	}
	DEBUG_DATA_TX_FLAG = 0;
	HAL_UART_Transmit_IT(&DEBUG_STREAM, (uint8_t*) temp, len);
	while (DEBUG_DATA_TX_FLAG != 1) {

	}
	DEBUG_DATA_TX_FLAG = 0;
#endif
}

