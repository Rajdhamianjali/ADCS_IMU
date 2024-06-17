/*
 * USER_FUNCTIONS.h
 *
 *  Created on: Jun 17, 2024
 *      Author: Anjali
 */

#ifndef INC_USER_FUNCTIONS_H_
#define INC_USER_FUNCTIONS_H_


#include "variables.h"
#include "main.h"
#include <stdarg.h>

int buffersize(char *buff);

void myprintf(const char *fmt, ...);
void myprintf_(const char *fmt, ...);

#endif /* INC_USER_FUNCTIONS_H_ */
