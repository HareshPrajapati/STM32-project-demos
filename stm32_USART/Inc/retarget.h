/*
 * retarget.h
 *
 *  Created on: 12-Apr-2021
 *      Author: Haesh_MikoToniks
 */

#ifndef _RETARGET_H__
#define _RETARGET_H__

#include "stm32l1xx_hal.h"
#include <sys/stat.h>

void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#endif //#ifndef _RETARGET_H__/
