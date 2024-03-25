/*
 * define_printf.c
 *
 *  Created on: Mar 5, 2024
 *      Author: akswnd98
 */

#include "uart_utils.h"

int _write (int fd, char *buf, int len) {
  do_debug_transmit_it((uint8_t *)buf, len);
  return len;
}
