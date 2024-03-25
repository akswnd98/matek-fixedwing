/*
 * uart_utils.c
 *
 *  Created on: Mar 4, 2024
 *      Author: akswnd98
 */

#include <stdio.h>
#include <string.h>
#include <tuning_receive.h>
#include "uart_utils.h"
#include "control_receive.h"

USART_TypeDef *ia6b_huart = USART2;
USART_TypeDef *hc12_huart = UART4;
USART_TypeDef *com_huart = UART4;
USART_TypeDef *debug_huart = UART5;

uint8_t hc12_transmit_buf[1000];
int cur_hc12_transmit_len;
int cur_hc12_transmit_cnt;
uint8_t hc12_transmit_lock;

uint8_t com_transmit_buf[1000];
int cur_com_transmit_len;
int cur_com_transmit_cnt;
uint8_t com_transmit_lock;

uint8_t debug_transmit_buf[1000];
int cur_debug_transmit_len;
int cur_debug_transmit_cnt;
uint8_t debug_transmit_lock;

void init_uarts () {
	LL_USART_EnableIT_RXNE(ia6b_huart);
	LL_USART_EnableIT_RXNE(com_huart);
}

void do_uart_transmit_it (USART_TypeDef *huart, uint8_t *buf, int len, uint8_t *transmit_buf, int *cur_transmit_len, int *cur_transmit_cnt, uint8_t *transmit_lock) {
	if (!(*transmit_lock)) {
		*transmit_lock = 1;
		memcpy(transmit_buf, buf, len);
		*cur_transmit_len = len;
		*cur_transmit_cnt = 0;
		LL_USART_EnableIT_TXE(huart);
	}
}

void handle_uart_transmit_it (USART_TypeDef *huart, uint8_t *transmit_buf, int *cur_transmit_len, int *cur_transmit_cnt, uint8_t *transmit_lock) {
	if (LL_USART_IsActiveFlag_TXE(huart)) {
		LL_USART_TransmitData8(huart, transmit_buf[*cur_transmit_cnt]);
		if (++(*cur_transmit_cnt) >= *cur_transmit_len) {
			LL_USART_DisableIT_TXE(huart);
			*transmit_lock = 0;
		}
	}
}

void do_hc12_transmit_it (uint8_t *buf, int len) {
	do_uart_transmit_it(hc12_huart, buf, len, hc12_transmit_buf, &cur_hc12_transmit_len, &cur_hc12_transmit_cnt, &hc12_transmit_lock);
}

void handle_hc12_transmit_it () {
	handle_uart_transmit_it(hc12_huart, hc12_transmit_buf, &cur_hc12_transmit_len, &cur_hc12_transmit_cnt, &hc12_transmit_lock);
}

void handle_hc12_receive_it () {
	if (LL_USART_IsActiveFlag_RXNE(hc12_huart)) {
		uint8_t data = LL_USART_ReceiveData8(hc12_huart);
		process_tuning_receive(data);
	}
}

void handle_ia6b_receive_it () {
	if (LL_USART_IsActiveFlag_RXNE(ia6b_huart)) {
		uint8_t data = LL_USART_ReceiveData8(ia6b_huart);
		process_control_receive(data);
	}
}

void do_com_transmit_it (uint8_t *buf, int len) {
	do_uart_transmit_it(com_huart, buf, len, com_transmit_buf, &cur_com_transmit_len, &cur_com_transmit_cnt, &com_transmit_lock);
}

void handle_com_transmit_it () {
	handle_uart_transmit_it(com_huart, com_transmit_buf, &cur_com_transmit_len, &cur_com_transmit_cnt, &com_transmit_lock);
}

void handle_com_receive_it () {
	/*if (LL_USART_IsActiveFlag_RXNE(com_huart)) {
		uint8_t data = LL_USART_ReceiveData8(com_huart);
		do_hc12_transmit_it(&data, 1);
	}*/
}

void do_debug_transmit_it (uint8_t *buf, int len) {
	do_uart_transmit_it(debug_huart, buf, len, debug_transmit_buf, &cur_debug_transmit_len, &cur_debug_transmit_cnt, &debug_transmit_lock);
}

void handle_debug_transmit_it () {
	handle_uart_transmit_it(debug_huart, debug_transmit_buf, &cur_debug_transmit_len, &cur_debug_transmit_cnt, &debug_transmit_lock);
}
