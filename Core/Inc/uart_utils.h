/*
 * uart_utils.h
 *
 *  Created on: Mar 4, 2024
 *      Author: akswnd98
 */

#include "main.h"

#ifndef INC_UART_UTILS_H_
#define INC_UART_UTILS_H_

/* USART_TypeDef *hc12_huart = UART4;
USART_TypeDef *com_huart = USART2;

uint8_t hc12_transmit_buf[1000];
int cur_hc12_transmit_len = 0;
int cur_hc12_transmit_cnt = 0;
uint8_t hc12_transmit_lock = 0;
uint8_t com_transmit_buf[1000];
int cur_com_transmit_len = 0;
int cur_com_transmit_cnt = 0;
uint8_t com_transmit_lock = 0; */

void init_uarts ();
void do_uart_transmit_it (USART_TypeDef *huart, uint8_t *buf, int len, uint8_t *transmit_buf, int *cur_transmit_len, int *cur_transmit_cnt, uint8_t *transmit_lock);
void handle_uart_transmit_it (USART_TypeDef *huart, uint8_t *transmit_buf, int *cur_transmit_len, int *cur_transmit_cnt, uint8_t *transmit_lock);
void do_hc12_transmit_it (uint8_t *buf, int len);
void handle_hc12_transmit_it ();
void handle_hc12_receive_it ();
void handle_ia6b_receive_it ();
void do_com_transmit_it (uint8_t *buf, int len);
void handle_com_transmit_it ();
void handle_com_receive_it ();
void do_debug_transmit_it (uint8_t *buf, int len);
void handle_debug_transmit_it ();

#endif /* INC_UART_UTILS_H_ */
