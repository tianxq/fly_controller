/*******************************************************************************
 * @file:hal_usart.h
 */
#ifndef __HAL_USART_H
#define __HAL_USART_H

#ifndef __stdint_h
 #include <stdint.h>
#endif

#ifndef __stdbool_h
 #include <stdbool.h>
#endif
/*******************************************************************************
 *  Macro
 */
/******************************* ���ڻ���ַ ************************************/
#define USART1_DR_Base  0x40013804
#define USART2_DR_Base  0x40004404
#define USART3_DR_Base  0x40004804
#define USART4_DR_Base	0x40004C04

//Error
#define UART_TX_SUCCESS					0
#define UART_BUSY								1
#define UART_DATA_SIZE_ERR	    2

#define UART_READ_SUCCESS				0
#define UART_READ_TIMEOUT				1


//DMA
#define USART1_DMA_RX_MAX_LEN   1024
#define USART1_DMA_TX_MAX_LEN   128

#define USART3_DMA_RX_MAX_LEN   256
#define USART3_DMA_TX_MAX_LEN   128

//Time delay
#define OS_DELAY_MS(x)			OSTimeDlyHMSM(0,0,0,x)



/******************************************************************************
 *  Type
 */
/**
 * usart fifo status struct
 */
typedef struct {
	uint8_t *pBuffer;
	uint16_t tail;
	uint16_t head:10;//10 bit for max 1024
}uart_fifo_t;

/******************************************************************************
 *  Declare
 */
void hal_usart1_init(uint32_t baudrate);
uint16_t read_usart1_fifo(uint8_t * restrict pRxBuffer, uint16_t max_len);
uint16_t usart1_read_string(char * str, uint16_t max_str_len);
uint8_t serialport1_read_frame(char * str, char start_mark, char end_mark, uint16_t max_str_len, uint16_t timeout_ms);
uint8_t hal_usart1_write(uint8_t * restrict pdata, uint16_t size, uint16_t timeout_ms);
void hal_usart3_init(uint32_t baudrate);
uint8_t hal_usart3_write(uint8_t * restrict pdata, uint16_t size, uint16_t timeout_ms);
void hal_usart1_base_init(uint32_t baudrate, uint16_t parity, uint16_t stop_bit);
void hal_usart1_dma_config(uint8_t * const rxbuffer, uint32_t rxbuffer_size, uint8_t * const txbuffer, uint32_t txbuffer_size);
bool hal_usart1_strobe_write(uint32_t size);
#endif //__HAL_USART_H
