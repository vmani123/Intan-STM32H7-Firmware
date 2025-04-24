/*
  Intan Technologies RHD STM32 Firmware Framework
  Version 1.1

  Copyright (c) 2024 Intan Technologies

  This file is part of the Intan Technologies RHD STM32 Firmware Framework.

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the “Software”), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.


  See <http://www.intantech.com> for documentation and product information.
  
 */

#ifndef INC_RHDINTERFACE_H_
#define INC_RHDINTERFACE_H_

#include "userconfig.h"
#include "rhdregisters.h"
#include "main.h"

// Communication Error Conditions.
// Errors 1-4 correspond to specific errors outlined in STM32U5 reference manual.
// Error 5 (Interrupt Clip) is when a TIM-triggered interrupt occurs before the previous
// sample interrupt routine finished execution, indicating that TIM period is too short
// (sample_interrupt_routine() takes too long), likely fixed by reducing sample rate by
// increasing TIM period or reducing number of commands per sequence.
// Error 6 (USART Transmission Error) indicates some issue with the transmission of
// data across USART.
typedef enum
{
	NoError = 0, // No Error
	TxDMAError = 1, // Transmit DMA Error
	RxDMAError = 2, // Receive DMA Error
	TxSPIError = 3, // Transmit SPI Error
	RxSPIError = 4, // Receive SPI Error
	ITClip = 5, // Interrupt Clip Error
	TxUSARTError = 6 // Transmit USART Error
} CommErrorStatus;

enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};

extern volatile uint16_t command_sequence_MOSI[CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE];
extern volatile uint32_t command_sequence_MISO[CONVERT_COMMANDS_PER_SEQUENCE + AUX_COMMANDS_PER_SEQUENCE];

extern volatile uint16_t sample_counter;
extern uint16_t *sample_memory;
extern uint32_t per_channel_sample_memory_capacity;

extern volatile uint16_t aux_command_list[AUX_COMMANDS_PER_SEQUENCE][AUX_COMMAND_LIST_LENGTH];
extern volatile uint8_t aux_command_index;

extern volatile int zcheck_DAC_command_slot_position;
extern volatile int zcheck_DAC_command_list_length;
extern volatile uint8_t zcheck_DAC_command_index;

extern volatile uint32_t command_transfer_state;

extern volatile uint8_t reception_in_progress;
extern volatile uint8_t main_loop_active;
extern volatile uint8_t uart_ready;

void sample_interrupt_routine();

void cycle_aux_commands();

void transfer_sequence_spi_dma();

void allocate_sample_memory();
void free_sample_memory();

void initialize_spi_with_dma();
void end_spi_with_dma();

void initialize_ddr_sclk_timers();
void end_ddr_sclk_timers();

void handle_comm_error(CommErrorStatus error_code);

void spi_rx_cplt_callback();
void spi_error_callback();

void write_initial_reg_values();

double calculate_sample_rate();

void create_convert_sequence(uint8_t* channel_numbers_to_convert);

int create_command_list_RHD_register_config(RHDConfigParameters *p, uint16_t *command_list, uint8_t calibrate, int num_commands);
int create_command_list_RHD_sample_aux_ins(uint16_t *command_list, int num_commands);
int create_command_list_RHD_update_DigOut(RHDConfigParameters *p, uint16_t *command_list, int num_commands);
int create_command_list_dummy(RHDConfigParameters *p, uint16_t *command_list, int n, uint16_t cmd);
int create_command_list_zcheck_DAC(RHDConfigParameters *p, uint16_t *command_list, double frequency, double amplitude);

void send_spi_command(uint16_t tx_data);
void send_receive_spi_command(uint16_t tx_data, uint16_t *rx_data_A, uint16_t *rx_data_B);
void extract_ddr_words(uint32_t merged_word, volatile uint16_t *word_A, volatile uint16_t *word_B);

#ifdef USE_HAL
extern UART_HandleTypeDef USART;
extern SPI_HandleTypeDef TRANSMIT_SPI;
extern SPI_HandleTypeDef RECEIVE_SPI;
extern TIM_HandleTypeDef INTERRUPT_TIM;
extern TIM_HandleTypeDef CS_DELAY_TIM;
extern TIM_HandleTypeDef RECEIVE_SCLK_TIM;
#else
void begin_spi_rx(uint32_t mem_increment, uint32_t mem_address, uint32_t num_words);
void begin_spi_tx(uint32_t mem_increment, uint32_t mem_address, uint32_t num_words);
void end_spi_rx();
void end_spi_tx();
void dma_interrupt_routine_rx();
void dma_interrupt_routine_tx();
void dma_interrupt_routine_usart_tx();
void spi_interrupt_routine_rx();
void spi_interrupt_routine_tx();
void uart_interrupt_routine();
#endif

// Write specified pin on specified port either high (1) or low (0).
static inline void write_pin(GPIO_TypeDef * gpio_port, uint32_t gpio_pin, int level)
{
#ifdef USE_HAL
	HAL_GPIO_WritePin(gpio_port, gpio_pin, level);
#else
	level ? LL_GPIO_SetOutputPin(gpio_port, gpio_pin) : LL_GPIO_ResetOutputPin(gpio_port, gpio_pin);
#endif
}

#endif /* INC_RHDINTERFACE_H_ */
