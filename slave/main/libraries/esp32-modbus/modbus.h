#ifndef modbus_H
#define modbus_H

#include <esp_err.h>
#include <esp_log.h>
#include <mbedtls/sha256.h>
#include <esp_task_wdt.h>
#include <driver/uart.h>
#include <math.h>
#include "driver/gpio.h"

/**
	@file modbus.h
	@author Jose Morais

	Classe para uso do MODBUS RTU (via UART)


	MIT License

	Copyright (c) 2019 José Morais

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in all
	copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
	SOFTWARE.
*/

class MODBUS
{
	private:
			uart_port_t _uart; ///< Porta UART utilizada em operacoes.
			int8_t _pin_de; ///< GPIO para DE (modbus specific).
			int8_t _pin_re; ///< GPIO para RE (modbus specific).
			int32_t _baud; ///< Baud Rate.

			void write(uint8_t *array, uint8_t size);
			int32_t read();
			int16_t available();
			uint16_t CRC (const uint8_t *data, const uint16_t length);
			void DE(uint8_t status);
			void RE(uint8_t status);



	public:
			

			void init(uart_port_t uart, int32_t baud, int16_t buffer, int8_t pin_de, int8_t pin_re, int8_t pin_tx, int8_t pin_rx);
			void set_baudrate(int32_t baud);
			void flush();
			int8_t set(uint8_t id, uint16_t address, int32_t value);
			int32_t request(uint8_t id, uint8_t function, uint16_t address);
			


			

};


#endif