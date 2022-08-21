#include "modbus.h"


/**
	@brief Configura o Modbus para uso da UART

	@param [uart]: UART para ser usada
	@param [baud]: UART Baud Rate (>= 4800)
	@param [buffer]: UART RX Buffer (>128)
	@param [pin_de]: GPIO para Driver Enable (DE)
	@param [pin_re]: GPIO para Receiver Enable (RE)
	@param [pin_tx]: GPIO para UART TX
	@param [pin_rx]: GPIO para UART RX
*/
void MODBUS::init(uart_port_t uart, int32_t baud, int16_t buffer, int8_t pin_de, int8_t pin_re, int8_t pin_tx, int8_t pin_rx)
{
	if (baud < 4800)
		{ESP_LOGE("MODBUS::init", "Baud <4800"); return;}

	if (buffer < 129)
		{ESP_LOGE("MODBUS::init", "Buffer <129"); return;}


	uart_config_t uart_cfg;
	_pin_de = pin_de;
	_pin_re = pin_re;
	_baud = baud;
	_uart = uart;
	

	gpio_pad_select_gpio(gpio_num_t(_pin_de));
	gpio_set_direction(gpio_num_t(_pin_de), GPIO_MODE_OUTPUT);
	
	gpio_pad_select_gpio(gpio_num_t(_pin_re));
	gpio_set_direction(gpio_num_t(_pin_re), GPIO_MODE_OUTPUT);
	
	DE(0);
	RE(1);


	uart_cfg.baud_rate = _baud;
	uart_cfg.data_bits = UART_DATA_8_BITS;
	uart_cfg.parity = UART_PARITY_DISABLE;
	uart_cfg.stop_bits = UART_STOP_BITS_1;
	uart_cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_cfg.rx_flow_ctrl_thresh = 0;
	//uart_cfg.use_ref_tick = 0;
	uart_cfg.source_clk =  UART_SCLK_APB;
;

	uart_param_config(_uart, &uart_cfg);
	uart_set_pin(_uart, gpio_num_t(pin_tx), gpio_num_t(pin_rx), UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	uart_driver_install(_uart, buffer, 0, 0, NULL, 0);
	
	flush();
}

/**
  	@brief Reconfigura o baud rate
  
  	@param [baud]: Baud rate (bps)
 */
void MODBUS::set_baudrate(int32_t baud)
{
	uart_set_baudrate(_uart, baud);
}

/**
	@brief Escreve um pacote na UART.

	@param [*array]: Array 'a ser enviados.
	@param [size]: Tamanho do array.
 */
void MODBUS::write(uint8_t *array, uint8_t size)
{
	char tag[40]; snprintf(tag, 40, "MODBUS::%s", __func__);

	flush();
	for (uint8_t i = 0; i < size; i ++)
	{
		const char c = array[i];
		uart_write_bytes(_uart, &c, 1);
	}
	uart_wait_tx_done(_uart, pdMS_TO_TICKS(100));

	if (0)//Debug
	{
		printf("wpkt: ");
		for (uint8_t i = 0; i < size; i++)
		{
			printf("%02x ", array[i]);
		}printf("\r\n");
	}
}

/**
	@brief Retorna os Bytes contendo dados (a resposta em si) que nos enviaram.

	So funciona para dados de ate 2B.

	@return (int32_t): 2B de dados lido
	@return INT32_MIN: ERROR.
 */
int32_t MODBUS::read()
{
	char tag[40]; snprintf(tag, 40, "MODBUS::%s", __func__);
	uint8_t bff[16] = {0};
 
	for (uint8_t i = 0; i < 10; i++)
	{
		vTaskDelay(pdMS_TO_TICKS(10));

		if (available() > 5)
		{
			int8_t lgt = uart_read_bytes(_uart, bff, 16, pdMS_TO_TICKS(2));
			uint16_t crc = CRC(bff, lgt-2);

			if (1)//Debug
			{
				printf("rpkt: ");
				for (uint8_t j = 0; j < lgt; j++)
				{
					printf("%02x ", bff[j]);
				}printf("\r\n");
			}

			lgt--;//Total de bytes e' diferente do index para array (que comeca em 0)
			if ( (crc & 0xFF) == bff[lgt-1] && ((crc >> 8) & 0xFF) == bff[lgt])
			{
				int32_t value = INT32_MIN;

				if (bff[1] == 3 || bff[1] == 4)
				{
					//Slave, Function, Size, data[size], CRC_H, CRC_L
					value = 0;
					for (uint8_t j = 0; j < bff[2]; j++)
					{
						value += bff[3+j] << (((bff[2]-1)-j)*8);
					}
				}
				else if (bff[1] == 16)
				{
					//Slave, Function, Address, Regs written, CRC_H, CRC_L
					value = 1;
				}
				

				return value;
			}
			else
			{
				ESP_LOGW(tag, "Wrong CRC [%d, %d]", (bff[lgt] << 8)+bff[lgt-1], crc);
				return INT32_MIN+1;
			}
		}
	}

	return INT32_MIN;
}

/**
  	@brief Obtem os bytes disponiveis para leitura na UART.

	@return (int16_t): Bytes disponiveis.
 */
int16_t MODBUS::available()
{
	size_t total = 0;
	uart_get_buffered_data_len(_uart, &total);

	return total;
}

/**
  	@brief Calcula o CRC dum vetor.

	@param [*data]: Vetor.
	@param [length]: Tamanho do vetor.

	@return (uint16_t): CRC.
 */
uint16_t MODBUS::CRC(const uint8_t *data, uint16_t length)
{
	uint8_t nTemp;
	uint16_t wCRCWord = 0xFFFF;
	const uint16_t wCRCTable[] = 	{0x0000,0xC0C1,0xC181,0x0140,0xC301,0x03C0,0x0280,0xC241,0xC601,0x06C0,0x0780,0xC741,0x0500,0xC5C1,
									0xC481,0x0440,0xCC01,0x0CC0,0x0D80,0xCD41,0x0F00,0xCFC1,0xCE81,0x0E40,0x0A00,0xCAC1,0xCB81,0x0B40,
									0xC901,0x09C0,0x0880,0xC841,0xD801,0x18C0,0x1980,0xD941,0x1B00,0xDBC1,0xDA81,0x1A40,0x1E00,0xDEC1,
									0xDF81,0x1F40,0xDD01,0x1DC0,0x1C80,0xDC41,0x1400,0xD4C1,0xD581,0x1540,0xD701,0x17C0,0x1680,0xD641,
									0xD201,0x12C0,0x1380,0xD341,0x1100,0xD1C1,0xD081,0x1040,0xF001,0x30C0,0x3180,0xF141,0x3300,0xF3C1,
									0xF281,0x3240,0x3600,0xF6C1,0xF781,0x3740,0xF501,0x35C0,0x3480,0xF441,0x3C00,0xFCC1,0xFD81,0x3D40,
									0xFF01,0x3FC0,0x3E80,0xFE41,0xFA01,0x3AC0,0x3B80,0xFB41,0x3900,0xF9C1,0xF881,0x3840,0x2800,0xE8C1,
									0xE981,0x2940,0xEB01,0x2BC0,0x2A80,0xEA41,0xEE01,0x2EC0,0x2F80,0xEF41,0x2D00,0xEDC1,0xEC81,0x2C40,
									0xE401,0x24C0,0x2580,0xE541,0x2700,0xE7C1,0xE681,0x2640,0x2200,0xE2C1,0xE381,0x2340,0xE101,0x21C0,
									0x2080,0xE041,0xA001,0x60C0,0x6180,0xA141,0x6300,0xA3C1,0xA281,0x6240,0x6600,0xA6C1,0xA781,0x6740,
									0xA501,0x65C0,0x6480,0xA441,0x6C00,0xACC1,0xAD81,0x6D40,0xAF01,0x6FC0,0x6E80,0xAE41,0xAA01,0x6AC0,
									0x6B80,0xAB41,0x6900,0xA9C1,0xA881,0x6840,0x7800,0xB8C1,0xB981,0x7940,0xBB01,0x7BC0,0x7A80,0xBA41,
									0xBE01,0x7EC0,0x7F80,0xBF41,0x7D00,0xBDC1,0xBC81,0x7C40,0xB401,0x74C0,0x7580,0xB541,0x7700,0xB7C1,
									0xB681,0x7640,0x7200,0xB2C1,0xB381,0x7340,0xB101,0x71C0,0x7080,0xB041,0x5000,0x90C1,0x9181,0x5140,
									0x9301,0x53C0,0x5280,0x9241,0x9601,0x56C0,0x5780,0x9741,0x5500,0x95C1,0x9481,0x5440,0x9C01,0x5CC0,
									0x5D80,0x9D41,0x5F00,0x9FC1,0x9E81,0x5E40,0x5A00,0x9AC1,0x9B81,0x5B40,0x9901,0x59C0,0x5880,0x9841,
									0x8801,0x48C0,0x4980,0x8941,0x4B00,0x8BC1,0x8A81,0x4A40,0x4E00,0x8EC1,0x8F81,0x4F40,0x8D01,0x4DC0,
									0x4C80,0x8C41,0x4400,0x84C1,0x8581,0x4540,0x8701,0x47C0,0x4680,0x8641,0x8201,0x42C0,0x4380,0x8341,
									0x4100,0x81C1,0x8081,0x4040};


	while (length--)
	{
		nTemp = *data++ ^ wCRCWord;
		wCRCWord >>= 8;
		wCRCWord ^= wCRCTable[nTemp];
	}

	return wCRCWord;
}

/**
  	@brief Descarta todos dados no buffer RX da UART.
 */
void MODBUS::flush()
{
	uart_flush(_uart);
}

/**
  	@brief Controla a GPIO de Receiver Enable (RE).

	@param [status]: Nivel de GPIO.
 */
void MODBUS::RE(uint8_t status)
{
	gpio_set_level(gpio_num_t(_pin_re), status);
}

/**
  	@brief Controla a GPIO de Driver Enable (DE).

	@param [status]: Nivel de GPIO.
 */
void MODBUS::DE(uint8_t status)
{
	gpio_set_level(gpio_num_t(_pin_de), status);
}

/**
  	@brief Envia (apenas) um comando para slaves da rede (Modbus Write)

	@param [id]: Slave ID.
	@param [function]: Numero da funcao (RTU).
	@param [address]: Endereco a ser escrito.
	@param [value]: Valor a ser escrito.

	@return 1: OK.
	@return 0: ERROR.
 */
int8_t MODBUS::set(uint8_t id, uint16_t address, int32_t value)
{
	char tag[40] = {0}; snprintf(tag, 40, "MODBUS::%s", __func__);

	//Delay para 1 start|stop em modbus (3.5 bytes)
	int32_t dly = ceil((1/_baud)*10.0*4.0*1000000.0);

	uint8_t pck[11] = {0};
	uint16_t crc = 0;

	pck[0] = id;
	pck[1] = 16;
	pck[2] = (address >> 8);//MSB 8b
	pck[3] = (address & 0xFF);//LSB 8b
	pck[4] = 0;
	pck[5] = 1;//Quantidade fixa
	pck[6] = 2;//Bytes fixo
	pck[7] = (value >> 8);
	pck[8] = (value & 0xFF);

	crc = CRC(pck, 9);
	pck[9] = (crc & 0xFF);
	pck[10] = (crc >> 8);

	int32_t rcv = INT32_MIN;
	for (int8_t i = 0; i < 3; i++)
	{
		DE(1);//Master write enabled
		RE(1);//Slave write disabled
		ets_delay_us(dly);

		write(pck, 11);

		ets_delay_us(dly);
		DE(0);//Master write disabled
		RE(0);//Slave write enabled


		rcv = read();

		if (rcv == INT32_MIN+1)//Erro CRC, try again
			{rcv = INT32_MIN; continue;}

		
		DE(1);//Master write enabled
		RE(1);//Slave write disabled

		break;
	}

	if (rcv == 1)
		{return 1;}
	
	return 0;
}

/**
  	@brief Faz a leitura de um endereco Modbus.

	@param [id]: Slave ID.
	@param [function]: Numero da funcao (RTU).
	@param [address]: Endereco a ser lido.

	@return (int32_t): Resposta obtida.
	@return INT32_MIN: ERROR.
 */
int32_t MODBUS::request(uint8_t id, uint8_t function, uint16_t address)
{
	char tag[40] = {0}; snprintf(tag, 40, "MODBUS::%s", __func__);

	if (function != 3 && function != 4)
		{ESP_LOGE(tag, "Function error [%d]", function); return -1;}

	//Delay para 1 start|stop em modbus (3.5 bytes)
	int32_t dly = ceil((1/_baud)*10.0*4.0*1000000.0);

	uint8_t pck[8] = {0};
	uint16_t crc = 0;

	pck[0] = id;
	pck[1] = function;
	pck[2] = (address >> 8);//MSB 8b
	pck[3] = (address & 0xFF);//LSB 8b
	pck[4] = 0;
	pck[5] = 1;//Quantidade fixa
	crc = CRC(pck, 6);
	pck[6] = (crc & 0xFF);
	pck[7] = (crc >> 8);
	
	int32_t value = INT32_MIN;
	for (int8_t i = 0; i < 3; i++)
	{
		DE(1);//Master write enabled
		RE(1);//Slave write disabled
		ets_delay_us(dly);

		write(pck, 8);

		ets_delay_us(dly);
		DE(0);//Master write disabled
		RE(0);//Slave write enabled


		value = read();

		if (value == INT32_MIN+1)//Erro CRC, try again
			{value = INT32_MIN; continue;}

		
		DE(1);//Master write enabled
		RE(1);//Slave write disabled

		break;
	}

	return value;
}
