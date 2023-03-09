/**
 * @file ymodem.c
 * @brief ymodem升级
 * @details 细节
 * @mainpage 工程概览
 * @author 作者
 * @email 邮箱
 * @version 版本号
 * @date 年-月-日
 * @license 版权
 */
#ifdef __cplusplus
extern "C" {
#endif
 /* Includes ------------------------------------------------------------------*/
#include "ymodem_includes.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* 文件名字  */
unsigned char file_name[FILE_NAME_LENGTH];
unsigned char file_size[FILE_SIZE_LENGTH];
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
 * @brief              接受一个字节
 * @param  c: ·        接受的字节
 * @param  timeout:    超时时间
 * @retval 0: ····     收到字节
 *        -1:          超时
 */
int32_t Receive_Byte(uint8_t *c, uint32_t timeout)
{
	while (timeout-- > 0)
	{
		if (Ymodem_GetChar(c) == 1)
		{
			return 0;
		}
	}

	return -1;
}

/**
  * @brief      发送一个数据
  * @param  c:  一个字节
  * @retval 0:  默认返回0
  */
uint32_t Send_Byte(uint8_t c)
{
	Ymodem_PutChar(c);

	return 0;
}

/**
  * @brief      接受一个包
  * @param      接收数据
  * @param      数据长度
  * @param      超时时间
  *     0:      结束传输
  *    -1:      被发送者结束
  *    >0:      包长度
  * @retval 0:  正常返回
  *        -1:  超时或者包错误
  *         1:  被用户中断
  */
int32_t Receive_Packet(uint8_t* data, int32_t* length, uint32_t timeout)
{
	uint16_t i, packet_size;
	uint8_t c;
	*length = 0;

	/* 循环判断有没有数据 */
	if (Receive_Byte(&c, timeout) != 0)
	{
		return -1;
	}

	switch (c)
	{
		/* 128字节的数据或者数据头 目前xshell支持是128 */
		case SOH:
		{
			packet_size = PACKET_SIZE;
			break;
		}
		/* 1024的数据 */
		case STX:
		{
			packet_size = PACKET_1K_SIZE;
			break;
		}
		/* 传输结束 */
		case EOT:
		{
			return 0;
		}
		/* 传输终止 */
		case CA:
		{
			if ((Receive_Byte(&c, timeout) == 0) && (c == CA))
			{
				*length = -1;
				return 0;
			}
			else
			{
				return -1;
			}
		}
		case ABORT1:
		case ABORT2:
		{
			return 1;
		}
		default:
		{ 
			return -1; 
		}
	}
	*data = c;
	/* 接收数据大小128 + 5（首部和校验码）   */
	for (i = 1; i < (packet_size + PACKET_OVERHEAD); i++)
	{
		if (Receive_Byte(data + i, timeout) != 0)
		{
			return -1;
		}
	}
	/* 猜测是取反判断是否相等 */
	if (data[PACKET_SEQNO_INDEX] != ((data[PACKET_SEQNO_COMP_INDEX] ^ 0xff) & 0xff))
	{
		return -1;
	}
	*length = packet_size;

	return 0;
}

/**
 * @description: 显示ymodem信息
 * @detail: 
 * @return {*}
 * @author: lkc
 */
void Ymodem_ShowFileInfo(void)
{
	Y_DEBUG("you maybe receive success!\r\n");
	Y_DEBUG("file name[%s], file size[%s]\r\n", file_name, file_size);

	return;
}

/**
  * @brief  使用ymodem接受一个文件
  * @param  buf: 第一个字节的地址.
  * @retval 文件的大小.
  */
int32_t Ymodem_Receive(uint8_t* buf)
{
	uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD], * file_ptr, * buf_ptr;
	int32_t i, packet_length, session_done, file_done, packets_received, errors, session_begin, size = 0;
	uint32_t ramsource;
	STATIC uint8_t ucSendCNum = 0;

	for (session_done = 0, errors = 0, session_begin = 0; ;)
	{
		for (packets_received = 0, file_done = 0, buf_ptr = buf; ;)
		{
			switch (Receive_Packet(packet_data, &packet_length, YMODEM_TIMEOUT))
			{
			/* 如果接收成功 */
			case 0:
			{
				/* 错误标志清零 */
				errors = 0;
				switch (packet_length)
				{
					/* 被发送者终止 */
				case -1:
				{
					Send_Byte(ACK);
					return 0;
				}
				/* 结束传输 */
				case 0:
				{
					/* 修改该地方 */
					Send_Byte(NAK);
					file_done = 1;
					break;
				}
				default:
					/* 正常包 */
					if ((packet_data[PACKET_SEQNO_INDEX] & 0xff) != (packets_received & 0xff))
					{
						Send_Byte(NAK);
					}
					else
					{
						/* 如果是第一个起始帧 */
						if (packets_received == 0)
						{
							/* 文件名字包 */
							if (packet_data[PACKET_HEADER] != 0)
							{
								/* file_ptr+3 因为前三个字节是首部 再就是文件名字 */
								for (i = 0, file_ptr = packet_data + PACKET_HEADER; (*file_ptr != 0) && (i < FILE_NAME_LENGTH);)
								{
									file_name[i++] = *file_ptr++;
								}
								/* 获取到的文件名字字符串 */
								file_name[i++] = '\0';

								/* 文件名字之后紧接着就是文件大小 */
								for (i = 0, file_ptr++; (*file_ptr != ' ') && (i < FILE_SIZE_LENGTH);)
								{
									file_size[i++] = *file_ptr++;
								}
								file_size[i++] = '\0';

								/* 将文件大小转换为整数 */
								Str2Int(file_size, &size);

								/* 检测被发送镜像bin文件的大小 */
								/* 如果镜像bin文件比flash大 */
								if (size > (APPLICATION_SIZE + 1))
								{
									/* End session */
									Send_Byte(CA);
									Send_Byte(CA);

									/*debug 文件太大 flash太小 没有合适的空间 */
									return -1;
								}
								/* 擦除flash */
								//YMODEM_FLASH_Erase(APPLICATION_ADDRESS);
								Ymodem_Flash_Erase_App();
								Send_Byte(ACK);
								/* 收到第起始帧后立马回应 发送c */
								Send_Byte(CRC16);
							}
							/* 文件名字包为空，结束传输 */
							else
							{
								Send_Byte(ACK);
								file_done = 1;
								session_done = 1;
								break;
							}
						}
						/* 数据帧 */
						else
						{
							/* 计算进度 */

							/* 拷贝当前的数据 */
							memcpy(buf_ptr, packet_data + PACKET_HEADER, packet_length);
							ramsource = (uint32_t)buf;

							/* 往flash里边写数据 packet_length/4的原因是一个字占4个字节 */
							/* flash地址 ramsource写数据 packet_length 写入长度*/
							if (Ymodem_Flash_Write((uint32_t *)ramsource, (uint16_t)packet_length / 4) == 0)
							{
								Send_Byte(ACK);
							}
							else /* 当往flash写的时候，出现错误 */
							{
								/* 结束传输 */
								Send_Byte(CA);
								Send_Byte(CA);
								return -2;
							}

						}
						packets_received++;
						session_begin = 1;
					}
				}
				break;
			}
			case 1:
			{
				Send_Byte(CA);
				Send_Byte(CA);
				return -3;
			}
			default:	/* 这里其实是-1 */
			{
				if (session_begin > 0)
				{
					errors++;
				}
				if (errors > MAX_ERRORS)
				{
					Send_Byte(CA);
					Send_Byte(CA);
					return 0;
				}

				/* 每次延时500ms 发送6次 */
				if (YMODEM_C_NUM == ucSendCNum)
				{
					return ERROR_NO_ACK;
				}
				/* 延时 */
				Ymodem_Delay(500);

				ucSendCNum++;
				Send_Byte(CRC16);
				break;
			}
			}
			if (file_done != 0)
			{
				break;
			}
		}
		if (session_done != 0)
		{
			break;
		}
	}

	/* 再次接收到EOT之后 */
	Send_Byte(ACK);
	Send_Byte(CRC16);
	/* 最后接收结束帧 该帧其实不管对错 都已经传输完成 */
	Receive_Packet(packet_data, &packet_length, YMODEM_TIMEOUT);
	Send_Byte(ACK);

	return (int32_t)size;
}

/**
  * @brief  check response using the ymodem protocol
  * @param  buf: Address of the first byte
  * @retval The size of the file
  */
int32_t Ymodem_CheckResponse(uint8_t c)
{
	return 0;
}

/**
  * @brief  Prepare the first block
  * @param  timeout
  *     0: end of transmission
  * @retval None
  */
void Ymodem_PrepareIntialPacket(uint8_t* data, const uint8_t* fileName, uint32_t* length)
{
	uint16_t i, j;
	uint8_t file_ptr[10];

	/* Make first three packet */
	data[0] = SOH;
	data[1] = 0x00;
	data[2] = 0xff;

	/* Filename packet has valid data */
	for (i = 0; (fileName[i] != '\0') && (i < FILE_NAME_LENGTH);i++)
	{
		data[i + PACKET_HEADER] = fileName[i];
	}

	data[i + PACKET_HEADER] = 0x00;

	Int2Str(file_ptr, *length);
	for (j = 0, i = i + PACKET_HEADER + 1; file_ptr[j] != '\0'; )
	{
		data[i++] = file_ptr[j++];
	}

	for (j = i; j < PACKET_SIZE + PACKET_HEADER; j++)
	{
		data[j] = 0;
	}
}

/**
  * @brief  Prepare the data packet
  * @param  timeout
  *     0: end of transmission
  * @retval None
  */
void Ymodem_PreparePacket(uint8_t* SourceBuf, uint8_t* data, uint8_t pktNo, uint32_t sizeBlk)
{
	uint16_t i, size, packetSize;
	uint8_t* file_ptr;

	/* Make first three packet */
	packetSize = sizeBlk >= PACKET_1K_SIZE ? PACKET_1K_SIZE : PACKET_SIZE;
	size = sizeBlk < packetSize ? sizeBlk : packetSize;
	if (packetSize == PACKET_1K_SIZE)
	{
		data[0] = STX;
	}
	else
	{
		data[0] = SOH;
	}
	data[1] = pktNo;
	data[2] = (~pktNo);
	file_ptr = SourceBuf;

	/* Filename packet has valid data */
	for (i = PACKET_HEADER; i < size + PACKET_HEADER;i++)
	{
		data[i] = *file_ptr++;
	}
	if (size <= packetSize)
	{
		for (i = size + PACKET_HEADER; i < packetSize + PACKET_HEADER; i++)
		{
			data[i] = 0x1A; /* EOF (0x1A) or 0x00 */
		}
	}
}

/**
  * @brief  Update CRC16 for input byte
  * @param  CRC input value
  * @param  input byte
  * @retval None
  */
uint16_t UpdateCRC16(uint16_t crcIn, uint8_t byte)
{
	uint32_t crc = crcIn;
	uint32_t in = byte | 0x100;

	do
	{
		crc <<= 1;
		in <<= 1;
		if (in & 0x100)
			++crc;
		if (crc & 0x10000)
			crc ^= 0x1021;
	}

	while (!(in & 0x10000));

	return crc & 0xffffu;
}


/**
  * @brief  Cal CRC16 for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint16_t Cal_CRC16(const uint8_t* data, uint32_t size)
{
	uint32_t crc = 0;
	const uint8_t* dataEnd = data + size;

	while (data < dataEnd)
		crc = UpdateCRC16(crc, *data++);

	crc = UpdateCRC16(crc, 0);
	crc = UpdateCRC16(crc, 0);

	return crc & 0xffffu;
}

/**
  * @brief  Cal Check sum for YModem Packet
  * @param  data
  * @param  length
  * @retval None
  */
uint8_t CalChecksum(const uint8_t* data, uint32_t size)
{
	uint32_t sum = 0;
	const uint8_t* dataEnd = data + size;

	while (data < dataEnd)
		sum += *data++;

	return (sum & 0xffu);
}

/**
  * @brief  Transmit a data packet using the ymodem protocol
  * @param  data
  * @param  length
  * @retval None
  */
void Ymodem_SendPacket(uint8_t* data, uint16_t length)
{
	uint16_t i;
	i = 0;
	while (i < length)
	{
		Send_Byte(data[i]);
		i++;
	}
}

/**
  * @brief  使用ymodem传输文件
  * @param  buf: Address of the first byte
	sendFileName	发送文件名字
	sizeFile	发送文件大小
  * @retval The size of the file
  */
uint8_t Ymodem_Transmit(uint8_t* buf, const uint8_t* sendFileName, uint32_t sizeFile)
{

	uint8_t packet_data[PACKET_1K_SIZE + PACKET_OVERHEAD];
	uint8_t filename[FILE_NAME_LENGTH];
	uint8_t* buf_ptr, tempCheckSum;
	uint16_t tempCRC;
	uint16_t blkNumber;
	uint8_t receivedC[2], CRC16_F = 0, i;
	uint32_t errors, ackReceived, size = 0, pktSize;

	errors = 0;
	ackReceived = 0;
	/* 拷贝文件名字 */
	for (i = 0; i < (FILE_NAME_LENGTH - 1); i++)
	{
		filename[i] = sendFileName[i];
	}
	CRC16_F = 1;

	/* 准备第一包 */
	Ymodem_PrepareIntialPacket(&packet_data[0], filename, &sizeFile);

	do
	{
		/* Send Packet */
		Ymodem_SendPacket(packet_data, PACKET_SIZE + PACKET_HEADER);

		/* Send CRC or Check Sum based on CRC16_F */
		if (CRC16_F)
		{
			tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
			Send_Byte(tempCRC >> 8);
			Send_Byte(tempCRC & 0xFF);
		}
		else
		{
			tempCheckSum = CalChecksum(&packet_data[3], PACKET_SIZE);
			Send_Byte(tempCheckSum);
		}

		/* Wait for Ack and 'C' */
		if (Receive_Byte(&receivedC[0], 10000) == 0)
		{
			if (receivedC[0] == ACK)
			{
				/* Packet transferred correctly */
				ackReceived = 1;
			}
		}
		else
		{
			errors++;
		}
	} while (!ackReceived && (errors < 0x0A));

	if (errors >= 0x0A)
	{
		return errors;
	}
	buf_ptr = buf;
	size = sizeFile;
	blkNumber = 0x01;
	/* Here 1024 bytes package is used to send the packets */


	/* Resend packet if NAK  for a count of 10 else end of communication */
	while (size)
	{
		/* Prepare next packet */
		Ymodem_PreparePacket(buf_ptr, &packet_data[0], blkNumber, size);
		ackReceived = 0;
		receivedC[0] = 0;
		errors = 0;
		do
		{
			/* Send next packet */
			if (size >= PACKET_1K_SIZE)
			{
				pktSize = PACKET_1K_SIZE;

			}
			else
			{
				pktSize = PACKET_SIZE;
			}
			Ymodem_SendPacket(packet_data, pktSize + PACKET_HEADER);
			/* Send CRC or Check Sum based on CRC16_F */
			/* Send CRC or Check Sum based on CRC16_F */
			if (CRC16_F)
			{
				tempCRC = Cal_CRC16(&packet_data[3], pktSize);
				Send_Byte(tempCRC >> 8);
				Send_Byte(tempCRC & 0xFF);
			}
			else
			{
				tempCheckSum = CalChecksum(&packet_data[3], pktSize);
				Send_Byte(tempCheckSum);
			}

			/* Wait for Ack */
			if ((Receive_Byte(&receivedC[0], 100000) == 0) && (receivedC[0] == ACK))
			{
				ackReceived = 1;
				if (size > pktSize)
				{
					buf_ptr += pktSize;
					size -= pktSize;
					if (blkNumber == (APPLICATION_SIZE / 1024))
					{
						return 0xFF; /*  error */
					}
					else
					{
						blkNumber++;
					}
				}
				else
				{
					buf_ptr += pktSize;
					size = 0;
				}
			}
			else
			{
				errors++;
			}
		} while (!ackReceived && (errors < 0x0A));
		/* Resend packet if NAK  for a count of 10 else end of communication */

		if (errors >= 0x0A)
		{
			return errors;
		}

	}
	ackReceived = 0;
	receivedC[0] = 0x00;
	errors = 0;
	do
	{
		Send_Byte(EOT);
		/* Send (EOT); */
		/* Wait for Ack */
		if ((Receive_Byte(&receivedC[0], 10000) == 0) && receivedC[0] == ACK)
		{
			ackReceived = 1;
		}
		else
		{
			errors++;
		}
	} while (!ackReceived && (errors < 0x0A));

	if (errors >= 0x0A)
	{
		return errors;
	}

	/* Last packet preparation */
	ackReceived = 0;
	receivedC[0] = 0x00;
	errors = 0;

	packet_data[0] = SOH;
	packet_data[1] = 0;
	packet_data[2] = 0xFF;

	for (i = PACKET_HEADER; i < (PACKET_SIZE + PACKET_HEADER); i++)
	{
		packet_data[i] = 0x00;
	}

	do
	{
		/* Send Packet */
		Ymodem_SendPacket(packet_data, PACKET_SIZE + PACKET_HEADER);

		/* Send CRC or Check Sum based on CRC16_F */
		tempCRC = Cal_CRC16(&packet_data[3], PACKET_SIZE);
		Send_Byte(tempCRC >> 8);
		Send_Byte(tempCRC & 0xFF);

		/* Wait for Ack and 'C' */
		if (Receive_Byte(&receivedC[0], 10000) == 0)
		{
			if (receivedC[0] == ACK)
			{
				/* Packet transferred correctly */
				ackReceived = 1;
			}
		}
		else
		{
			errors++;
		}
	} while (!ackReceived && (errors < 0x0A));

	/* Resend packet if NAK  for a count of 10  else end of communication */
	if (errors >= 0x0A)
	{
		return errors;
	}

	do
	{
		Send_Byte(EOT);
		/* Send (EOT); */
		/* Wait for Ack */
		if ((Receive_Byte(&receivedC[0], 10000) == 0) && receivedC[0] == ACK)
		{
			ackReceived = 1;
		}
		else
		{
			errors++;
		}
	} while (!ackReceived && (errors < 0x0A));

	if (errors >= 0x0A)
	{
		return errors;
	}
	return 0; /* file transmitted successfully */
}

#ifdef __cplusplus
};
#endif
