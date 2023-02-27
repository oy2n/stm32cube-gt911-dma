#ifndef __GT911_H_
#define __GT911_H_

#include "stm32f4xx_hal.h"

#define GT911_MAX_WIDTH				1024    	//Touchscreen pad max width
#define GT911_MAX_HEIGHT			600			//Touchscreen pad max height

#define CT_CMD_WR					0XBA					//д��������
#define CT_CMD_RD   				0XBB					//����������

#define CT_MAX_TOUCH    			5						//���ݴ��������֧�ֵĵ���

#define Tuch_Read  	                HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

#define GT911_COMMAND_REG   		0x8040	/* ʵʱ���� */
#define GT911_CONFIG_REG			0x8047	/* ���ò����Ĵ��� */
#define GT911_PRODUCT_ID_REG 		0x8140 	/* Product ID */
#define GT911_FIRMWARE_VERSION_REG  0x8144  /* �̼��汾�� */
#define GT911_READ_XY_REG 			0x814E	/* ����Ĵ��� */

#define GT911_RST_CLK()				__GPIOB_CLK_ENABLE()
#define GT911_RST_PORT				GPIOB
#define GT911_RST_PIN				GPIO_PIN_4

#define GT911_INT_CLK()				__GPIOB_CLK_ENABLE()
#define GT911_INT_PORT				GPIOB
#define GT911_INT_PIN				GPIO_PIN_9


typedef struct
{
	uint8_t GT911_RST;
	uint8_t Touch;
	uint8_t TouchpointFlag;
	uint8_t TouchCount;

	uint8_t Touchkeytrackid[CT_MAX_TOUCH];
	uint16_t X[CT_MAX_TOUCH];
	uint16_t Y[CT_MAX_TOUCH];
	uint16_t S[CT_MAX_TOUCH];
}GT911_Dev;



void GT911_init(void);
void GT911_Scan(void);


#endif /*__GT911_H_*/
