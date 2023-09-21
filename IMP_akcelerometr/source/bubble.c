/*
 * František Horázný
 * xhoraz02
 * cca 60%
 * Změna kromě funkce deleay (vlastní funkce) pouze v main funkci
 * Veškerá práce s daty ze senzorů - podle zadání
 **/


/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "fsl_debug_console.h"
#include "board.h"
#include "math.h"
#include "fsl_mma.h"
#include "fsl_tpm.h"

#include "fsl_common.h"
#include "pin_mux.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* The Flextimer instance/channel used for board */
#define BOARD_TIMER_BASEADDR TPM2
#define BOARD_FIRST_TIMER_CHANNEL 0U
#define BOARD_SECOND_TIMER_CHANNEL 1U
#define BOARD_FIRST_CHANNEL_INT kTPM_Chnl0InterruptEnable
#define BOARD_SECOND_CHANNEL_INT kTPM_Chnl1InterruptEnable
#define BOARD_TIMER_PRESCALE_DIVIDER kTPM_Prescale_Divide_128
/* Get source clock for TPM driver */
#define BOARD_TIMER_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_McgIrc48MClk)
#define TIMER_CLOCK_MODE 1U
#define BOARD_TPM_IRQ_HANDLER_FUNC TPM2_IRQHandler
#define BOARD_TPM_IRQ_ID TPM2_IRQn
/* I2C source clock */
#define I2C_BAUDRATE 100000U

#define I2C_RELEASE_SDA_PORT PORTD
#define I2C_RELEASE_SCL_PORT PORTD
#define I2C_RELEASE_SDA_GPIO GPIOD
#define I2C_RELEASE_SDA_PIN 6U
#define I2C_RELEASE_SCL_GPIO GPIOD
#define I2C_RELEASE_SCL_PIN 7U
#define I2C_RELEASE_BUS_COUNT 100U

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void BOARD_I2C_ReleaseBus(void);

static void Board_UpdatePwm(uint16_t x, uint16_t y);
/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile int16_t xAngle = 0;
volatile int16_t yAngle = 0;
volatile int16_t xAngleNoAbs = 0;
volatile int16_t yAngleNoAbs = 0;
volatile int16_t zAngle = 0;

/* MMA8451 device address */
const uint8_t g_accel_address[] = {0x1CU, 0x1DU, 0x1EU, 0x1FU};

/*******************************************************************************
 * Code
 ******************************************************************************/

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

void BOARD_I2C_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortD);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (i = 0; i < 9; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}
/* Initialize timer module */
static void Timer_Init(void)
{
    /* convert to match type of data */
    tpm_config_t tpmInfo;
    tpm_chnl_pwm_signal_param_t tpmParam[2];

    /* Configure tpm params with frequency 24kHZ */
    tpmParam[0].chnlNumber = (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL;
    tpmParam[0].level = kTPM_LowTrue;
    tpmParam[0].dutyCyclePercent = 0U;

    tpmParam[1].chnlNumber = (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL;
    tpmParam[1].level = kTPM_LowTrue;
    tpmParam[1].dutyCyclePercent = 0U;

    /* Initialize TPM module */
    /*
     * tpmInfo.prescale = kTPM_Prescale_Divide_128;
     * tpmInfo.useGlobalTimeBase = false;
     * tpmInfo.enableDoze = false;
     * tpmInfo.enableDebugMode = false;
     * tpmInfo.enableReloadOnTrigger = false;
     * tpmInfo.enableStopOnOverflow = false;
     * tpmInfo.enableStartOnTrigger = false;
     * tpmInfo.enablePauseOnTrigger = false;
     * tpmInfo.triggerSelect = kTPM_Trigger_Select_0;
     * tpmInfo.triggerSource = kTPM_TriggerSource_External;
     */
    TPM_GetDefaultConfig(&tpmInfo);
    TPM_Init(BOARD_TIMER_BASEADDR, &tpmInfo);

    CLOCK_SetTpmClock(TIMER_CLOCK_MODE);

    TPM_SetupPwm(BOARD_TIMER_BASEADDR, tpmParam, 2U, kTPM_EdgeAlignedPwm, 24000U, BOARD_TIMER_SOURCE_CLOCK);
    TPM_StartTimer(BOARD_TIMER_BASEADDR, kTPM_SystemClock);
}

/* Update the duty cycle of an active pwm signal */
static void Board_UpdatePwm(uint16_t x, uint16_t y)
{
    /* Updated duty cycle */
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_FIRST_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, x);
    TPM_UpdatePwmDutycycle(BOARD_TIMER_BASEADDR, (tpm_chnl_t)BOARD_SECOND_TIMER_CHANNEL, kTPM_EdgeAlignedPwm, y);
}

void delay(long int i)
{
	i = i*4800;
	while (i)
		i--;
}

int main(void)
{
    mma_handle_t mmaHandle = {0};
    mma_data_t sensorData = {0};
    mma_config_t config = {0}; 
    uint8_t sensorRange = 0;
    uint8_t dataScale = 0;
    int16_t xData = 0;
    int16_t yData = 0;
    int16_t zData = 0;
    long int xDataDouble = 0;
    long int yDataDouble = 0;

    long int rychlosty = 0;
    long int rychlostz = 0;
    long int rychlostx = 0;
    uint8_t i = 0;
    uint8_t array_addr_size = 0;
    status_t result = kStatus_Fail;

    /* Board pin, clock, debug console init */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();

    /* I2C initialize */
    BOARD_Accel_I2C_Init();
    /* Configure the I2C function */
    config.I2C_SendFunc = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;

    /* Initialize sensor devices */
    array_addr_size = sizeof(g_accel_address) / sizeof(g_accel_address[0]);
    for (i = 0; i < array_addr_size; i++)
    {
        config.slaveAddress = g_accel_address[i];
        /* Initialize accelerometer sensor */
        result = MMA_Init(&mmaHandle, &config);
        if (result == kStatus_Success)
        {
            break;
        }
    }

    if (result != kStatus_Success)
    {
        PRINTF("\r\nSensor device initialize failed!\r\n");
        return -1;
    }
    /* Get sensor range */
    if (MMA_ReadReg(&mmaHandle, kMMA8451_XYZ_DATA_CFG, &sensorRange) != kStatus_Success)
    {
        return -1;
    }
    if (sensorRange == 0x00)
    {
        dataScale = 2U;
    }
    else if (sensorRange == 0x01)
    {
        dataScale = 4U;
    }
    else if (sensorRange == 0x10)
    {
        dataScale = 8U;
    }
    else
    {
    }
    
    /* Init timer */
    Timer_Init();


    PRINTF("\r\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");


    char a = 0;
    zacatek:
    PRINTF("\r zvolte možnost stiskem klávesy: \n\r\t1: signalizace volného pádu \n\r\t2: detekce úhlu náklonu \n\r\t3: krokoměr \n\r\t4,5: odhad okamžité rychlosti");

    a = GETCHAR();


    PRINTF("\r\n zvolili jste %c\n",a);
    if (a==49)
    {
    	PRINTF("\r\n tedy: signalizace volného pádu\n\n");
    } else if (a==50)
    {
    	PRINTF("\r\n tedy: detekce úhlu náklonu\n\n");
    } else if (a==51)
    {
       	PRINTF("\r\n tedy: krokoměr\n\n");
    } else if (a==52 || a==53)
    {
    	PRINTF("\r\n tedy: odhad okamžité rychlosti\n\n");
    } else
    {
    	PRINTF("\r\n zvolili jste špatně!\n\n");
       	goto zacatek;
    }

    long int k=-1;
    /* Main loop. Get sensor data and update duty cycle */
    while (1)
    {
    	/* Get new accelerometer data. */
        if (MMA_ReadSensorData(&mmaHandle, &sensorData) != kStatus_Success)
        {
            return -1;
        }

        /* Get the X and Y data from the sensor data structure in 14 bit left format data*/
        xData = (int16_t)((uint16_t)((uint16_t)sensorData.accelXMSB << 8) | (uint16_t)sensorData.accelXLSB) / 4U;
        yData = (int16_t)((uint16_t)((uint16_t)sensorData.accelYMSB << 8) | (uint16_t)sensorData.accelYLSB) / 4U;
        zData = (int16_t)((uint16_t)((uint16_t)sensorData.accelZMSB << 8) | (uint16_t)sensorData.accelZLSB) / 4U;

        /* Convert raw data to angle (normalize to 0-90 degrees). No negative angles. */
        xAngle = (int16_t)floor((double)xData * (double)dataScale * 90 / 8192);
        xAngleNoAbs = xAngle;
        xAngle = abs(xAngle);

        yAngle = (int16_t)floor((double)yData * (double)dataScale * 90 / 8192);
        yAngleNoAbs = yAngle;
        yAngle = abs(yAngle);


        zAngle = (int16_t)floor((double)zData * (double)dataScale * 90 / 8192);

        /* Update the duty cycle of PWM */
        Board_UpdatePwm(xAngle, yAngle);


        if (a==51) {		//KROKOMĚR
        	if (  sqrt( (zAngle*zAngle) + (yAngle*yAngle) + (xAngle*xAngle))>120 )
        	{
        		k++;
        		delay(200);
        		PRINTF("\rx= %4d° y = %4d° a z = %4d° kroky = %d                                        \r", xAngle, yAngle, zAngle, k);

        	}
        }

        /*když jsou uhly(zrychlení) kolem 0 a zároveň byl zadán test VOLNÉHO PÁDU*/
        if ((abs(zAngle) + yAngle + xAngle)<10 && a==49 && abs(zAngle)<25 && yAngle<25 && xAngle<25)
        {
        	PRINTF("x= %4d° y = %4d° a z = %4d° !!VOLNY PAD!!                                        \n\r", xAngle, yAngle, zAngle);
        }

        else if(a==50) //DETEKCE ÚHLU
        {
        	/* Print out the angle data. */
            PRINTF("x= %4d° y = %4d° a z = %4d°                                      \r", xAngle, yAngle, zAngle);

        }

        else if (a==52) //ODHAD RYCHLOSTI
        {

        	xDataDouble = xAngleNoAbs;
            yDataDouble = yAngleNoAbs;

            if (xAngle > 5 || yAngle > 5)
            {
            	rychlostx = xDataDouble + rychlostx;
                rychlosty = yDataDouble + rychlosty;
                rychlostz = zAngle-90 + rychlostz;
            }


            PRINTF("\r x= %5d   y=  %5d  z=  %5d   vector= %5d\n", rychlostx, rychlosty, rychlostz, ((int) sqrt((rychlostx*rychlostx)+(rychlosty*rychlosty)+((rychlostz)*(rychlostz)))));
        }

        else if (a==53) //ODHAD RYCHLOSTI
        {

        	xDataDouble = xAngleNoAbs;
            yDataDouble = yAngleNoAbs;

            if (xAngle > 5 || yAngle > 5)
            {
            	rychlostx = xDataDouble + rychlostx;
                rychlosty = yDataDouble + rychlosty;
            }


            	PRINTF("\r x= %5d   y=  %5d  vector= %5d\n", rychlostx, rychlosty, ((int) sqrt((rychlostx*rychlostx)+(rychlosty*rychlosty))));
         }
    }
}
