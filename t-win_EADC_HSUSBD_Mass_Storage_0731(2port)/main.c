/**************************************************************************//**
* @file     main.c
* @version  V1.00
* $Date: 16/08/02 5:11p $
* @brief    Use internal SRAM as back end storage media to simulate a
*           30 KB USB pen drive
*
* @note
* Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "massstorage.h"
#include "vcom_serial.h"
#include "M480.h"                      

#define PLL_CLOCK       192000000
#define TRANS_NUM 6//6
#define CHAN_NUM 16
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;
volatile uint32_t g_u32IsTestOver = 0;
int16_t  g_i32ConversionData[TRANS_NUM*CHAN_NUM] __attribute__((at(0x20002500)));  //2500
uint32_t g_u32SampleModuleNum = 0;
uint32_t conversionAddress  ; 
extern uint8_t volatile g_u8MscStart;
extern int volatile formated;
extern int8_t volatile readflag;

///////////////////////////////////////////////////////////////////////////
/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING gLineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t gCtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

#define TX_FIFO_SIZE        16  /* TX Hardware FIFO size */


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
volatile uint8_t comRbuf[RXBUFSIZE];
volatile uint16_t comRbytes = 0;
volatile uint16_t comRhead = 0;
volatile uint16_t comRtail = 0;

volatile uint8_t comTbuf[TXBUFSIZE];
volatile uint16_t comTbytes = 0;
volatile uint16_t comThead = 0;
volatile uint16_t comTtail = 0;

uint8_t gRxBuf[64] = {0};
uint8_t *gpu8RxBuf = 0;
uint32_t gu32RxSize = 0;
uint32_t gu32TxSize = 0;

volatile int8_t gi8BulkOutReady = 0;
///////////////////////////////////////////////////////////////////////////

void ADC00_IRQHandler(void)
{
	g_u32AdcIntFlag = 1;
	EADC_CLR_INT_FLAG(EADC, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
}

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
	uint32_t volatile i;

	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Set XT1_OUT(PF.2) and XT1_IN(PF.3) to input mode */
	PF->MODE &= ~(GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);

	/* Enable External XTAL (4~24 MHz) */
	CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

	/* Waiting for 12MHz clock ready */
	CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

	/* Switch HCLK clock source to HXT */
	CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

	/* Set core clock as PLL_CLOCK from PLL */
	CLK_SetCoreClock(FREQ_192MHZ);

	/* Set both PCLK0 and PCLK1 as HCLK/2 */
	CLK->PCLKDIV = CLK_PCLKDIV_PCLK0DIV2 | CLK_PCLKDIV_PCLK1DIV2;

	SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;    /* select HSUSBD */
	/* Enable USB PHY */
	SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
	for (i=0; i<0x1000; i++);      // delay > 10 us
	SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

	/* Enable IP clock */
	CLK_EnableModuleClock(HSUSBD_MODULE);

	/* Select IP clock source */
	CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HXT, CLK_CLKDIV0_UART0(1));

	/* Enable IP clock */
	CLK_EnableModuleClock(UART0_MODULE);

	/*Timer*/
	CLK_EnableModuleClock(TMR0_MODULE);
	CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, 0);

	/* Set GPB multi-function pins for UART0 RXD and TXD */
	SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
	SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);


	/******* this part for PDMA *****/

	/* Enable EPWM0 module clock */
	CLK_EnableModuleClock(EPWM0_MODULE);

	/* Select EPWM0 module clock source as PCLK0 */
	CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

	/* Enable EADC module clock */
	CLK_EnableModuleClock(EADC_MODULE);

	/* EADC clock source is 96MHz, set divider to 8, EADC clock is 96/8 MHz */
	CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV0_EADC(8));

	/* Enable PDMA clock source */
	CLK_EnableModuleClock(PDMA_MODULE);

	/* Set PB.0 ~ PB.3 to input mode */
	PB->MODE &= ~(GPIO_MODE_MODE0_Msk | GPIO_MODE_MODE1_Msk | GPIO_MODE_MODE2_Msk | GPIO_MODE_MODE3_Msk);
	/* Configure the GPB0 - GPB3 ADC analog input pins.  */
	SYS->GPB_MFPL &= ~(SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk |
		SYS_GPB_MFPL_PB2MFP_Msk | SYS_GPB_MFPL_PB3MFP_Msk);
	SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_EADC0_CH0 | SYS_GPB_MFPL_PB1MFP_EADC0_CH1 |
		SYS_GPB_MFPL_PB2MFP_EADC0_CH2 | SYS_GPB_MFPL_PB3MFP_EADC0_CH3);

	/* Disable the GPB0 - GPB3 digital input path to avoid the leakage current. */
	GPIO_DISABLE_DIGITAL_PATH(PB, BIT3|BIT2|BIT1|BIT0);
//USBD
/* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Select IP clock source */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_USBDIV_Msk) | CLK_CLKDIV0_USB(4);

    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Set PA.12 ~ PA.14 to input mode */
    PA->MODE &= ~(GPIO_MODE_MODE12_Msk | GPIO_MODE_MODE13_Msk | GPIO_MODE_MODE14_Msk);
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk|SYS_GPA_MFPH_PA13MFP_Msk|SYS_GPA_MFPH_PA14MFP_Msk|SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS|SYS_GPA_MFPH_PA13MFP_USB_D_N|SYS_GPA_MFPH_PA14MFP_USB_D_P|SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
    SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
}

void EPWM0_Init()
{

	/* Set EPWM0 timer clock prescaler */
	EPWM_SET_PRESCALER(EPWM0, 0, 0);

	/* Set up counter type */
	EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE0_Msk;

	/* Set EPWM0 timer duty */
	EPWM_SET_CMR(EPWM0, 0, 108);

	/* Set EPWM0 timer period */
	EPWM_SET_CNR(EPWM0, 0, 216);

	/* EPWM period point trigger ADC enable */
	EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

	/* Set output level at zero, compare up, period(center) and compare down of specified channel */
	EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);

	/* Enable output of EPWM0 channel 0 */
	EPWM_EnableOutput(EPWM0, BIT0);
}

void PDMA_Init()
{

	/* Configure PDMA peripheral mode form EADC to memory */
	/* Open Channel 2 */
	PDMA_Open(PDMA,BIT2);

	/* transfer width is half word(16 bit) and transfer count is 6 */
	PDMA_SetTransferCnt(PDMA,2, PDMA_WIDTH_16, TRANS_NUM);

	/* Set source address as EADC data register(no increment) and destination address as g_i32ConversionData array(increment) */
	conversionAddress = (uint32_t) g_i32ConversionData;
	PDMA_SetTransferAddr(PDMA,2, (uint32_t)&EADC->DAT[g_u32SampleModuleNum], PDMA_SAR_FIX, conversionAddress, PDMA_DAR_INC);

	/* Select PDMA request source as ADC RX */
	PDMA_SetTransferMode(PDMA,2, PDMA_ADC_RX, FALSE, 0);

	/* Set PDMA as single request type for EADC */
	PDMA_SetBurstType(PDMA,2, PDMA_REQ_SINGLE, PDMA_BURST_4);

	PDMA_EnableInt(PDMA,2, PDMA_INT_TRANS_DONE);
	NVIC_EnableIRQ(PDMA_IRQn);

}

void ReloadPDMA()
{
	/* transfer width is half word(16 bit) and transfer count is 6 */
	PDMA_SetTransferCnt(PDMA,2, PDMA_WIDTH_16, TRANS_NUM);

	/* Select PDMA request source as ADC RX */
	PDMA_SetTransferMode(PDMA,2, PDMA_ADC_RX, FALSE, 0);

	conversionAddress += TRANS_NUM * 2;
	/* Set source address as EADC data register(no increment) and destination address as g_i32ConversionData array(increment) */
	PDMA_SetTransferAddr(PDMA,2, (uint32_t)&EADC->DAT[g_u32SampleModuleNum], PDMA_SAR_FIX, conversionAddress, PDMA_DAR_INC);
}

void EADC_FunctionTest()
{
	uint8_t i;
	
	/******* This part is copied from EADC_PDMA_EPWM_Trigger   **********************************************************/

	for(i = 0; i < CHAN_NUM; i++) {

		ReloadPDMA();

		/* Configure the sample module 0 for analog input channel 2 and enable EPWM0 trigger source */
		EADC_ConfigSampleModule(EADC, g_u32SampleModuleNum, EADC_PWM0TG0_TRIGGER, i);
		EADC_ENABLE_PDMA(EADC);

		/* Enable EPWM0 channel 0 counter */
		EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

		while(1)
		{
			/* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
			while(g_u32IsTestOver == 0);
			break;
		}
		g_u32IsTestOver = 0;

		/* Disable EPWM0 channel 0 counter */
		EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

	}
	/************************************************************************************/
}

void delay(int count)//1ms
{
	int i,j;
	for(i=0;i<count;i++)
	{
		for(j=0;j<30000;j++)
			__NOP();
	}
}

void USB_ufi()
{
	int f=0;
	while(1)
	{
		if(g_u8MscStart)
		{
			MSC_ProcessCmd();
			if(formated>=6)
				f++;
			if(f>=120)
			{
				break;
			}
		}
	}
}

void PDMA_IRQHandler(void)
{
	uint32_t status = PDMA_GET_INT_STATUS(PDMA);

	if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
	{
		if(PDMA_GET_ABORT_STS(PDMA) & PDMA_ABTSTS_ABTIF2_Msk)
			g_u32IsTestOver = 2;
		PDMA_CLR_ABORT_FLAG(PDMA,PDMA_ABTSTS_ABTIF2_Msk);
	}
	else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
	{
		if(PDMA_GET_TD_STS(PDMA) & PDMA_TDSTS_TDIF2_Msk)
			g_u32IsTestOver = 1;
		PDMA_CLR_TD_FLAG(PDMA,PDMA_TDSTS_TDIF2_Msk);
	}
	else
		printf("unknown interrupt !!\n");
}

int time=0;
int t1=0,t2=0;
void TMR0_IRQHandler(void)
{
	time++;
	TIMER_ClearIntFlag(TIMER0);
}

void VCOM_TransferData(void)
{
    int32_t i, i32Len;

    /* Check whether USB is ready for next packet or not*/
    if(gu32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(comRbytes)
        {
            i32Len = comRbytes;
            if(i32Len > EP2_MAX_PKT_SIZE)
                i32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < i32Len; i++)
            {
                gRxBuf[i] = comRbuf[comRhead++];
                if(comRhead >= RXBUFSIZE)
                    comRhead = 0;
            }

            __set_PRIMASK(1);
            comRbytes -= i32Len;
            __set_PRIMASK(0);

            gu32TxSize = i32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)gRxBuf, i32Len);
            USBD_SET_PAYLOAD_LEN(EP2, i32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            i32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(i32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(gi8BulkOutReady && (gu32RxSize <= TXBUFSIZE - comTbytes))
    {
        for(i = 0; i < gu32RxSize; i++)
        {
            comTbuf[comTtail++] = gpu8RxBuf[i];
            if(comTtail >= TXBUFSIZE)
                comTtail = 0;
        }

        __set_PRIMASK(1);
        comTbytes += gu32RxSize;
        __set_PRIMASK(0);

        gu32RxSize = 0;
        gi8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(comTbytes)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART_WRITE(UART0, comTbuf[comThead++]);
            if(comThead >= TXBUFSIZE)
                comThead = 0;

            __set_PRIMASK(1);
            comTbytes--;
            __set_PRIMASK(0);

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

void UART0_IRQHandler(void)
{
    uint8_t bInChar;
    int32_t size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAINT_Msk) || (u32IntStatus & UART_INTSTS_RXTOINT_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while( (!UART_GET_RX_EMPTY(UART0)) )
        {
            /* Get the character from UART Buffer */
            bInChar = UART_READ(UART0);    /* Rx trigger level is 1 byte*/

            /* Check if buffer full */
            if(comRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                comRbuf[comRtail++] = bInChar;
                if(comRtail >= RXBUFSIZE)
                    comRtail = 0;
                comRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREINT_Msk)
    {

        if(comTbytes)
        {
            /* Fill the Tx FIFO */
            size = comTbytes;
            if(size >= TX_FIFO_SIZE)
            {
                size = TX_FIFO_SIZE;
            }

            while(size)
            {
                bInChar = comTbuf[comThead++];
                UART_WRITE(UART0, bInChar);
                if(comThead >= TXBUFSIZE)
                    comThead = 0;
                comTbytes--;
                size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

int32_t main (void)
{
	SYS_Init();
    UART_Open(UART0, 115200);
	    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
	
    printf("NuMicro USB CDC One Port\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    USBD_Start();

    NVIC_EnableIRQ(UART0_IRQn);

    NVIC_EnableIRQ(USBD_IRQn);

    while(1)
    {
        VCOM_TransferData();
				if(comTbuf[20]=='U')
					break;
    }
	SYS_Init();
	/* Lock protected registers */
	SYS_LockReg();
	/* Init UART to 115200-8n1 for print message */
	UART_Open(UART0, 115200);
	/* Init EPWM for EADC */
	EPWM0_Init();
	/* Init PDMA for EADC */
	PDMA_Init();	

	printf("M480 HSUSB Mass Storage\n");

	/*initial HSUSBD*/
	HSUSBD_Open(&gsHSInfo, MSC_ClassRequest, NULL);		
	/* Endpoint configuration */
	MSC_Init();

	// Set timer frequency to 1HZ
	TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 1000000);
	// Enable timer interrupt
	TIMER_EnableInt(TIMER0);
	NVIC_EnableIRQ(TMR0_IRQn);
	// Start Timer 0
	TIMER_Start(TIMER0);

	/* Enable USBD interrupt */
	NVIC_EnableIRQ(USBD20_IRQn);
	EADC_Open(EADC, EADC_CTL_DIFFEN_SINGLE_END);    // Set input mode as single-end and enable the A/D converter
	EADC_SetTriggerDelayTime(EADC,g_u32SampleModuleNum,0x00,EADC_SCTL_TRGDLYDIV_DIVIDER_1);  //Set EADC sampling rate
	
	while(1) 
	{
		if(HSUSBD_IS_ATTACHED())
		{
			HSUSBD_Start();
			break;
		}
	}

	USB_ufi();  //wait for formated
	
	while(1)
	{
		EADC_FunctionTest();
		conversionAddress=0x20002500;  //back to the original address
		
		readflag=0;
		while(!readflag)  //wait pc read
		{
			if(g_u8MscStart)
				MSC_ProcessCmd();
		}
	}
	
	EADC_Close(EADC);
	/* Disable EADC IP clock */
	CLK_DisableModuleClock(EADC_MODULE);

	/* Disable EPWM0 IP clock */
	CLK_DisableModuleClock(EPWM0_MODULE);

	/* Disable PDMA clock source */
	CLK_DisableModuleClock(PDMA_MODULE);

	/* Disable PDMA Interrupt */
	NVIC_DisableIRQ(PDMA_IRQn);

	while(1);

}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
