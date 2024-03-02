 /*******************************************************************************
  *Main Source File
  *
  *Company:
  *  Microchip Technology Inc.
  *
  *File Name:
  *  main.c
  *
  *Summary:
  *  This file contains the "main" function for a project.
                              *
  *Description:
  * This file contains the "main" function for a project.  The
  *  "main" function calls the "   SYS_Initialize" function to initialize the state
  *  machines of all modules in the system
  ******************************************************************************/
/*****************************************************************************/
/******************************************************************************
 * History : 17-10-2023  Ver.60
 *          
 *****************************************************************************/

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stddef.h>                     // Defines NULL
#include <stdbool.h>                    // Defines true
#include <stdlib.h>                     // Defines EXIT_FAILURE
#include "definitions.h"                // SYS function prototypes
#include <string.h>

//#include "plib_sercom0_usart.h"

/******************************************************************************
 * Macro definitions
 *****************************************************************************/
#define Range_select          0x14                                              /* Range Select Register Address for External ADC */
#define switch_on             1                                                 /* active level of switch */
#define switch_off            0                                                 /* inactive level of Switch */
#define SLIP_END              0xC0                                              /* indicates start or end of packet */
#define SLIP_ESC              0xDB                                              /* indicates byte stuffing */
#define SLIP_ESC_END          0xDC                                              /* ESC ESC_END means END data byte */
#define SLIP_ESC_ESC          0xDD                                              /* ESC ESC_ESC means ESC data byte */
#define ACK                   0x00                                              /* indicates ACK Byte*/
#define NACK                  0x80                                              /* indicates NACK Byte*/
#define Status                0x01                    
#define status_byte1          0x00
#define status_byte2          0x00
#define Firmware_version      35                                                /*indicates firmware version*/
#define Hardware_version      0x10                                              /*indicates hardware version*/
#define CALIBRATION_SAMPLES   20                                                /*Total no of calibration data*/
#define TOTAL_SAMPLES         200                                               /*Total no of samples*/
#define SELFTEST_TOLERANCE    10                                                 /*selftest current tolerance value (+- 10mA) */
#define SELFTEST_CURRENT      46                                                /*self test compare current*/
#define ZERO_CALIBRATION_SAMPLES 2                                              /*Zero calibration samples */                                          
#define RMS_AVG               5
#define TypeB7_compensate      4
/******************************************************************************
 * Private global variables and functions
 *****************************************************************************/

static volatile bool isTimerExpired = false;
static volatile bool isTransferDone = false;
static volatile bool readStatus=false;
static volatile bool writeStatus=false;
static volatile bool errorStatus=false;
static volatile bool usart_read_status;
static volatile bool TypeB8_F=false;
uint8_t txData[4]={0};                                                          /*!< SPI Transmit data */
uint8_t rxData[2]={0};                                                          /*!< SPI Receive data  */
uint8_t uartTxData[10]={};                                                      /*!< UART Transmit data */
uint8_t uartRxData[11]={};                                                      /*!< UART Receive data */
uint8_t RCM_Address;                                                            /*!< Store RCM address */
uint8_t command_type;                                                           /*!< Store command type */
uint8_t calibration_flag;                                                       /*!< Calibration data packet status */
uint16_t Current_Reading[TOTAL_SAMPLES]={0};                                    /*!< Store the ADC data */
uint16_t AC_Rms_voltage1;                                                       /*!< AC RMS voltage */
uint16_t DC_Rms_voltage1;                                                       /*!< DC RMS voltage */
uint16_t N_DC_Rms_voltage1;                                                     /*!< Negative DC RMS coltage */
uint16_t ac_count;                                                              /*!< AC calibration sample number */
uint16_t dc_count;                                                              /*!< DC calibration sample number */
uint16_t zero_current, pdc, ndc;                                                /*!< Zero current, positive DC, Negative DC samplu number */
uint16_t Register_combination;                                                  /*!< Register combination address */
int16_t Rms_Data;                                                               /*!< RMS data */  
int16_t Rms_Dataa;                                                               /*!< RMS data */  
int16_t Rms_DataAvg;
int previous_ac;                                                                /*!<  Store previous  AC calibration data */
int previous_dc;                                                                /*!<  Store previous  DC calibration data */
uint32_t nBytesRead;                                                            /*!<  No. of bytes read */
uint8_t selftest_ACflag = 0;                                                    /*!<  Self test AC current flag */
float AC_voltage,DC_voltage,AC_cal_value,DC_cal_value,AC_calibrated_value,DC_calibrated_value;  /*!< To calculate RMS voltage */
uint8_t counter =0;                                                             /*!<  Counter */
uint16_t counters =0;                                                           /*!<  Counters */

/*********************************************************/
static volatile bool SelfTestSTF1 = false;
static volatile bool EICheckSTF = false;
static volatile bool freqcountenable = false;
//static volatile bool FFFFF = false;
uint16_t EICheckState=0;
uint32_t SysTimecount=0;
uint16_t FrzCheckCount1=0;
uint16_t FrzCheckCount2=0;
uint16_t FrzCheckCount1Store = 4000;                //corresponds to a 10Hz signal
uint16_t FrzCheckCount2Store = 4000;
uint16_t FrzCheckDCount=0;
uint16_t FrzPreiod=0;
uint16_t FrzPreiod1=0;
uint16_t FrzPreiod2=0;
uint16_t TimerBaseCode=4400;
uint16_t TimerBaseCodeStore=0;
uint32_t TimerBaseCodeTemp=0;
uint16_t aa[RMS_AVG];
uint16_t countaa;
/*uint16_t bb[10];

uint16_t countbb;*/
int16_t fit_Rms(int16_t * Rvalue,uint16_t Preiod);
uint16_t fit_Preiod(uint16_t m, uint16_t n);
void fit_Preiod1(uint16_t m, uint16_t n);
void EIC_User_Handler(uintptr_t context);   //????????
#define _calibration_table                                                      /*!<Define predetermined calibration table */
#ifdef _calibration_table
uint32_t offset= 0x7f4c;                                                         /*!< predetermined offset value */                                                                             /*!< predetermined calibration array */
uint16_t Adc_calibration[CALIBRATION_SAMPLES][4]={   
{32102,32702,19148,19650},
{32204,32794,21708,22219},
{32307,32886,23808,24344},
{32409,33003,25856,26410},
{32512,33097,27904,28444},
{32768,33322,28979,29520},
{33024,33590,29951,30490},
{33280,33821,30976,31532},
{33536,34076,31488,32057},
{34048,34581,31744,32282},
{34560,35063,32255,32805},
{35072,35541,32512,33048},
{36096,36525,33024,33574},
{37120,37540,34048,34585},
{38144,38538,35072,35632},
{39168,39548,36096,36649},
{40192,40529,37120,37677},
{41216,41538,39168,39707},
{42240,42595,41216,41784},
{44800,45090,44851,45457}
};
#endif

#ifndef _calibration_table
uint32_t offset;                                                                /*!< To measure offset value */
uint16_t Adc_calibration[CALIBRATION_SAMPLES][4];                               /*!< To store calibration value */
#endif

/*!
 *  Define a constant array in Flash.
 *  It must be aligned to row boundary and size has to be in multiple of rows
 */
const uint8_t nvm_rwwee_user_start_address[NVMCTRL_RWWEEPROM_ROWSIZE] __attribute__((address(NVMCTRL_RWWEEPROM_START_ADDRESS)))= {0};
uint16_t data3 [NVMCTRL_RWWEEPROM_PAGESIZE] = {0};                          
uint16_t data4 [NVMCTRL_RWWEEPROM_PAGESIZE] = {0};
uint16_t data5 [NVMCTRL_RWWEEPROM_PAGESIZE] = {0};
int zerocross;                                                                  /*!< measure no of zero crossing */
uint32_t h=1,g=0,z=0;
uint32_t count,b,a;
size_t      txSize = 4;
size_t      txSize1 = 2;
size_t      rxSize = 2;
size_t      RX_Count;

/**
 * Function Name : RMS_calculation
 *
 * Description   : Do RMS calculation of ADC Samples without offset
 *
 * @param Current[] : Array of 300 samples
 * 
 * @return RMS_ADC_Value : RMS value
 */
int Rms_calculationACDC(int16_t AC,int16_t DC)
{
   int Rms_ADC_Value;                                                      /*!< RMS value */

    Rms_ADC_Value=AC*AC+DC*DC;
    Rms_ADC_Value=DIVAS_SquareRoot (Rms_ADC_Value);                                       /*!< Square root function */

    return Rms_ADC_Value;

}


/**
 * Function Name : RMS_calculation
 *
 * Description   : Do RMS calculation of ADC Samples without offset
 *
 * @param Current[] : Array of 300 samples
 * 
 * @return RMS_ADC_Value : RMS value
 */
uint16_t Rms_calculation(uint16_t current[])
{
   uint16_t Rms_ADC_Value;                                                      /*!< RMS value */
   uint32_t sum=0;                                                              /*!< Sum value */
 
    for(uint32_t l=0;l<TOTAL_SAMPLES;l++)
     {
        sum+=((current[l]-offset)*(current[l]-offset))/TOTAL_SAMPLES;           /*!< RMS calculation (I*I/N)*/
     }
   
    Rms_ADC_Value=DIVAS_SquareRoot (sum);                                       /*!< Square root function */
    return Rms_ADC_Value;

}


/**
 * Function Name : RMS_calculation1
 *
 * Description   : Do RMS calculation of ADC Samples with offset
 *
 * @param Current[] : Array of 300 samples
 * 
 * @return RMS_ADC_Value : RMS value
 */
uint16_t Rms_calculation1(uint16_t current[])
{
   uint16_t Rms_ADC_Value;                                                      /*!< RMS value */
   uint32_t sum=0;                                                              /*!< Sum value */
    
    for(uint32_t l=0;l<TOTAL_SAMPLES;l++)
    {
       sum+=((current[l])*(current[l]))/TOTAL_SAMPLES;                                    /*!< RMS Calculation(I*I/N)*/
    }
   
    Rms_ADC_Value=DIVAS_SquareRoot (sum);                                       /*!< square root function */
    
    return  Rms_ADC_Value;

}

/**
 * Function Name : usartWriteEventHandler
 *
 * Description   : called by USART PLIB when Transfer is completed
 *
 * @param event : Type of event occured
 * 
 * @param context : pointer of the buffer
 * 
 * @return None
 */
void usartWriteEventHandler(SERCOM_USART_EVENT event, uintptr_t context )
{
    if(event==SERCOM_USART_EVENT_WRITE_THRESHOLD_REACHED)
    { 
        writeStatus= true;
    }
}

void EIC_User_Handler(uintptr_t context)
{
    EICheckSTF=true;
    
    if(EICheckState==0)                                 //Idle State, not yet measuring frequency, ignoring startup glitches
    {
        if(freqcountenable)
        {
            EICheckState=1;                             //Start counting for period 1
            TC1_TimerStart();
        } 

    }
    else if(EICheckState==1)                            //Store result for period 1 and start counting for period 2
    {
        FrzCheckCount1Store = FrzCheckCount1;
        FrzCheckCount1 = 0;
        EICheckState=2;     
        //TC1_TimerStart(); 
    }
    
    else if(EICheckState==2)
    {
        FrzCheckCount2Store = FrzCheckCount2;           //Store result for period 2 and start counting for period 1
        FrzCheckCount2 = 0;        
        EICheckState=1;
        //TC1_TimerStart(); 
    }
}

void TC1_Callback_InterruptHandler(TC_TIMER_STATUS status, uintptr_t context)
{
   // 
    if((EICheckState==1))
    {
        FrzCheckCount1++;
        if (FrzCheckCount1 > 4000)                       //If an edge is not detected within 50ms, then freq is < 10Hz, treat as DC
        {
            FrzCheckCount1Store = FrzCheckCount1;
            FrzCheckCount1 = 0;
            EICheckState=2;  
        }
    }
    if((EICheckState==2))            //If an edge is not detected within 50ms, then freq is < 10Hz, treat as DC
    {
        FrzCheckCount2++; 
        if (FrzCheckCount2 > 4000)
        {
            FrzCheckCount2Store = FrzCheckCount2;
            FrzCheckCount2 = 0;
            EICheckState=1;  
        }
    }
}
//Max Min
int average(uint16_t a[], int n)
{
    uint16_t max=a[0];
    uint16_t min=a[0];
    uint16_t sum=0;
    for (int i=0;i<n;i++)
    {
        sum+=a[i];
        max=max<a[i]? a[i]:max;
        min=min>a[i]? a[i]:min;
    }
    return (sum-max-min)/(n-2);
    //return sum/(n);
}

void delay_100ms(void)
{
    while(counters<200)                                                                /*!< Wait for 10 ms*/
    {
        TC0_TimerStart();                                                       /*!< Enable the TC counter */
        while(!TC0_TimerPeriodHasExpired());                                    /*!< wait for timer(100us) to Expire*/
        TC0_TimerStop();                                                        /*!< Disable the TC counter */
        counters++;
    }
    counters=0;
}
/**
 * Function Name : delay_10ms
 *
 * Description   : Delay for 10ms
 *
 * @param None
 * 
 * @return none
 */
void delay_10ms(void)
{
    while(counter<10)                                                                /*!< Wait for 10 ms*/
    {
        TC0_TimerStart();                                                       /*!< Enable the TC counter */
        while(!TC0_TimerPeriodHasExpired());                                    /*!< wait for timer(100us) to Expire*/
        TC0_TimerStop();                                                        /*!< Disable the TC counter */
        counter++;
    }
    counter=0;
}

/**
 * Function Name : delay 400us
 *
 * Description   : Delay for 10ms
 *
 * @param None
 * 
 * @return none
 */
void delay_400us(void)
{
    //LED_CONTROL_Set();
    while(counter<4)                                                                /*!< Wait for 10 ms*/
    {
        TC0_TimerStart();                                                       /*!< Enable the TC counter */
        while(!TC0_TimerPeriodHasExpired());                                    /*!< wait for timer(100us) to Expire*/
        TC0_TimerStop();                                                        /*!< Disable the TC counter */
        //LED_CONTROL_Toggle();
        counter++;
    }
    //LED_CONTROL_Set();
    counter=0;
}

/**
 * Function Name : slip_transmission
 *
 * Description   : Transmitting SLIP command
 *
 * @param TxData : for the sign identification
 * 
 * @param size : No of bytes of data
 *
 * @return None
 */

void slip_transmission(uint8_t TxData[], int size)
{
    writeStatus = false;
    RS485_UART_RDE_Set();                                                       /*!< Enable Transmit Enable of RS485 */
    SERCOM0_USART_Write(&TxData[0],size);                                       /*!< USART write */
    UART_interrupt_enable();                                                    /*!< UART Interrupt Enable */
}

/**
 * Function Name : SPIEventHandler
 *
 * Description   : called by SPI PLIB when transfer is completed
 *
 * @param context : pointer of the buffer
 * 
 * @return None
 */
void SPIEventHandler(uintptr_t context )
{
    ADC_CS_Set();                                                               /*!< CS pin high */
    isTransferDone = true;
}

/**
 * Function Name : usartReadEventHandler
 *
 * Description   : called by USART PLIB when Read Event occur
 *
 * @param event : Type of spi event occured
 * 
 * @param context : pointer of the buffer
 * 
 * @return None
 */
void usartReadEventHandler(SERCOM_USART_EVENT event, uintptr_t context )
{
    uint32_t nBytesAvailable = 0;
    
    if (event == SERCOM_USART_EVENT_READ_THRESHOLD_REACHED)
    {
        nBytesAvailable = SERCOM0_USART_ReadCountGet();                         /*!< Receiver should have the thershold number of bytes in the receive buffer */
        
        nBytesRead += SERCOM0_USART_Read((uint8_t*)&rxData[nBytesRead], nBytesAvailable);                          
    }
    if(event == SERCOM_USART_EVENT_READ_BUFFER_FULL)
        UART_clearbuffer();
} 

/**
 * Function Name : crc8
 *
 * Description   : Do crc calculation of data
 *
 * @param inCrc : Calculated CRC
 * 
 * @param inData : new data to calculate CRC
 * 
 * @return crc result : CRC output
 */
uint8_t crc8( uint8_t inCrc, uint8_t inData )
{
uint8_t i2;
uint8_t crc;                                                                    /*!< store calculated CRC */
crc = inCrc ^ inData;
for ( i2 = 0; i2 < 8; i2++ )                                                    /*!< CRC calculation */
{
    if (( crc & 0x80 ) != 0 )
    {
        crc <<= 1;
        crc ^= 0x07;
     }
    else
          crc <<= 1;
}
return crc;
}

/**
 * Function Name : current_cal
 *
 * Description   : To get the total current from current samples
 *
 * @param current[] : Array of 300 samples
 * 
 * @param off_set : Offset value
 * 
 * @return RMS_current : Total RMS current
 */
int current_cal(uint16_t current[], uint32_t off_set)
{
    uint16_t min_val;                                                           /*!< To find minimum value from current samples */
    uint16_t max_val;                                                           /*!< To find maximum value from current samples */
    uint16_t average;                                                           /*!< To find average of current samples */
    //uint16_t median;                                                            /*!< To find median from current samples */
    uint16_t AC_Rms_voltage = 0;                                                /*!< AC Rms value from current samples */
    uint16_t DC_Rms_voltage = 0;                                                /*!< DC Rms value from current samples */
    int DC_current =0;                                                          /*!< Total DC current */
    int AC_current =0;                                                          /*!< Total AC current */                                       
    int Rms_current = 0;                                                        /*!< Total current */                                                              
    unsigned long sum2 =0;                                                      /*!< Sum of all current samples */                                                 
    uint16_t current_val[TOTAL_SAMPLES]= {0};                                   /*!< Temp array to store current sample */
    int16_t ACC_current;
    min_val = current[0];
    max_val = current[0];
    sum2 += current[0];
   // sorted_current_val[0]= current[0];         
    
    for(uint32_t i=1;i<TOTAL_SAMPLES;i++)                                       /*!< Finding min and max of current samples */
    {
        sum2 += current[i];
    }  
    average = sum2/TOTAL_SAMPLES;   											/*!< Average value */         
																				
    if((min_val < off_set) && (max_val > off_set))                              /*!< Only AC signal */
    {
        selftest_ACflag = 1;
	}
        
	if((average > (off_set - 1000)) && (average < (off_set + 1000)))
	{
		
		//printf("\n\n\rAC Only");
		AC_Rms_voltage=Rms_calculation(&current[0])+ off_set;                   /*!< AC RMS voltage calculation */
	   
		if((AC_Rms_voltage>=45800))
		  a=19;
		else
		{
			if(AC_Rms_voltage>=Adc_calibration[19][1])
				a=19;            
			else
			{
				for(uint32_t n=0;n<20;n++)
				{
				  if(AC_Rms_voltage<Adc_calibration[n][1])
					{
						a=n;
						break;
					}
				}
			}
		}
		//printf("\r\n AC Rms volatge is %d", AC_Rms_voltage);
		
		AC_calibrated_value=((AC_Rms_voltage - Adc_calibration[a-1][1])*(Adc_calibration[a][0]-Adc_calibration[a-1][0]));
		AC_cal_value=(AC_calibrated_value/(Adc_calibration[a][1]-Adc_calibration[a-1][1]))+Adc_calibration[a-1][0];
		AC_voltage=((AC_cal_value*5120)/65535)-2500;
		AC_current = (AC_voltage/4);
		ACC_current=(int16_t)AC_current;
		Rms_current =  fit_Rms(&ACC_current,FrzPreiod);                      /*!< Total RMS current */
	}
	else
	{
	   // printf("\n\n\rSelftest AC Current ");
		
	  DC_Rms_voltage = min_val + (average-min_val);
		
		//DC_Rms_voltage= 35714;//positive 50mA
		
		if(DC_Rms_voltage>=Adc_calibration[19][3])
			b=19;              
		else if(DC_Rms_voltage<Adc_calibration[0][3])
			b=1;                    
		else
		{
			for(uint32_t n=0;n<20;n++)
			{   
				if(DC_Rms_voltage<Adc_calibration[n][3])
				{
					b=n;
					break;
				}
			}
		}
		DC_calibrated_value=((DC_Rms_voltage-Adc_calibration[b-1][3])*(Adc_calibration[b][2]-Adc_calibration[b-1][2]));
		DC_cal_value=(DC_calibrated_value/(Adc_calibration[b][3]-Adc_calibration[b-1][3]))+Adc_calibration[b-1][2];
		DC_voltage=((DC_cal_value*5120)/65535)-2500;
		DC_current=(DC_voltage)/4;                                              /*!< DC Current */    
		for(uint32_t i=0;i<TOTAL_SAMPLES;i++)                                   /*!< AC / Half wave AC signal */ 
		{
			current_val[i] = current[i] - DC_Rms_voltage + off_set;
		}
		AC_Rms_voltage = Rms_calculation(&current_val[0]) + off_set;            /*!< AC rms value */
		if((AC_Rms_voltage>=45800))
		  a=19;
		else
		{
			if(AC_Rms_voltage>=Adc_calibration[19][1])
				a=19;            
			else
			{
				for(uint32_t n=0;n<20;n++)
				{
				  if(AC_Rms_voltage<Adc_calibration[n][1])
					{
						a=n;
						break;
					}
				}
			}
		}
		AC_calibrated_value=((AC_Rms_voltage - Adc_calibration[a-1][1])*(Adc_calibration[a][0]-Adc_calibration[a-1][0]));
		AC_cal_value=(AC_calibrated_value/(Adc_calibration[a][1]-Adc_calibration[a-1][1]))+Adc_calibration[a-1][0];
		AC_voltage=((AC_cal_value*5120)/65535)-2500;
		AC_current = (AC_voltage/4);                                            /*!< AC Rms current */
		ACC_current=(int16_t)AC_current;            
		Rms_current = Rms_calculationACDC(fit_Rms(&ACC_current,FrzPreiod),DC_current);
	}
    return Rms_current;                                                         /*!< Total RMS Current */
}

/**
 * Function Name : self_test
 *
 * Description   : Do self test and returns the status
 *
 * @param None
 * 
 * @return retval : Status of the self test
 */
uint8_t self_test(void)
{
    uint8_t retval;
    uint8_t AC_Flag;                                                            /*!< AC flag */    
    int compare_currentotal=0;
    AC_Flag = 0;
    selftest_ACflag = 0;
    int st_current = 0, res_current = 0, compare_current=0 ,res_currentotal=0;
    count=0;
    SelfTestSTF1=true;
    //SET_SelfTestSwitch_Set();//
   // delay_10ms();
    for(uint8_t c=0;c<5;c++)
    {
        fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
        TC0_Timer16bitPeriodSet(TimerBaseCode);
        
        for(uint32_t i=0;i<(TOTAL_SAMPLES+1);i++)                                   /*!< Getting 300 samples */
        {
            TC0_TimerStart();                                                        /*!< Enable the TC counter */
            txData[0]=0x44;                                                          /*!< Reading ADC Value*/
            txData[1]=0x00;
            ADC_CS_Clear();                                                          /*!< CS Pin Low */
            SERCOM1_SPI_WriteRead(txData, txSize1,rxData,rxSize);                    /*!< SPI Write Read */
            
            while(isTransferDone!=true);                                             /*!< wait for SPI write To complete*/

            if (isTransferDone==true)
            {
                isTransferDone=false;
                if(i>0)
                {
                    Current_Reading[count]=(rxData[0]<< 8) | rxData[1];               /*!< storing received data */
                    count++;
                }
                if(count == TOTAL_SAMPLES)                                                    /*!< reading 300 ADC samples and doing rms calculations*/
                {
                    //res_current = current_cal(&Current_Reading[0], offset);             /*!< RMS calculation */
                    count=0;
                }
            }
            while(!TC0_TimerPeriodHasExpired());                                     /*!< wait for timer(100us) to Expire*/
            TC0_TimerStop();                                                         /*!< Disable the TC counter */
        } 
        res_current = current_cal(&Current_Reading[0], offset);             /*!< RMS calculation */
        res_currentotal+=res_current;
    }
    AC_Flag = selftest_ACflag;
        
    SELF_TEST_Set();                                                            /*!< Self test pin high */
    delay_100ms();
    
    for(uint8_t c=0;c<5;c++)
    {    
                                                              /*!< Delay of 100ms to allow proper frequency measurement*/  
        fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
        TC0_Timer16bitPeriodSet(TimerBaseCode);

        for(uint32_t i=0;i<(TOTAL_SAMPLES+1);i++)                                   /*!< Getting 300 samples */
        {
            TC0_TimerStart();                                                        /*!< Enable the TC counter */
            txData[0]=0x44;                                                          /*!< Reading ADC Value*/
            txData[1]=0x00;
            ADC_CS_Clear();                                                          /*!< CS Pin Low */
            SERCOM1_SPI_WriteRead(txData, txSize1,rxData,rxSize);                    /*!< SPI Write Read */          

            while(isTransferDone!=true);                                             /*!< wait for SPI write To complete*/

            if (isTransferDone==true)
            {
                isTransferDone=false;
                if(i>0)
                {
                    Current_Reading[count]=(rxData[0]<< 8) | rxData[1];               /*!< storing received data */
                    count++;
                }
                if(count == TOTAL_SAMPLES)                                            /*!< reading 300 ADC samples and doing rms calculations*/
                {
                    //st_current = current_cal(&Current_Reading[0], offset);          /*!< RMS calculation */
                    count=0;
                    SelfTestSTF1=0;
                }
            }
            while(!TC0_TimerPeriodHasExpired());                                     /*!< wait for timer(100us) to Expire*/
            TC0_TimerStop();                                                         /*!< Disable the TC counter */
        } 
        st_current = current_cal(&Current_Reading[0], offset);                       /*!< RMS calculation */
        compare_currentotal+=st_current;
    }

    SELF_TEST_Clear();

    res_current=res_currentotal/5;
    st_current=compare_currentotal/5;
    if((AC_Flag == 1)&& (SELFTEST_CURRENT < 0))
        compare_current = st_current*st_current + res_current*res_current;                             /* Difference current */
    else
        compare_current = st_current*st_current - res_current*res_current;                             /* Difference current */
    compare_current=DIVAS_SquareRoot(compare_current);
    if((compare_current >= (SELFTEST_CURRENT - SELFTEST_TOLERANCE ) && (compare_current <= (SELFTEST_CURRENT + SELFTEST_TOLERANCE))))    /*!< Self test current comparison */     
        retval = 1;
    else
        retval = 2;
    printf("\n\rRes Current is : %d",res_current);
    printf("\n\rst Current is : %d",st_current);
    printf("\n\rSelf Test Current is : %d",compare_current);
    //SET_SelfTestSwitch_Clear();//???
    //printf("\n\rCompare Current is : %d",compare_current);
    return retval;
}

/**
 * Function Name : calibration
 *
 * Description   : Does calibration based on data input
 *
 * @param cur_type : Type of current calibration
 * 
 * @param cur_data1 : Data value 1
 * 
 * @param cur_data2 : Data value 2
 * 
 * @return retval1 : Status of the calibration test
 */
uint8_t calibration(uint8_t cur_type, uint8_t data1, uint8_t data2)
{
   
    uint8_t retval1;
    uint16_t Rms_voltage1;
    int current_data=0,sum1=0;
    uint32_t Numerator=65536;
    uint32_t e=1,f=0,g=0;
    float Denomenator,cal;
    float code; 
    count = 0;
    current_data = ((data1)<<8|data2);                                          /*!< current data value */
    if(data1 > 0x0F)                                                            /*!< Negative DC */
    {
        current_data=(~(current_data))&0xFFFF;                                  /*!< compliment */
        current_data = -1*current_data;                                         /*!< Negative DC current value */
    }
    
    if(cur_type == 0x00)                                                        /*!< 0 mA calibration */
    {
        for(uint32_t d=0;d<ZERO_CALIBRATION_SAMPLES;d++)                                              /*!< sampling 10 times */
        {
           fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
           TC0_Timer16bitPeriodSet(TimerBaseCode);
           for(uint32_t c=0;c<(TOTAL_SAMPLES+1);c++)                            /*!< Getting 300 samples */
            {
                TC0_TimerStart();
                txData[0]=0x00;
                txData[1]=0x00;
                ADC_CS_Clear();                                                 /*!< CS Pin low */
                SERCOM1_SPI_WriteRead(txData,txSize1, rxData, rxSize);          /*!< SPI Write Read */
               
                while(isTransferDone!=true);                                    /*!< check if SPI transfer is completed */

                if (isTransferDone==true)
                {
                    isTransferDone=false;
                    if(c>0)
                    {
                       Current_Reading[count]=(rxData[0]<< 8) | rxData[1];      /*!< storing received data */
                       count++;
                    }
                    if(count==TOTAL_SAMPLES)
                    {
                       //sum1=sum1+Rms_calculation1(&Current_Reading[0]);         /*!< RMS calculation */
                       count=0;
                    }
                }
                while(!TC0_TimerPeriodHasExpired());                            /*!< check if timer period interrupt flag is set */
                TC0_TimerStop();                                                /*!< Disable the TC counter */
            }
        }
        sum1=sum1+Rms_calculation1(&Current_Reading[0]);
        zero_current=1;
        offset=sum1/ZERO_CALIBRATION_SAMPLES;                                    /*!< divide by no of samples */
        ac_count=0;                                                             /*!< AC calibration table index to 0 */
        dc_count=0;                                                             /*!< DC calibration table index to 0 */
        pdc=0;                                                                  
        ndc=0;       
        previous_ac = 0;                                                        /*!< Less than min AC calibration current */
        previous_dc = -300;                                                     /*!< Less than min DC calibration current */
        calibration_flag = 1;
    }
    else if((cur_type == 0x01) && (ac_count<CALIBRATION_SAMPLES) && (zero_current==1) && (previous_ac<current_data))        /*!< AC Calibration*/
    {
        previous_ac = current_data;
        cal=((4*current_data)+2500);
        Denomenator=((float)5120/cal);//
        code=(float)Numerator/Denomenator;                                      /*!< Ideal ADC value of set current */
        Adc_calibration[ac_count][0]=code;
        
        fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
        TC0_Timer16bitPeriodSet(TimerBaseCode);
        
        for(uint32_t c=0;c<(TOTAL_SAMPLES+1);c++)                               /*!< Getting 300 samples */
        {
            TC0_TimerStart();
            txData[0]=0x00;
            txData[1]=0x00;
            ADC_CS_Clear();                                                      /*!< CS pin set low */
            SERCOM1_SPI_WriteRead(txData,txSize1, rxData, rxSize);               /*!< SPI Write Read */

            while(isTransferDone!=true);                                         /*!< wait for SPI write to complete*/    

            if (isTransferDone==true)
            {
               isTransferDone=false;
                if(c>0)
                {
                    Current_Reading[count]=(rxData[0]<< 8) | rxData[1];         /*!< storing received data */
                    count++;
                }
                if(count==TOTAL_SAMPLES)
                {
                    //Rms_voltage1=Rms_calculation(&Current_Reading[0]);          /*!< RMS calculation */
                    //Adc_calibration[ac_count][1]=Rms_voltage1+offset;           /*!< storing in ADC calibration table */
                    count=0;
                }
            }
           while(! TC0_TimerPeriodHasExpired());                                /*!< wait for Timer to Expire*/ 
           TC0_TimerStop();                                                     /*!< Disable the TC counter */
        }
        Rms_voltage1=Rms_calculation(&Current_Reading[0]);          /*!< RMS calculation */
        Adc_calibration[ac_count][1]=Rms_voltage1+offset;           /*!< storing in ADC calibration table */
        calibration_flag = 1;                                                   /*!< Set the flag for correct data */
        ac_count++;
    }
    else if((cur_type == 0x02) && ((pdc<(CALIBRATION_SAMPLES/2))||(ndc<(CALIBRATION_SAMPLES/2)))&&(zero_current==1)&&(previous_dc < current_data))       //!< DC Calibration
    {
        if(data1 > 0x0F)                                                        /*!< Negative DC*/
            ndc++;
        else
            pdc++;                                                              /*!< Positive DC*/
      
        previous_dc = current_data;
        cal=((4*current_data)+2500);
        Denomenator=((float)5120/cal);
        code=(float)Numerator/(float)Denomenator;                               /*!< Ideal ADC value of set current */
        Adc_calibration[dc_count][2]=code;
        
        fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
        TC0_Timer16bitPeriodSet(TimerBaseCode);
        
        for(uint32_t c=0;c<(TOTAL_SAMPLES+1);c++)                               /*!< Getting 300 samples */
        {
            TC0_TimerStart();                                                   /*!< Enable the TC counter */
            txData[0]=0x00;
            txData[1]=0x00;
            ADC_CS_Clear();                                                     /*!< CS pin LOW */
            SERCOM1_SPI_WriteRead(txData,txSize1, rxData, rxSize);              /*!< SPI Write Read */
            
            while(isTransferDone!=true);                                        /*!< wait for spi write to complete*/    

            if (isTransferDone==true)
            {
                isTransferDone=false;
                if(c>0)
                {
                    Current_Reading[count]=(rxData[0]<< 8) | rxData[1];         /*!< storing received data */
                    count++;
                }
                if(count==TOTAL_SAMPLES)
                {
                    //Rms_voltage1=Rms_calculation1(&Current_Reading[0]);         /*!< RMS calculation */
                    //Adc_calibration[dc_count][3]= Rms_voltage1;                 /*!< storing in adc calibration table */
                    count=0;
                }
            }
            while(! TC0_TimerPeriodHasExpired());                               /*!<wait for Timer to Expire*/ 
            TC0_TimerStop();                                                    /*!< Disable the TC counter */
        } 
        Rms_voltage1=Rms_calculation1(&Current_Reading[0]);                     /*!< RMS calculation */
        Adc_calibration[dc_count][3]= Rms_voltage1;                             /*!< storing in adc calibration table */
        calibration_flag = 1;                                                   /*!< Set the flag for correct data */
        dc_count++;
    }
    if(calibration_flag == 0)                                                   /*!< Wrong data*/
        retval1 =3;     
    else if(ac_count==CALIBRATION_SAMPLES && ndc==(CALIBRATION_SAMPLES/2) && pdc==(CALIBRATION_SAMPLES/2) && zero_current==1 )      /*!< check calibration table completely filled */
    {       
        *(data3+0)= offset;                                                     /*!< Storing data in a buffer*/
        for(uint32_t p=0;p<7;p++) 
        {
           for(uint32_t q=0;q<4;q++)
            {
               *(data3+e)=Adc_calibration[p][q];
               e++;
            }
        }
        for(uint32_t p=7;p<15;p++)                                              /*!< Storing Data in a buffer*/
        {
            for(uint32_t q=0;q<4;q++)
            {
                *(data4+f)=Adc_calibration[p][q];
                 f++;
            }
        }
        for(uint32_t p=15;p<21;p++)                                             /*!< Storing Data in a buffer*/
        {
            for(uint32_t q=0;q<4;q++)
            {
                *(data5+g)=Adc_calibration[p][q];
                 g++;
            }
        }
        while(NVMCTRL_IsBusy());
        NVMCTRL_RWWEEPROM_RowErase((uint32_t)nvm_rwwee_user_start_address);                                 /*!< Erase the row */
        while(NVMCTRL_IsBusy());                                                                            /*!< Wait for row erase  to complete */
        NVMCTRL_RWWEEPROM_PageWrite( (uint32_t *)data3,(uint32_t)nvm_rwwee_user_start_address  );           /*!< Program 64 byte page */
        while(NVMCTRL_IsBusy());                                                                            /*!< Wait for page program to compete */
        NVMCTRL_RWWEEPROM_PageWrite( (uint32_t *)data4,(uint32_t)(nvm_rwwee_user_start_address+0x40)  );    /*!< Program 64 byte page */ 
        while(NVMCTRL_IsBusy());                                                                            /*!< Wait for page program to compete */
        NVMCTRL_RWWEEPROM_PageWrite( (uint32_t *)data5,(uint32_t)(nvm_rwwee_user_start_address+0x80)  );    /*!< Program 64 byte page */
        while(NVMCTRL_IsBusy());  
        retval1 = 1;                                                             /*!< Calibration completed*/
    }
    else
    {
        retval1 = 2;                                                            /*!< Calibration in progress*/
    }
    calibration_flag = 0;
    return retval1;
}

/**
 * Function Name : main
 *
 * Description   : Initialization and main routine
 *
 * @param None
 * 
 * @return None
 */
int main ( void )
{
    uint8_t crc, crc_1, crc_2, crc_3, crc_4, crc_5, crc_6, crc_7, status_val;   /* CRC check */
    
    __enable_irq();
    count = 0;
    zero_current=0;                                                             
    calibration_flag =0;                                          
    SYS_Initialize ( NULL );                                                    /*!< Initialize all modules */
    RS485_UART_RDE_Clear();                                                     /*!< Disable Transmit Enable of RS485*/
    LED_CONTROL_Set();                                                          /*!< LED Off*/
    ALERT_CONTROL_OUT_Clear(); 
    ADC_CS_Set();                                                               /*!< CS pin high */
    SELF_TEST_Clear();                                                          /*!< self test pin low */
   // SET_SelfTestSwitch_Clear();
    //SET_SelfTestSwitch_Set();//????
    //SELF_TEST_Set();                                                          /*!< self test pin low */
    SERCOM1_SPI_CallbackRegister(SPIEventHandler, (uintptr_t) 0);               /*!< Register callback function   */
    SERCOM0_USART_ReadCallbackRegister(usartReadEventHandler, (uintptr_t)NULL);
    SERCOM0_USART_WriteCallbackRegister(usartWriteEventHandler, (uintptr_t)NULL);
    /* Register callback function for TC3 period interrupt */
    TC1_TimerCallbackRegister(TC1_Callback_InterruptHandler, (uintptr_t)NULL);
    TC1_Timer16bitPeriodSet(1200U);
    /* Start the timer channel 0*/
   // TC1_TimerStart();    
    TC0_Timer16bitPeriodSet(4800U);
    EIC_CallbackRegister(EIC_PIN_2,EIC_User_Handler, 0);
    if(SET_ADDRESS_1_Get()==switch_off && SET_ADDRESS_2_Get()==switch_off)      /*!< For Identifying RCM Address*/
       {
           RCM_Address=0xF1;
       }
    else if(SET_ADDRESS_1_Get()==switch_off && SET_ADDRESS_2_Get()==switch_on)
       {
           RCM_Address=0xF2;
       }
    else if (SET_ADDRESS_1_Get()==switch_on && SET_ADDRESS_2_Get()==switch_off)
       {
           RCM_Address=0xF3;
       }
    else if (SET_ADDRESS_1_Get()==switch_on && SET_ADDRESS_2_Get()==switch_on)
      {
         RCM_Address=0xF0;
      }
    else
      {
        ;   
      }
    txData[0]=0xD0;                                                             /*!< Setting Range=5.12V for External ADC*/
    txData[1]=Range_select;
    txData[2]=0x00;
    txData[3]=0x04;                               
    ADC_CS_Clear();                                                             /*!< CS Pin Low */
    SERCOM1_SPI_Write(txData, txSize);                                          /*!< SPI Write  */
    while(isTransferDone !=true);

    #ifndef _calibration_table
    while(NVMCTRL_IsBusy());  
    NVMCTRL_RWWEEPROM_Read( (uint32_t *)data3, (uint32_t)NVMCTRL_RWWEEPROM_PAGESIZE,  (uint32_t)nvm_rwwee_user_start_address );         /*!< Read 64 byte Data from EEPROM */
    while(NVMCTRL_IsBusy());                                                    /*Wait for  page Read to complete*/
    NVMCTRL_RWWEEPROM_Read( (uint32_t *)data4, (uint32_t)NVMCTRL_RWWEEPROM_PAGESIZE,  (uint32_t)(nvm_rwwee_user_start_address+0x40) );  /*!< Read 64 byte Data from EEPROM */
    while(NVMCTRL_IsBusy());                                                    /*Wait for  page Read to complete*/
    NVMCTRL_RWWEEPROM_Read( (uint32_t *)data5, (uint32_t)NVMCTRL_RWWEEPROM_PAGESIZE,  (uint32_t)(nvm_rwwee_user_start_address+0x80) );  /*!< Read 64 byte Data from EEPROM */
    while(NVMCTRL_IsBusy());                                                    /*Wait for  page Read to complete*/
    
    offset=*(data3);                                                            /*!< Storing Data from buffer*/
    for(uint32_t a1=0;a1<7;a1++)
    {
         for(uint32_t b1=0;b1<4;b1++)
         {
            Adc_calibration[a1][b1]= *(data3+h);
            h++;
         }
    }
    
    for(uint32_t c=7;c<15;c++)                                                  /*!< Storing Data from buffer*/
    {
         for(uint32_t d=0;d<4;d++)
         {
           Adc_calibration[c][d]= *(data4+g);
           g++;
         }
    }
    for(uint32_t x=15;x<20;x++)                                                 /*!<Storing Data from buffer*/
    {
         for(uint32_t y=0;y<4;y++)
         {
           Adc_calibration[x][y]= *(data5+z);
           z++;
         }
    }
    #endif

    freqcountenable = true;                                                     //enable the frequency counter sampling
     
    while ( true )
    { 
        //TimerBaseCodeStore=TimerBaseCode;
        for(uint8_t ii=0;ii<(RMS_AVG-1);ii++)
        {
            fit_Preiod1(FrzCheckCount1Store,FrzCheckCount2Store);
            TC0_Timer16bitPeriodSet(TimerBaseCode);
            //printf("\r\nTimerBaseCode: %d", TimerBaseCode);
            for(uint32_t i=0;i<(TOTAL_SAMPLES+1);i++)                               /*!< Getting 300 samples */
            //for(uint32_t i=0;i<(50+1);i++) 
            {
                TC0_TimerStart();
                txData[0]=0x44;                                                      /*!< Reading ADC Value*/
                txData[1]=0x00;
                ADC_CS_Clear();                                                      /*!< CS Pin Low */
                LED_CONTROL_Toggle();
                SERCOM1_SPI_WriteRead(txData, txSize1,rxData,rxSize);                /*!< SPI Write Read */                 
                                                                                    /*!< Enable the TC counter */
                 
                while(isTransferDone!=true);                                         /*!< wait for SPI write To complete*/
                                                                                    /*!< Disable the TC counter */
                if (isTransferDone==true)
                {
               
                    isTransferDone=false;
                    if(i>0)
                    {
                        Current_Reading[count]=(rxData[0]<< 8) | rxData[1];             /*!< storing received data */
                        count++;
                    }
                    if(count==TOTAL_SAMPLES)                                          /*!< reading 200 ADC samples and doing rms calculations*/
                    { 
                        //Rms_DataAvg = (int) current_cal(&Current_Reading[0],offset);
                        count=0;
                    }
                }
                while(!TC0_TimerPeriodHasExpired());                                 /*!< wait for timer(100us) to Expire*/           
                TC0_TimerStop();   
            }
            Rms_DataAvg = (int) current_cal(&Current_Reading[0],offset);
            Rms_Data+=Rms_DataAvg;

           // aa[ii]=Rms_DataAvg;
           /* if(countaa<RMS_AVG)
            {
                aa[countaa++]=Rms_DataAvg;
            }
            else
            {
                Rms_Data=average(aa,RMS_AVG);
                countaa=0;
                
            }*/
            
          /* if((FrzPreiod>10)&&(FrzPreiod<1001)&&(EICheckSTF==true))//
            {
                if(TimerBaseCodeStore!=TimerBaseCode)
                {
                    TC0_Timer16bitPeriodSet(TimerBaseCode);
                    TimerBaseCodeStore=TimerBaseCode;
                }
            }
            else
            {
                 TC0_Timer16bitPeriodSet(4400U);
                 FrzPreiod=0;
            }*/
        }
        Rms_Data=Rms_Data/RMS_AVG;

       // if(FrzPreiod>1)
      //  Rms_Dataa=fit_Rms(&Rms_Data,FrzPreiod);
        Rms_Dataa=Rms_Data;
       // Rms_Data=average(aa,RMS_AVG-1);
            
      //  printf("\r\nAC Frz = %dHz", FrzPreiod);
      //  printf("\n\n\rRMS_FIT Current: %d", Rms_Dataa);
        //FFFFF=true;
        if((Rms_Dataa>=6)&&(Rms_Dataa<=15))                                       /*!< Rms Current is greater than 6mA  and less than 15mA LED will toggle*/
        {
            //LED_CONTROL_Toggle();                                               /*!< LED Toggle*/         
        }
        else if (Rms_Dataa>15)                                                   /*!< Rms Current is greater than 15mA LED will Glow*/         
        {
            //LED_CONTROL_Clear();                                                /*!< LED ON*/
            ALERT_CONTROL_OUT_Set();
        }
        else
        {
            //LED_CONTROL_Set();                                                  /*!< LED OFF*/
            ALERT_CONTROL_OUT_Clear();
        }        
        
        RX_Count=SERCOM0_USART_ReadCountGet();                                  /*!< size of the received data */
        if(RX_Count==7)                                                         /*!< Received data size size is 7 */
        {
           SERCOM0_USART_Read(&uartRxData[0],7);                                /*!< Read the received data */
           if(uartRxData[0]==0xC0 && uartRxData[6]==0xC0) 
            { 
                crc_1=crc8(0x00,uartRxData[2]);                                 /*!< CRC calculation */
                crc_2=crc8(crc_1,uartRxData[3]);
                crc_3=crc8(crc_2,uartRxData[4]);
                command_type = uartRxData[2];                                   /*!< Getting command type */
                Register_combination=(uartRxData[3]<< 8) | uartRxData[4];
                if ((uartRxData[1]==RCM_Address) && (uartRxData[5]==crc_3) && ((((Register_combination== 0xFE00) || (Register_combination==0xFE01) || (Register_combination==0xFE02) || (Register_combination==0xFE03)) && (command_type == 0x01)) || ((Register_combination==0xFE04) && (command_type == 0x11))))
                {   
                    uartTxData[0]=SLIP_END;                                     /*!< Acknowledgement  command */
                    uartTxData[1]=RCM_Address;
                    uartTxData[2]=ACK;
                    crc = crc8(0x00,ACK);
                    uartTxData[3]=crc;
                    uartTxData[4]=SLIP_END;
                    slip_transmission(&uartTxData[0], 5);
                    if(Register_combination==0xFE00)                            /*!< Firmware version */
                    {
                        uartTxData[0]=SLIP_END;
                        uartTxData[1]=RCM_Address;                              /*!< RCM address */
                        uartTxData[2]=status_byte1;
                        uartTxData[3]=status_byte2;
                        uartTxData[4]=0x00;
                        uartTxData[5]=0x00;
                        uartTxData[6]=0x00;
                        uartTxData[7]=Firmware_version;
                        crc=crc8(0x00,Firmware_version);                        /*!< CRC calculation */
                        uartTxData[8]=crc;
                        uartTxData[9]=SLIP_END;
                        slip_transmission(&uartTxData[0], 10);                  /*!< Data Transmission */
                    }
                    else if(Register_combination==0xFE01)                       /*!< Hardware Version */
                    {
                        uartTxData[0]=SLIP_END;
                        uartTxData[1]=RCM_Address;                              /*!< RCM address */
                        uartTxData[2]=status_byte1;
                        uartTxData[3]=status_byte2;
                        uartTxData[4]=0x00;
                        uartTxData[5]=0x00;
                        uartTxData[6]=0x00;
                        uartTxData[7]=Hardware_version;
                        crc=crc8(0x00,Hardware_version);                        /*!< CRC calculation */
                        uartTxData[8]=crc;
                        uartTxData[9]=SLIP_END;
                        slip_transmission(&uartTxData[0], 10);                  /*!< Data Transmission */
                    }
                    else if(Register_combination==0xFE02)                       /*!< RMS data */
                    {
                        int8_t Rms_Data_H,Rms_Data_L;
                        Rms_Data_H=Rms_Dataa>>8;
                        Rms_Data_L=Rms_Dataa& 0x00ff;
                        uartTxData[0]=SLIP_END;
                        uartTxData[1]=RCM_Address;                              /*!< RCM address */
                        uartTxData[2]=status_byte1;
                        uartTxData[3]=status_byte2;
                        uartTxData[4]=0x00;
                        uartTxData[5]=0x00;
                        uartTxData[6]= Rms_Data_H;
                        uartTxData[7]= Rms_Data_L;        
                        uartTxData[9]=SLIP_END;
                        if(Rms_Data_H==0xC0)                                    /*!< SLIP command packet */
                        {
                            if(Rms_Data_L==0xC0)
                            {
                                uartTxData[4]=SLIP_ESC;
                                uartTxData[5]=SLIP_ESC_END;
                                uartTxData[6]=SLIP_ESC;
                                uartTxData[7]=SLIP_ESC_END;
                            }
                            else if(Rms_Data_L==0xDB) 
                            {
                                uartTxData[4]=SLIP_ESC;
                                uartTxData[5]=SLIP_ESC_ESC;
                                uartTxData[6]=SLIP_ESC;
                                uartTxData[7]=SLIP_ESC_END;  
                            }
                            else
                            {
                                uartTxData[4]=0x00;
                                uartTxData[5]=SLIP_ESC;
                                uartTxData[6]=SLIP_ESC_END;
                            }
                        }
                        else if(Rms_Data_L==0xC0)
                        {
                            uartTxData[4]=0x00;
                            uartTxData[5]=Rms_Data_H;
                            uartTxData[6]=SLIP_ESC;
                            uartTxData[7]=SLIP_ESC_END;
                        }
                        else if(Rms_Data_H==0xDB)
                        {
                            if(Rms_Data_L==0xDB)
                            {
                                uartTxData[4]=SLIP_ESC;
                                uartTxData[5]=SLIP_ESC_ESC;
                                uartTxData[6]=SLIP_ESC;
                                uartTxData[7]=SLIP_ESC_ESC;
                            }
                            else if(Rms_Data_L==0xc0)
                            {
                                uartTxData[4]=SLIP_ESC;
                                uartTxData[5]=SLIP_ESC_END;
                                uartTxData[6]=SLIP_ESC;
                                uartTxData[7]=SLIP_ESC_ESC;
                            }
                            else
                            {
                                uartTxData[4]=0x00;
                                uartTxData[5]=SLIP_ESC;
                                uartTxData[6]=SLIP_ESC_ESC;
                            }
                        }
                        else if(Rms_Data_L==0xDB)
                        {
                            uartTxData[4]=0x00;
                            uartTxData[5]=Rms_Data_H;
                            uartTxData[6]=SLIP_ESC;
                            uartTxData[7]=SLIP_ESC_ESC;
                        }
                       // Rms_Dataa=0;
                        crc_1=crc8(0x00, uartTxData[4]);                        /*!< CRC calculation */
                        crc_2=crc8(crc_1, uartTxData[5]);
                        crc_3=crc8(crc_2, uartTxData[6]);
                        crc=crc8(crc_3, uartTxData[7]);
                        uartTxData[8]=crc;
                        slip_transmission(&uartTxData[0], 10);                  /*!< Data Transmission */
                    }
                    else if(Register_combination==0xFE03)                       /*!< Status */
                    {
                        uartTxData[0]=SLIP_END;
                        uartTxData[1]=RCM_Address;                              /*!< RCM address */
                        uartTxData[2]=status_byte1;
                        uartTxData[3]=status_byte2;
                        uartTxData[4]=0x00;
                        uartTxData[5]=0x00;
                        uartTxData[6]=0x00;
                        uartTxData[7]=Status;
                        crc=crc8(0x00,Status);                                  /*!< CRC calculation */
                        uartTxData[8]=crc;
                        uartTxData[9]=SLIP_END;
                        slip_transmission(&uartTxData[0], 10);                  /*!< Data Transmission */                       
                    }
                    else if(Register_combination==0xFE04)                       /*!< Self Test */
                    {
                        status_val = self_test();
                        uartTxData[0]=SLIP_END;
                        uartTxData[1]=RCM_Address;
                        uartTxData[2]=status_byte1;
                        uartTxData[3]=status_byte2;
                        uartTxData[4]=0x00;
                        uartTxData[5]=0x00;
                        uartTxData[6]=0x00;
                        uartTxData[7]=(status_val & 0xFF);
                        crc=crc8(0x00,(status_val & 0xFF));                     /*!< CRC calculation */
                        uartTxData[8]=crc;
                        uartTxData[9]=SLIP_END;
                        slip_transmission(&uartTxData[0], 10);                  /*!< Data Transmission */
                    }
                }
                else if(uartRxData[1]==RCM_Address)
                {
                    uartTxData[0]=SLIP_END;
                    uartTxData[1]=RCM_Address;
                    uartTxData[2]=NACK;
                    crc=crc8(0x00,NACK);                                        /*!< CRC calculation */
                    uartTxData[3]=crc;
                    uartTxData[4]=SLIP_END; 
                    slip_transmission(&uartTxData[0], 5);                       /*!< Data Transmission */
                }
            }
        }
        else if(RX_Count==11)
        {
           SERCOM0_USART_Read(&uartRxData[0],11);
           if(uartRxData[0]==0xC0 && uartRxData[10]==0xC0) 
            { 
                crc_1=crc8(0x00,uartRxData[2]);                                 /*!< CRC calculation */
                crc_2=crc8(crc_1,uartRxData[3]);
                crc_3=crc8(crc_2,uartRxData[4]);
                crc_4=crc8(crc_3,uartRxData[5]);
                crc_5=crc8(crc_4,uartRxData[6]);
                crc_6=crc8(crc_5,uartRxData[7]);
                crc_7=crc8(crc_6,uartRxData[8]);
                command_type = uartRxData[2];
                Register_combination=(uartRxData[3]<< 8) | uartRxData[4];
                if ((uartRxData[1]==RCM_Address) && (uartRxData[9]==crc_7) && ((Register_combination==0xFE05) && (command_type == 0x11)))       /* Calibration */
                {
                    uartTxData[0]=SLIP_END;                                     /*!< Acknowledgement command */
                    uartTxData[1]=RCM_Address;
                    uartTxData[2]=ACK;
                    crc = crc8(0x00,ACK);
                    uartTxData[3]=crc;
                    uartTxData[4]=SLIP_END;
                    slip_transmission(&uartTxData[0], 5);
                    status_val = calibration(uartRxData[5],uartRxData[7],uartRxData[8]);
                    uartTxData[0]=SLIP_END;
                    uartTxData[1]=RCM_Address;
                    uartTxData[2]=status_byte1;
                    uartTxData[3]=status_byte2;
                    uartTxData[4]=0x00;
                    uartTxData[5]=0x00;
                    uartTxData[6]=0x00;
                    uartTxData[7]=(status_val & 0xFF);                          /*!< CRC calculation */
                    crc=crc8(0x00,(status_val & 0xFF));                         
                    uartTxData[8]=crc;
                    uartTxData[9]=SLIP_END;
                    slip_transmission(&uartTxData[0], 10);                      /*!< Data Transmission */
                }
                else if(uartRxData[1]==RCM_Address)                             /*!< NACK command */
                {
                    uartTxData[0]=SLIP_END;
                    uartTxData[1]=RCM_Address;
                    uartTxData[2]=NACK;
                    crc=crc8(0x00,NACK);                                        /*!< CRC calculation */
                    uartTxData[3]=crc;
                    uartTxData[4]=SLIP_END; 
                    slip_transmission(&uartTxData[0], 5);                       /*!< Data Transmission */
                 
                }
            }
        }  
    UART_clearbuffer();                                                         /*!< Clearing the ring buffer index */
    memset( uartRxData, 0x00, 11 );                                             /*!< Clearing RX buffer */
    memset( uartTxData, 0x00, 10 );                                             /*!< Clearing TX buffer */    
    }
    
    // Execution should not come here during normal operation 
    return ( EXIT_FAILURE );
}

int16_t fit_Rms(int16_t *Rvalue,uint16_t FrzPreiod)
{
    int16_t ret;
    int16_t temp=*Rvalue;
    //uint8_t ff=7;
    if(FrzPreiod>800)
    {
        if(TypeB8_F==true)
            ret=temp*(100+8*TypeB7_compensate)/100;
        else
            ret=temp*(100+8)/100;
    }
    else if(FrzPreiod>600)
    {
        if(TypeB8_F==true)
            ret=temp*(100+5*TypeB7_compensate)/100;
        else
            ret=temp*(100+5)/100;
    }
    else if(FrzPreiod>400)
    {
        if(TypeB8_F==true)
            ret=temp*(100+3*TypeB7_compensate)/100;
        else
            ret=temp*(100+3)/100;
    }
    else if(FrzPreiod>200)    
    {
        if(TypeB8_F==true)
            ret=temp*(100+1*TypeB7_compensate)/100;
        else
            ret=temp*(100+1)/100;
    }
    else
        ret= temp;
    return (ret);
} 

void fit_Preiod1(uint16_t m,uint16_t n)//m=100(100Hz) n=200(50Hz)
{
   // float dvalue;
   float t;
   float Time0preiod;    
   if(m>=n)
   {
        t=m*10/n;
        if((t>15)&&(t<35))
        {
            Time0preiod=m+n;
            TypeB8_F=true;
        }
        else
        {
            Time0preiod= (float)(m+n)/2;   
            TypeB8_F=false;
        }
   }
   else
   {
        t=n*10/m;
        if((t>15)&&(t<35))
        {
            Time0preiod=m+n;
            TypeB8_F=true;
        }
        else
        {
           Time0preiod= (float)(m+n)/2; 
           TypeB8_F=false;
        }
   }
    FrzPreiod= 40000/Time0preiod;
    if(FrzPreiod>250)
        Time0preiod*=5;
    if(FrzPreiod>10)
    {
        TimerBaseCode=(Time0preiod*60-1000)/10;
    }
    else
    {
        TimerBaseCode=4400;            
    }        
   /* if(FrzPreiod>500)
    {
        TimerBaseCodeTemp=(Time0preiod*480-160*10);
        TimerBaseCode=TimerBaseCodeTemp/10;
    }
    else if(FrzPreiod>20)
    {
        TimerBaseCodeTemp=(Time0preiod*240-400*10);
        TimerBaseCode=TimerBaseCodeTemp/10;
    }
    else
    {
        TimerBaseCode=4400;            
    }*/
    //TC0_Timer16bitPeriodSet(TimerBaseCode);
    //printf("\r\nTimerBaseCode:%d",TimerBaseCode);
}
/*******************************************************************************
 End of File
*/

