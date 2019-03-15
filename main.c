
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "inc/tm4c123gh6pm.h"
#include "driverlib/pwm.h"
#include "driverlib/uart.h"
#include "driverlib/rom.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "UART.h"
#include "driverlib/ROM.h"
#include "driverlib/interrupt.h"

int32_t  position_error,pre_position_error;
float	 Part_P,Part_I,Part_D;
float 	Kp=5.2 ,Ki=0.02,Kd=2.9;
int32_t PID_Saturation=200;//MAX DUTY
int32_t averageSpeed1  = 65;
int32_t averageSpeed2  = 65;
int32_t tick = 0;
 /****************************************************************
 * DEFINE
 ****************************************************************/
#define f0          100                  //
#define LED_RED		GPIO_PIN_1
#define LED_BLUE	GPIO_PIN_2
#define LED_GREEN	GPIO_PIN_3

#define MOTOR1_DIR1 GPIO_PIN_4

#define MOTOR2_DIR1 GPIO_PIN_6

//
uint64_t u64_DutyCycle1 = 100;   //Duty cycle of PWM
uint64_t u64_DutyCycle2 = 100;   //Duty cycle of PWM
uint64_t u64_Period1;           //Period of PWM
uint64_t u64_Period2;           //Period of PWM

/*****************************************************************
GLOBAL VARIABLE
******************************************************************/
uint32_t u32_Period0;        // tao mot timer moi voi time delay luu trong ui32Peroid
//uint32_t u32_Period1;      // tao mot timer moi voi time delay luu trong ui32Peroid
uint32_t u32_ADC0Value[8];   // mang chua gia tri  phan tu ADC xu ly doc duoc
uint32_t u32_compareADC1;
uint32_t u32_compareADC2;
uint32_t u32_t1;
uint32_t u32_t2;
uint8_t  count;
uint8_t  count1;

static bool  ControlFlag= false;
volatile uint32_t u32_Value_CH0;         // gia tri nay thay doi lien tuc nen dinh dang kieu bien volatite
volatile uint32_t u32_Value_CH1;
volatile uint32_t u32_Value_CH2;
volatile uint32_t u32_Value_CH3;
volatile uint32_t u32_Value_CH4;
volatile uint32_t u32_Value_CH5;
volatile uint32_t u32_Value_CH6;
volatile uint32_t u32_Value_CH7;
volatile uint32_t u32_Value_AVG;
uint32_t sensors_data[8]={ 0 };

int32_t  set;
/*****************************************************************
extern
******************************************************************/
void Timer0IntHandler(void);

/*****************************************************************
* ROUTINES
******************************************************************/
void Timer0IntHandler(void)
{
     // SS0
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // xoa co ngat khi tran
	ADCIntClear(ADC0_BASE, 0);                            // xoa gia tri ADC trc do
	ControlFlag = false;
	ADCProcessorTrigger(ADC0_BASE, 0);                    // thuc hien trigger ADC
	while(!ADCIntStatus(ADC0_BASE, 0, false))             // neu trang thai khoi tao ADC ok thi thuc hien chuyen doi
	{
	}
	   ADCSequenceDataGet(ADC0_BASE, 0, u32_ADC0Value); // lay du lieu va luu lai
       u32_Value_CH0 = u32_ADC0Value[0];       // sensor1
	   u32_Value_CH1 = u32_ADC0Value[1];       // sensor2
	   u32_Value_CH2 = u32_ADC0Value[2];       // sensor3
	   u32_Value_CH3 = u32_ADC0Value[3];       // sensor4
	   u32_Value_CH4 = u32_ADC0Value[4];       // sensor5
	   u32_Value_CH5 = u32_ADC0Value[5];       // sensor6
	   u32_Value_CH6 = u32_ADC0Value[6];       // sensor7
	   u32_Value_CH7 = u32_ADC0Value[7];       // sensor8
	   u32_Value_AVG = (u32_Value_CH0+u32_Value_CH1+u32_Value_CH2+u32_Value_CH3
			   +u32_Value_CH4+u32_Value_CH5+u32_Value_CH6+u32_Value_CH7)/8;
	   uint8_t i = 0;
	   uint32_t sensors_sum = 0;
	   uint32_t sensors_weighted = 0;
	   for (i = 0; i < 8; i++)
	   {
		   sensors_data[i] = u32_ADC0Value[i];
		   sensors_weighted +=  sensors_data[i] * (i+1) ;  		//Calculating the weighted mean
		   sensors_sum += sensors_data[i];						//Calculating sum of sensor readings
	   }
	   position_error = (int)(10*sensors_weighted / sensors_sum) -set ;

	   if(tick==1)
	   {
	   set = (int)(10*sensors_weighted / sensors_sum);
	   }
	   if(tick==3)
	   {
	   position_error = (int)(10*sensors_weighted / sensors_sum) -set ;
	   ControlFlag = true;
	   }

	//   UARTprintf("%d \n", position_error);
    //UARTprintf("%d %d %d %d %d %d %d %d\n", u32_Value_CH0, u32_Value_CH1, u32_Value_CH2, u32_Value_CH3, u32_Value_CH4, u32_Value_CH5, u32_Value_CH6, u32_Value_CH7 );
}
void Config_Clock(void)
{   // 40Mhz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
}

void Config_ADC(void)
{
	//Configure pin for ADC
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);	//Enable Peripheral Port
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
 	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//ADC pin: PE3
 	GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);	//ADC pin: PE3
 	//Configure ADC module
 	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);		//Enable Module ADC0
    //Configure the hardware oversampling for the ADC - 4 conversions at once
 	ADCHardwareOversampleConfigure(ADC0_BASE, 64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);   // enable ADC0
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);    // cau hinh trigger ADC
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH3);           // lay du lieu ADC thu dc luu vao gia tri cua mang tren
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH2);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 5, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 6, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 7, ADC_CTL_CH4| ADC_CTL_IE| ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 0);           // bat dau nhan du lieu
}
void Config_Timer(void)
{
	//timer0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);       // enable timer0
	TimerIntRegister(TIMER0_BASE, TIMER_A, &Timer0IntHandler);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);  // cau hinh timer0 32bit che do Priodic mode
	u32_Period0 = (40000000/f0);              //chu ki nhiem vu la 50% luu vao ui32Period
	TimerLoadSet(TIMER0_BASE, TIMER_A, u32_Period0 -1);     // set cho timer0A, thoi gian xay ra ngat - 1(vi ngat khi = 0)
//    IntEnable(INT_TIMER0A);                                // cho phep ngat timer0A
//	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // ngat khi timer dem tran
	IntDisable(INT_TIMER0A);                                // cho phep ngat timer0A
	TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // ngat khi timer dem tran
	TimerDisable(TIMER0_BASE, TIMER_A);                     // stop timer
}
void Config_PWM(void)
{
	//Configure PWM clock to match system
	SysCtlPWMClockSet(SYSCTL_PWMDIV_1);
	// config PWM GEN2
	//Enable the peripherals used by this program.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//Tiva Launchpad has 2 modules (0 and 1) and module 1 covers led pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	//Configure PF1 pins as PWM
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);
	//Configure PWM Options
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, PID_Saturation);//PWM frequency = 200Hz
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, u64_DutyCycle1);

	//Turn on the Output pins
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
	PWMGenDisable(PWM1_BASE, PWM_GEN_2);
	//Enable the peripherals used by this program.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);//Tiva Launchpad has 2 modules (0 and 1) and module 1 covers led pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	//Configure PF2 pins as PWM
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
	//Configure PWM Options
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3,PID_Saturation);//PWM frequency = 200Hz
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, u64_DutyCycle2);
	//Turn on the Output pins
	PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);
	PWMGenDisable(PWM1_BASE, PWM_GEN_3);
}
void Config_IO(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);             // enable portF
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, LED_RED|LED_GREEN|LED_BLUE);             // cau hinh output PF123
	GPIOPinWrite(GPIO_PORTF_BASE, LED_RED|LED_BLUE|LED_GREEN, 0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);             // enable portE
	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, MOTOR1_DIR1);
	GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);             // enable portA
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, MOTOR2_DIR1);     // cau hinh output PF123
	GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
}

void motor_enable(void)//DI TOI
{
	GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1, ~MOTOR1_DIR1);
	GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1, ~MOTOR2_DIR1);
}
void config_SPEAKER(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	//Config GPIO that connect to LED - PIN 1,2,3
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE,GPIO_PIN_5);
	//Turn off all LEDs
	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,~GPIO_PIN_5);
}
void ButtonISR(void)
{
		tick++;
		GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_4);//delete flag carry
		if(tick==1)//lay gia tri
		{
			IntEnable(INT_TIMER0A);                                // cho phep ngat timer0A
			TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // ngat khi timer dem tran
			TimerEnable(TIMER0_BASE, TIMER_A);              // bat dau chay timer0->lay ADC
			SysCtlDelay(SysCtlClockGet()/30);
		}
		if(tick==2)//stop , loa keu
		{
			IntDisable(INT_TIMER0A);                                // cho phep ngat timer0A
			TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);       // ngat khi timer dem tran
			TimerDisable(TIMER0_BASE, TIMER_A);              // bat dau chay timer
			//LOA KEU
			GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_PIN_5);
			SysCtlDelay(SysCtlClockGet()/30);
			GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,~GPIO_PIN_5);
		}
		if(tick==3)//enable timer, PWM, ADC,...
		{
			IntEnable(INT_TIMER0A);                              	  // cho phep ngat timer0A
			TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);     	  // ngat khi timer dem tran
			TimerEnable(TIMER0_BASE, TIMER_A);                 		  // bat dau chay timer             		  // bat dau chay timer
			SysCtlDelay(SysCtlClockGet()/30);
		}

}
void config_button(void)
{
	//Config Buttons
	GPIODirModeSet(GPIO_PORTF_BASE,  GPIO_PIN_4, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD_WPU);
	//Config GPIO Interrupt for SW1
	GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_4, GPIO_FALLING_EDGE);
	GPIOIntRegister(GPIO_PORTF_BASE,&ButtonISR);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_PIN_4);
	IntEnable(INT_GPIOF);

}
/******************************************************************************
 * MAIN
 *****************************************************************************/
void main(void)

{
	Config_Clock();
	Config_IO();
	config_button();
	Config_Timer();
	Config_PWM();
	config_SPEAKER();
	Config_ADC();
//	Config_UART();
	IntMasterEnable();
	while(1)
	{
		if(ControlFlag)
		{
//			PWMGenDisable(PWM1_BASE, PWM_GEN_2);
//			PWMGenDisable(PWM1_BASE, PWM_GEN_3);
		//	motor_enable();
			static float u;
					//PID
					Part_P=Kp*position_error;
					Part_I+=Ki*0.1*position_error;
					Part_D=Kd*(position_error-pre_position_error)/0.1;
					u=Part_P+Part_I+Part_D;  //gia tri nhap pwm
					if (u > PID_Saturation)
					{
						u = PID_Saturation;
					}
					if (u < (-PID_Saturation))
					{
						u = -PID_Saturation;
					}
					if(position_error<8 && position_error>-8)
					{
						if(u32_Value_AVG<500)             /// line trang het thang
						{
							GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
							GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
							u64_DutyCycle1= 100;
							u64_DutyCycle2= 100;
						}




						else if(u32_Value_AVG>2000)                // line den het cua trai
						{
							GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
							GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
							u64_DutyCycle1= 100;
							u64_DutyCycle2= 1;

						}




//
//						else if(u32_ADC0Value[0]>2000&&u32_ADC0Value[1]>2000&&u32_ADC0Value[2]>2000&&u32_ADC0Value[3]>2000&&u32_ADC0Value[4]>2000)                // line den het cua phai
//												{
//													GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
//													GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
//													u64_DutyCycle1= 50;
//													u64_DutyCycle2= 1;
//												}



						else                                      // di thang
						{

							count1=1;
							GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
							GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
							u64_DutyCycle1= averageSpeed1 + (int32_t)(u/2);
							u64_DutyCycle2= averageSpeed2 - (int32_t)(u/2);

						}
					}

					if ( count==1 && count1==1)
					{
						GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
						GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
						u64_DutyCycle1= 20;
						u64_DutyCycle2= 20;
						count=0;
						count1=0;

					}
					if(position_error>=8 && position_error <=25) // vach ben trai queo phai
					{
						count=1;
						GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  ~MOTOR1_DIR1);
						GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  MOTOR2_DIR1);
						u64_DutyCycle1= 130;
						u64_DutyCycle2= 100;            //50
					}
					if(position_error<=-8 && position_error>=-25) // vach ben phai queo trai
					{
						count=1;
						GPIOPinWrite(GPIO_PORTE_BASE, MOTOR1_DIR1,  MOTOR1_DIR1);
						GPIOPinWrite(GPIO_PORTA_BASE, MOTOR2_DIR1,  ~MOTOR2_DIR1);
						u64_DutyCycle1= 50;           //50
						u64_DutyCycle2= 170;
					}

					u64_DutyCycle1= averageSpeed1 + (int32_t)(u/2);
					u64_DutyCycle2= averageSpeed2 - (int32_t)(u/2);

					pre_position_error=position_error;

					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5,u64_DutyCycle1);
					PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,u64_DutyCycle2);
					PWMGenEnable(PWM1_BASE, PWM_GEN_2);
					PWMGenEnable(PWM1_BASE, PWM_GEN_3);

					ControlFlag = false;
		}
	}
}
