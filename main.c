/*************************************Librerias**************************************/
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
/************************************Definiciones********************************************/
#define WheelMotorsFreq 1000
#define ServoMotorsFreq 50
#define CLOCK 80000000
//-----------DC Motors Rotation---------//
#define LEFT 0x50
#define RIGHT 0xA0
#define BACKWARD 0x60
#define FORWARD 0x90
#define STOP 0x00
#define ROTATION_PINS GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
//----------------Arm Motors----------//
#define ARMBb_LL 75
#define ARMUb_HL 110
#define EMb_LL 60
#define EMb_HL 105
//----------------Arm Motors----------//
#define ARMB_LL 41
#define ARMB_C 75
#define ARMB_HL 105
//------------------//
#define ARMU_LL 35
#define ARMU_C 75
#define ARMU_HL 107
//---------Camera-Base Motor----//
//#define CAMERAB_LL 55
#define CAMERAB_LL 40
#define CAMERAB_C 75
#define CAMERAB_UL 105
//----------Camera Upper Motor---//
#define CAMERAU_LL 60
#define CAMERAU_C  85
#define CAMERAU_UL 120
//----------Electroiman Motor---//
#define EM_LL 45
#define EM_C 78
#define EM_HL 105
/*************************************Variables************************************************/
//*************PWM*************************//
volatile uint32_t LoadDC;
volatile uint32_t LoadSV;
volatile uint32_t PWMClock;
volatile uint8_t Duty_Cycle_I=100;
volatile uint8_t Duty_Cycle_D=100;
volatile uint8_t ARMU=ARMU_LL;
volatile uint8_t ARMB=ARMB_HL;
volatile uint8_t CAMERAB=CAMERAB_C;
volatile uint8_t CAMERAU=CAMERAU_C;
volatile uint8_t EM=105;
//*************UART*************************//
volatile uint8_t Command;
//*************GPIO*************************//
volatile short LED_TOGGLE=0;
volatile short EM_TOGGLE=0;
/************************************PWM*********************************************************/
void PWM_Init(void){
	PWMClock = CLOCK / 64;
	LoadDC = (PWMClock / WheelMotorsFreq) - 1;
	LoadSV = (PWMClock / ServoMotorsFreq) - 1;
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, LoadDC);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, LoadSV);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, LoadSV);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, LoadSV);
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_5_BIT, true);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM1_BASE, PWM_OUT_3_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_1);

}
/************************************UART*********************************************************/
void UARTIntHandler(void) {
	uint32_t ui32Status;
	ui32Status = UARTIntStatus(UART1_BASE, true);
	UARTIntClear(UART1_BASE, ui32Status);
	while(UARTCharsAvail(UART1_BASE)){
    	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x0C);
		Command = UARTCharGetNonBlocking(UART1_BASE);
		if(Command=='X'){
			LED_TOGGLE^=1;
		}
		else if(Command=='Y'){
			EM_TOGGLE^=1;
		}
		SysCtlDelay(SysCtlClockGet()*5/(1000 * 3));
		}
		GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
}
/************************************Main*********************************************************/

int main(void) {
/************************************Reloj********************************************************/
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_XTAL_16MHZ);
/************************************UART1*********************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
			(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
/************************************GPIOS*********************************************************/
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2|GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, STOP);
/**************************************PWM0*********************************************************/
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PB7_M0PWM1);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PB4_M0PWM2);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PB5_M0PWM3);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PE4_M0PWM4);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PE5_M0PWM5);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinConfigure(GPIO_PA6_M1PWM2);
	GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
	GPIOPinConfigure(GPIO_PA7_M1PWM3);

/*********************************INTERRUPCIONES*****************************************************/
	IntMasterEnable();
	IntEnable(INT_UART1);
	UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
/************************************METODOS********************************************************/
	PWM_Init();
/**************************************INIT**********************************************************/
	Command='0';
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x02);
	SysCtlDelay(SysCtlClockGet()/3);
	GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0x00);
	Command='0';
/*******************************CICLO INFINITO******************************************************/
	while (1){
//-----------------------------------------DC----------------------------------------------------//
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, Duty_Cycle_I*LoadDC/100);
		PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, Duty_Cycle_D*LoadDC/100);
//----------------------------------------Motor--------------------------------------------------//
    	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ARMB*LoadSV/1000);
    	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, ARMU*LoadSV/1000);
//---------------------------------------Camera-------------------------------------------------//
    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, CAMERAU*LoadSV/1000);
    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, CAMERAB*LoadSV/1000);
//-----------------------------------ElectroIman Motor-------------------------------------------//
    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, EM*LoadSV/1000);
    	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, EM*LoadSV/1000);
//-----------------------------------------------------------------------------------------------//
    	GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_2, 4*EM_TOGGLE);
    	GPIOPinWrite(GPIO_PORTB_BASE,  GPIO_PIN_3, 8*LED_TOGGLE);


    	switch(Command){
		    case 'w':
		    	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, FORWARD);
		    	Duty_Cycle_I++;
		    	if(Duty_Cycle_I>100){Duty_Cycle_I=100;}
		    	Duty_Cycle_D++;
		    	if(Duty_Cycle_D>100){Duty_Cycle_D=100;}
		    	break;
		    case 's'  :
		    	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, BACKWARD);
		    	Duty_Cycle_I++;
		    	if(Duty_Cycle_I>100){Duty_Cycle_I=100;}
		    	Duty_Cycle_D++;
		    	if(Duty_Cycle_D>100){Duty_Cycle_D=100;}
		    	break;
		    case 'd'  :
		    	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, RIGHT);
		    	Duty_Cycle_I+=3;
		    	if(Duty_Cycle_I>100){Duty_Cycle_I=100;}
		    	Duty_Cycle_D+=3;
		    	if(Duty_Cycle_D>100){Duty_Cycle_D=100;}
		    	break;
		    case 'a'  :
		    	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, LEFT);
		    	Duty_Cycle_I+=3;
		    	if(Duty_Cycle_I>100){Duty_Cycle_I=100;}
		    	Duty_Cycle_D+=3;
		    	if(Duty_Cycle_D>100){Duty_Cycle_D=100;}
		    	break;
		    case '0'  :
		    	GPIOPinWrite(GPIO_PORTC_BASE,  ROTATION_PINS, STOP);
		    	Duty_Cycle_I=40;
		    	Duty_Cycle_D=40;
		   	    break;
		    case 'N':
		    	ARMU++;
		    	if(ARMU>ARMU_HL){
					ARMU=ARMU_HL;
			    	EM++;
					if(EM>EM_C){EM=EM_C;}
					ARMB--;
					if(ARMB<ARMB_LL){ARMB=ARMB_LL;}
				}
				SysCtlDelay(266666);
		    	break;
		    case 'R':
				ARMU--;
				if(ARMU<ARMU_LL){
					ARMU=ARMU_LL;
			    	EM++;
					if(EM>EM_HL){EM=EM_HL;}
					ARMB++;
					if(ARMB>ARMB_HL){ARMB=ARMB_HL;}
				}
				SysCtlDelay(266666);
		    	break;
		    case 'V':
		    	ARMU++;
		    	EM--;
		    	if(EM<EMb_LL){EM=EMb_LL;}
		    	if(ARMU>ARMUb_HL){
					ARMU=ARMUb_HL;
					ARMB--;
					if(ARMB<ARMBb_LL){ARMB=ARMBb_LL;}
				}
				SysCtlDelay(266666);
				break;
		    case 'L'  :
		    	EM--;
				if(EM<EM_LL){EM=EM_LL;}
				SysCtlDelay(133333);
		    	break;
		    case 'M'  :
		    	EM++;
				if(EM>EM_C){EM=EM_C;}
				SysCtlDelay(133333);
		    	break;
		    case '2'  :
		    	CAMERAU++;
				if(CAMERAU>CAMERAU_UL){CAMERAU=CAMERAU_UL;}
				SysCtlDelay(133333);
		    	break;
		    case '8'  :
		    	CAMERAU--;
				if(CAMERAU<CAMERAU_LL){CAMERAU=CAMERAU_LL;}
				SysCtlDelay(133333);
		    	break;
		    case '4'  :
		    	CAMERAB++;
				if(CAMERAB>CAMERAB_UL){CAMERAB=CAMERAB_UL;}
				SysCtlDelay(133333);
		    	break;
		    case '6'  :
		    	CAMERAB--;
				if(CAMERAB<CAMERAB_LL){CAMERAB=CAMERAB_LL;}
				SysCtlDelay(133333);
		    	break;
	    	case '1'  :
	    		CAMERAB=CAMERAB_C;
	    		CAMERAU=CAMERAU_C;
	    		SysCtlDelay(133333);
	    		break;
		}
    	SysCtlDelay(266666);

	}
}
