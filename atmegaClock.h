#ifndef ATMEGA_CLOCK_H
#define ATMEGA_CLOCK_H

// ATmega328p clock with Li-ion battery management
//
// Schematics can be found here:
//     https://easyeda.com/Yulay/atmega328p-clock_copy
//
// Simple clock, based on 32768 Hz crystal oscillator
// with automatic brightness control and power saving modes
// for backup Li-ion 18600 type battery.
// Display TM1637 7-segment LED i2c module
// Fine clock calibration can be achieved by changing correction values in code.

/*
Running on internal 8 MHz clock

CPU Clock Freq = 1 MHz
Main prescaler = 8

Asynchronous clock 32768 Hz on Timer2
PB6 : TOSC1
PB7 : TOSC2

Input ports
PD2 : Inc Minutes
PD3 : Inc Hours
PD4 : Time Correction Mode
PD5 : Snoose
PD6 : Alarm Clock Set Mode
PD7 : Alarm On/Off

PC3 : Power Control TM1637

Timer1 1000000 Hz / 256 / 32 = ~122 Hz

I2C
PC4 : SDA
PC5 : SCL

ADC
ADC0 : Measure ambient light level with Vref = Vcc
ADC BG: Measure bandgap voltage 1.1V with Vref = Vcc ==> measure Vcc

SLEEP MODE
Wakeup sources
PCINT2 pin change
Timer2 1 Hz

*/

#ifndef F_CPU
    #define F_CPU   1000000
#endif

#include <stdint.h>
#include <stdbool.h>

#include <avr/io.h>

#include "tm1637.h"

#define INC_M_IN  PORTD2
#define INC_H_IN  PORTD3
#define TIME_IN  PORTD4
#define SNOOSE_IN  PORTD5
#define ALARM_IN  PORTD6
#define ALARM_ON  PORTD7

#define INPUT_AVERAGING 5

#define RESET_SECS	(_BV(INC_M_IN) | _BV(INC_H_IN))

#define POWER_TM1637	PC3

#define ADC_VREF_VCC        0                           // Vcc as voltage reference
#define ADC_VREF_1V1        _BV(REFS1)                  // 1.1V internal VREF without external capacitor
#define ADC_VREF_2V56      (_BV(REFS2) | _BV(REFS1))    // 2.56V internal VREF without external capacitor

#define VBANDGAP_1v1_IN    (_BV(MUX3) | _BV(MUX2))      //0x0c 1.1V bandgap voltage source
#define TEMPERATURE_IN     (_BV(MUX3) | _BV(MUX2) | _BV(MUX1) | _BV(MUX0)) //0x07 ADC4 internal temperature

#define ADC_LEFT_JUSTIFIED  _BV(ADLAR)  //0x20 - left justified, so we can use ADCH as a 8 bit result

#define VCC_ADC		255
#define BG_VOLTAGE	11/10
#define VOLTAGE(x)	(255 - 96 - ((VCC_ADC * BG_VOLTAGE) / x)

#define BATTERY_FULL			92 // 4.2V
#define BATTERY_NORMAL			87 // 3.9V
#define BATTERY_DISCHARGING		83 // 3.7V


void Setup();

void DisplayTime();


inline void Timer1_Enable()
{
	TIMSK1 |= _BV(OCIE1A);
}

inline void Timer1_Disable()
{
	TIMSK1 &= ~_BV(OCIE1A);
}

inline void StartWaitForRtcTick()
{
	TCCR2B = _BV(CS22) | _BV(CS20);  //prescaler 128}
}

inline void WaitForRtcTick()
{
    while (ASSR & (_BV(TCN2UB) | _BV(OCR2BUB) | _BV(TCR2BUB)))	//Wait until TC2 is updated
    {}
}

inline void ADC_Enable()
{
	PRR &= ~_BV(PRADC);
}

inline void ADC_Disable()
{
	PRR |= _BV(PRADC);
}

inline void ADC_Start()
{
	ADC_Enable();
	ADCSRA |= _BV(ADSC);
}

inline void AdcIn_Voltage()
{
	ADMUX = _BV(REFS0) | _BV(ADLAR) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
}

inline void AdcIn_Light()
{
	ADMUX = _BV(REFS0) | _BV(ADLAR);
}

inline void Buttons_Enable()
{
	PCICR |= _BV(PCIE2);
}

inline void Buttons_Disable()
{
	PCICR &= ~_BV(PCIE2);
}

inline void Display_Initialize()
{
	TM1637_enable(1);
}

inline void Display_Shutdown()
{
	TM1637_enable(0);
}

inline uint8_t Buttons_Read()
{
	return (~PIND) & 0xfc;
}

inline void Display_On()
{
	PORTC |= _BV(PC3);
}

inline void Display_Off()
{
	PORTC &= ~_BV(PC3);
}

inline void SetLED(bool v)
{
	if(v)
	{
		PORTB |= _BV(PORTB5);
	}
	else
	{
		PORTB &= ~_BV(PORTB5);
	}
}

inline void ToggleLED()
{
	PINB |= _BV(PORTB5); //toggle output port
}



#endif //ATMEGA_CLOCK_H
