#include "atmegaClock.h"

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <util/delay.h>


enum eUserInput
{
    eHours = 1,
    eMinutes,
    eDisplay,
};

enum ePower
{
    PowerFull,
    PowerSave,
    PowerSleep
};


volatile static uint8_t timeSec;
volatile static uint8_t timeMin;
volatile static uint8_t timeHour;

volatile static uint8_t weekDay;	//0...6, virtual week
volatile static int8_t secCorrection = 19; //+- weekly seconds correction

volatile static uint8_t userInputFlag;
volatile static uint8_t userInput;

volatile static uint8_t avgCounter;
volatile static uint8_t inputBuf;

volatile static uint8_t adcState = 1;

volatile static uint8_t batteryVoltage = BATTERY_FULL;
volatile static uint8_t lightLevel = 7;

static uint8_t powerSaveMode;
static uint8_t brightness;

/* PCINT0 pin change handler */
ISR( PCINT2_vect )
{
    inputBuf = Buttons_Read();
    avgCounter = INPUT_AVERAGING;

    Timer1_Enable();
}

/* Timer1 interrupt handler */
ISR( TIMER1_COMPA_vect ) //122 Hz
{
    if(avgCounter)
    {
        --avgCounter;

        uint8_t val = Buttons_Read();

        if(inputBuf != val)
        {
            avgCounter = INPUT_AVERAGING;
            inputBuf = val;
        }
        else if(0 == avgCounter)
        {
            userInputFlag = true;
            userInput = inputBuf;

            Timer1_Disable();
        }
    }
}

/* Timer2 interrupt handler */
ISR( TIMER2_OVF_vect ) //wakes up each second
{
	++timeSec;

	if(60 == timeSec)
	{
		timeSec = 0;
		++timeMin;

		if(60 == timeMin)
		{
			timeMin = 0;
			++timeHour;

			if(24 == timeHour)
			{
				timeHour = 0;
				++weekDay;

				if(weekDay == 7)
				{
					weekDay = 0;
					timeSec += (secCorrection % 7); //leftover correction for a week
				}

				timeSec += (secCorrection / 7); //daily correction
			}
		}
	}

	ADC_Start();
}

// ADC interrupt service routine
ISR( ADC_vect )
{
	if(adcState)
	{
		uint8_t v = ~(ADCH + 96); //converting to positive slope

		batteryVoltage = (batteryVoltage + v) >> 1; //averaging

		if(PowerFull == powerSaveMode)
		{
			AdcIn_Light();

			adcState = 0;

			SetLED(1);
		}
	}
	else
	{
		uint8_t v = (ADCH >> 5) + 2;
		if(v > 7)
		{
			v = 7;
		}

		lightLevel += v;		//averaging
		lightLevel >>= 1;

		AdcIn_Voltage();

		adcState = 1;

		SetLED(0);
	}

	ADC_Disable();
}

void onPowerFull()
{
	powerSaveMode = PowerFull;
}

void onPowerSave()
{
	Display_On();

	powerSaveMode = PowerSave;

	Buttons_Enable();
	TM1637_init(1, brightness); //reinitialize after power restore
}

void onPowerSleep()
{
	powerSaveMode = PowerSleep;

	Display_Off();
	Buttons_Disable();
}

//TODO  
//	add chime at noon, short beep each hour from 8:00 to 20:00
int main( void )
{
    Setup(); //setup avr hardware

    TM1637_init(1, 7);
    
    Display_Initialize();

    uint8_t prevInput = Buttons_Read();

    for(;;)
    {
    	StartWaitForRtcTick();

        if(userInputFlag) //buttons changed their state
        {
            userInputFlag = 0;
            
            uint8_t change = prevInput ^ userInput;
            prevInput = userInput;
            
            if((RESET_SECS & prevInput) == RESET_SECS) //reset seconds depressed
            {
            	if(RESET_SECS & change)
            	{
					timeSec = 0; //reset seconds
            	}
            }
            else if(_BV(TIME_IN) & prevInput) //time correction
            {
            	if(_BV(INC_M_IN) & change & prevInput) //minutes
    			{
    				timeMin = (timeMin + 1) % 60;
    			}
                else if(_BV(INC_H_IN) & change & prevInput) //hours
    			{
    				timeHour = (timeHour + 1) % 24;
    			}
            }
            else if(_BV(ALARM_IN) & prevInput) //alarm correction
            {
//            	if(_BV(INC_M_IN) & change & prevInput) //minutes
//    			{
//    				timeMin = (timeMin + 1) % 60;
//    			}
//                else if(_BV(INC_H_IN) & change & prevInput) //hours
//    			{
//    				timeHour = (timeHour + 1) % 24;
//    			}
            }
            else if(_BV(ALARM_ON) & change & prevInput)
            {
            	//ALARM on/off
            }
        }
        else //check power save mode
        {
			switch(powerSaveMode)
			{
			case PowerFull:
				brightness = lightLevel;

				if(batteryVoltage < BATTERY_NORMAL)
				{
					brightness = 2;
					onPowerSave();
				}

				DisplayTime();
				break;

			case PowerSave:
				if(batteryVoltage < BATTERY_DISCHARGING)
				{
					onPowerSleep();
				}
				else
				{
					if(batteryVoltage > BATTERY_NORMAL)
					{
						onPowerFull();
					}

					DisplayTime();
				}
				break;

			case PowerSleep:
				if(batteryVoltage > BATTERY_DISCHARGING)
				{
					onPowerSave();
				}
				break;

			default:
				powerSaveMode = PowerFull;
				break;
			}
        }

        WaitForRtcTick();

        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }

    return 0;
}

inline void Setup()
{
	DDRB |= _BV(DDB5); //output for LED

	DDRC |= _BV(POWER_TM1637); //set output port

    PORTD |= _BV(INC_H_IN) | _BV(INC_M_IN) | 0xfc;  //enabling pull ups
    PCMSK2 |= _BV(INC_H_IN) | _BV(INC_M_IN) | 0xfc;  //enable pin change interrupt mask
    Buttons_Enable();                      //enable pin change interrupt

    Display_On();

    DIDR0 |= _BV(ADC0D); //disable digital input on ADC0

    ADCSRA |= _BV(ADPS2) | _BV(ADIE) | _BV(ADEN); //enable ADC, interrupt and set prescaler 16
    AdcIn_Voltage();

    //Timer2 1Hz time clock (32768 Hz / 128 / 256) = 1 Hz
    ASSR |= _BV(AS2);	//Async mode

    TCNT2 = 0;
    StartWaitForRtcTick();
    WaitForRtcTick();

    //Timer1 122 Hz
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS12); //CTC, prescaler 256
    OCR1A = 32;

    TIMSK2 |= _BV(TOIE2); //enable Timer2 OVF interrupt

    PRR |= _BV(PRTWI) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC); //disable twi, usart and adc clocks

    sei();
}

void DisplayTime()
{
    uint8_t hh = timeHour / 10;
    uint8_t hl = timeHour % 10;

    if(0 == hh)
    {
        hh = 10; //blank if zero
    }
    
    uint8_t mh = timeMin / 10;
    uint8_t ml = timeMin % 10;

    TM1637_set_brightness(brightness);
    
    TM1637_display_colon(timeSec & 1);

    TM1637_display_digit(0, hh);
    TM1637_display_digit(1, hl);
    TM1637_display_digit(2, mh);
    TM1637_display_digit(3, ml);
}



