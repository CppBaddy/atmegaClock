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

enum State
{
    eTimeDisplay    = 0,
    eTimeSet        = kInputTime,
    eAlarmSet       = kInputAlarm,
    eSnoose         = kInputSnoose,
    eCorrectionSet  = kInputAlarm | kInputSnoose
};


volatile static uint8_t timeSec;
volatile static uint8_t timeMin;
volatile static uint8_t timeHour;

volatile static uint8_t alarmOn;
volatile static uint8_t alarmFlag;
volatile static uint8_t alarmMin;
volatile static uint8_t alarmHour;

volatile static uint8_t weekDay;	//0...6, virtual week
volatile static int8_t secCorrection; // = 19; +- weekly seconds correction

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

            alarmOn = (~userInput) & _BV(ALARM_ON);

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

        if(timeHour == alarmHour && timeMin == alarmMin) //checks each minute
        {
            alarmFlag = 1;
            Speaker_Freq(noteA4);
        }
        else if(alarmFlag) //switch off at the end of minute
        {
            alarmFlag = 0;
            Speaker_Off();
        }
	}

	if(alarmFlag)
	{
	    if(alarmOn && (timeSec & 1))
	    {
	        Speaker_On();
	    }
	    else
	    {
	        Speaker_Off();
	    }
	}

	ADC_Start();
}

ISR( TIMER0_COMPA_vect ) //notes driver (second octave)
{
    PORTC |= _BV(SPEAKER_N);
    PORTC &= ~_BV(SPEAKER_P);
}

ISR( TIMER0_COMPB_vect ) //notes driver (second octave)
{
    PORTC |= _BV(SPEAKER_P);
    PORTC &= ~_BV(SPEAKER_N);
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

void Display(State state);

//TODO  
//	add chime at noon, short beep each hour from 8:00 to 20:00
// store clock correction in flash
int main( void )
{
	State state = State::eTimeDisplay;
	
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
            
            //figure out state first, as we need to update display accordingly
            switch(prevInput & kCmdMask)
            {
				case eTimeSet:
					state = eTimeSet;
					break;
				case eAlarmSet:
					state = eAlarmSet;
					break;
				case eSnoose:
					state = eSnoose;
					break;
				case eCorrectionSet:
					state = eCorrectionSet;
					break;
				case eTimeDisplay:
				default:
					state = eTimeDisplay;
					break;
			}
            
            //see what is input modifiers state is
            uint8_t active = (prevInput & change);

            switch(state)
            {
				case eTimeSet:
					if(kInputMinutes & active)
					{
						timeMin = (timeMin + 1) % 60;
					}

					if(kInputHours  & active)
                    {
	                    timeHour = (timeHour + 1) % 24;
                    }
					break;
				case eAlarmSet:
                    if(kInputMinutes & active)
                    {
                        alarmMin = (alarmMin + 1) % 60;
                    }

                    if(kInputHours & active)
                    {
                        alarmHour = (alarmHour + 1) % 24;
                    }
					break;
				case eSnoose:
				    alarmFlag = 0;
				    Speaker_Off();
					break;
				case eCorrectionSet:
                    if(kInputMinutes & active)
                    {
                        if(secCorrection < 99)
                        {
                            ++secCorrection;
                        }
                    }

                    if(kInputHours & active)
                    {
                        if(-99 < secCorrection)
                        {
                            --secCorrection;
                        }
                    }
					break;
				case eTimeDisplay:
				default:
		            if((kResetSecs & prevInput) == kResetSecs) //reset seconds depressed
		            {
		                if(kResetSecs & active)
		                {
		                    timeSec = 0; //reset seconds
		                    //TODO beep
		                }
		            }
					break;
			}
        }
            
        //check power save mode
        switch(powerSaveMode)
        {
        case PowerFull:
            brightness = lightLevel;

            if(batteryVoltage < BATTERY_NORMAL)
            {
                brightness = 2;
                onPowerSave();
            }

            Display(state);
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

                Display(state);
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

        WaitForRtcTick();

        set_sleep_mode(SLEEP_MODE_IDLE);
        sleep_mode();
    }

    return 0;
}

void Setup()
{
	DDRB |= _BV(DDB5); //output for LED

	DDRC |= _BV(POWER_TM1637) | _BV(SPEAKER_N) | _BV(SPEAKER_P); //set output portS

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

    //Timer0 1000 Hz
    TCCR0A = _BV(WGM01); //CTC mode
    TCCR0B = _BV(CS01); //prescaler 8 => 125000 Hz
    OCR0A = noteA4; // A4 la note

    TIMSK2 |= _BV(TOIE2); //enable Timer2 OVF interrupt

    PRR |= _BV(PRTWI) | _BV(PRSPI) | _BV(PRUSART0) | _BV(PRADC); //disable twi, usart and adc clocks

    sei();
}

void Display(State state)
{
    if(alarmFlag && (timeSec & 1))
    {
        return; //No display update during speaker output
    }

    switch(state)
    {
        case eAlarmSet:
            DisplayAlarm();
            break;
        case eCorrectionSet:
            DisplayClockCorrection();
            break;
        default:
            DisplayTime();
            break;
    }
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

void DisplayAlarm()
{
    uint8_t hh = alarmHour / 10;
    uint8_t hl = alarmHour % 10;

    if(0 == hh)
    {
        hh = 10; //blank if zero
    }

    uint8_t mh = alarmMin / 10;
    uint8_t ml = alarmMin % 10;

    TM1637_set_brightness(brightness);

    TM1637_display_colon(timeSec & 1);

    TM1637_display_digit(0, hh);
    TM1637_display_digit(1, hl);
    TM1637_display_digit(2, mh);
    TM1637_display_digit(3, ml);
}

void DisplayClockCorrection()
{
    uint8_t val = secCorrection;
    bool neg = (secCorrection < 0);

    if(neg)
    {
        val = -val;
    }

    uint8_t mh = val / 10;
    uint8_t ml = val % 10;

    TM1637_set_brightness(brightness);

    TM1637_display_colon(0);

    TM1637_display_char(0, 10); //letter C

    if(neg)
    {
        TM1637_display_char(1, 11); //minus sign
    }
    else
    {
        TM1637_display_char(1, 12); //blank
    }

    TM1637_display_digit(2, mh);
    TM1637_display_digit(3, ml);
}

