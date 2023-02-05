/* Atmega328P @16MHz
 * main.c
 *
 *  Created on: Nov 25, 2022
 *      Author: jcaf
 *
 *
 *https://www.engbedded.com/fusecalc/
 http://eleccelerator.com/fusecalc/fusecalc.php?chip=atmega328p


 *      avrdude -c usbasp -B5 -p m328P
avrdude -c usbasp -B5 -p m328P -U lfuse:w:0xD6:m -U hfuse:w:0xD1:m -U efuse:w:0xFE:m


- No full swing crytal + 65ms
- No divide clock by 8
- BODEN 4.3V

avrdude -c usbasp -B5 -p m328P -U lfuse:w:0xff:m -U hfuse:w:0xd9:m -U efuse:w:0xfc:m
 */

#include "main.h"
#include "lcdan/lcdan.h"
#include "lcdan/lcdan_aux.h"
#include "i2c/I2C.h"
#include "i2c/I2CCommonFx.h"
#include "INA238/INA238.h"
#include "ads1115/ads1115.h"
#include "kb_setup.h"
#include "ikb/ikb.h"
#include "kb_setup.h"
#include "indicator/indicator.h"
///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
float mv1;
float current;

volatile struct _isr_flag
{
    unsigned tick :1;
    unsigned __a :7;
} isr_flag;


struct _main_flag main_flag;

struct _job
{
	int8_t sm0;//x jobs

	uint16_t counter0;
	uint16_t counter1;
	//int8_t mode;

	struct _job_f
	{
		unsigned enable:1;
		unsigned job:1;
		unsigned lock:1;
		unsigned capturing:1;
		unsigned __a:4;
	}f;
};

struct _smoothAlg
{
	int8_t sm0;//x jobs
	uint16_t counter0;
	float average;
	int16_t Pos;	//# de elementos > que la media
	int16_t Neg;	//# de elementos > que la media
	float TD;	//Total Deviation
	int SMOOTHALG_MAXSIZE;
};


struct _job job_reset;
struct _smoothAlg smoothAlg_reset;
//
struct _job job_capture_mvx;
struct _smoothAlg smoothAlg_mvx;
//
struct _job job_capture_current;
struct _smoothAlg smoothAlg_current;
//

struct _job jobBotonP1;

int8_t smoothAlg_nonblock(struct _smoothAlg *smooth, int16_t *buffer, int SMOOTHALG_MAXSIZE, float *Answer);

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

enum _JOB_CMD
{
	JOB_STOP = 0,
	JOB_START,
	//JOB_RESTART
};
void job_cmd(struct _job *job, enum _JOB_CMD cmd)
{
	if (cmd == JOB_START)
	{
		job->sm0 = JOB_START;
	}
	else if (cmd == JOB_STOP)
	{
		//job->sm0 = JOB_STOP;
		*job = job_reset;
	}
}


/* El voltaje de polarizacion se resta ni bien inicia la captura y se mantiene la resta hasta las 1200,
 * se sigue midiendo mv1 en todo momento */
#define P1_TIME_ACTIVATE_OUT1 200E-3	//200mS

/* Define the time of capturing */
#define P1_TIME_CAPTURING 1200E-3		//1200mS

/* Define the time of capturing */
#define P1_TIME_DEACTIVATE_OUT1 1400E-3	//1400mS

//int8_t
float mv1_last;
void job_boton_p1(void)
{
	if (jobBotonP1.sm0 == JOB_STOP)
	{
		jobBotonP1.f.capturing = 0;
		jobBotonP1 = job_reset;
		jobBotonP1.sm0--;

		//ADDED
		main_flag.freeze_capture_in_display = 0; //unlock viewing in LCD
	}

	else if (jobBotonP1.sm0 == JOB_START)
	{
		jobBotonP1.f.capturing = 1;

		mv1_last = mv1;
		//
		jobBotonP1.counter0 = 0;
		//
		jobBotonP1.sm0++;

	}
	else if (jobBotonP1.sm0 == 2)
	{
		if (main_flag.systick)
		{
			if (++jobBotonP1.counter0 >= (P1_TIME_ACTIVATE_OUT1 /SYSTICK ))//200
			{
				//ACTIVA OUT 1
				PinTo1(PORTWxRELAY1,PINxKB_RELAY1);

				jobBotonP1.sm0++;
			}
		}
	}
	else if (jobBotonP1.sm0 == 3)
	{
		if (main_flag.systick)
		{
			if (++jobBotonP1.counter0 >= (P1_TIME_CAPTURING /SYSTICK ))//1200
			{
				jobBotonP1.f.capturing = 0;
				main_flag.freeze_capture_in_display = 1;// frezze
				indicator_Off();
				//
				jobBotonP1.sm0++;
			}
		}
	}
	else if (jobBotonP1.sm0 == 4)
	{
		if (main_flag.systick)
		{
			if (++jobBotonP1.counter0 >= (P1_TIME_DEACTIVATE_OUT1 /SYSTICK ))
			{
				//DEACTIVATE OUT 1
				PinTo0(PORTWxRELAY1,PINxKB_RELAY1);

				main_flag.sw1_lock = 0; //unlock switch de captura, el usuario ya puede volver a presionar

				//jobBotonP1.counter0 = 0;

				jobBotonP1.sm0++;

			}
		}
	}


	//return 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////

#define ADS1115_SMOOTHALG_MAXSIZE 55//55
//#define ADS1115_SMOOTHALG_MAXSIZE 0
#define ADS1115_KTIME_CAPTURE_AVERAGE 0//8//8//ADS1115 DATARATE = 128 -> 1/128 = 7.8ms

#define MV1_KCORRECTION +0.120E-3	//mV
//#define MV2_KCORRECTION +0.5E-3	//mV
//#define MV3_KCORRECTION -0.7E-3	//mV


int8_t ADS1115_capture_mvx(float *mvx)
{
	int16_t ib16;
	float smoothAnswer;
	uint8_t reg[2];
	//
	static int16_t smoothVector[ADS1115_SMOOTHALG_MAXSIZE];

	if (job_capture_mvx.sm0 == 0)
	{
		if (main_flag.systick)
		{
			if (++job_capture_mvx.counter1 >= ADS1115_KTIME_CAPTURE_AVERAGE)
			{
				job_capture_mvx.counter1 = 0;

				//Aqui ya tengo la primera muestra
				I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
				ib16 = (reg[0]<<8) + reg[1];


				smoothVector[job_capture_mvx.counter0] = ib16;
				if (++job_capture_mvx.counter0 >= ADS1115_SMOOTHALG_MAXSIZE)
				{
					job_capture_mvx.counter0 = 0x00;
					//job_capture_mvx.sm0 = 2;//calcular smooth
					job_capture_mvx.sm0++;//calcular smooth
				}
				else
				{
					job_capture_mvx.counter1 = 0;
					//job_capture_mvx.sm0 = 1;//volver a ejercer un cierto intervalo entre cada muestra para el smooth
				}
			}
		}
	}
	else if (job_capture_mvx.sm0 == 1)
	{
		#if ADS1115_SMOOTHALG_MAXSIZE == 0
				*mvx = (smoothVector[0]*-2048.00f/32768);//expresado en mV
				job_capture_mvx.sm0 = 0x0;
				return 1;
		#else
			if (smoothAlg_nonblock(&smoothAlg_mvx, smoothVector, ADS1115_SMOOTHALG_MAXSIZE, &smoothAnswer))
			{
				//smoothAnswer*=-1;//invirtiendo la senal
				//*mvx = (smoothAnswer*2.048f/32768);//expresados en Voltios..tal como es
				*mvx = (smoothAnswer*-2048.00f/32768);//expresado en mV

				job_capture_mvx.sm0 = 0x0;
				return 1;
			}
		#endif
	}
	//
	return 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////
//#define INA238_KTIME_CAPTURE_AVERAGE 51//ms INA238_ADC_CONFIGMODE_CT_150uS *  INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024
//#define INA238_KTIME_CAPTURE_AVERAGE 36//ms INA238_ADC_CONFIGMODE_CT_150uS *  INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024
#define INA238_KTIME_CAPTURE_AVERAGE 154//ms INA238_ADC_CONFIGMODE_CT_150uS *  INA238_ADC_CONFIGMODE_AVG_SAMPLE_1024
#define INA238_SMOOTHALG_MAXSIZE 2

#define CURRENT_FACTOR_CORRECTION 1.0209F//

int8_t IN238_capture_current(float *current)
{
	float smoothAnswer;

	//
	static int16_t smoothVector[INA238_SMOOTHALG_MAXSIZE];

	if (job_capture_current.sm0 == 0)
	{
		if (main_flag.systick)
		{
			if (++job_capture_current.counter1 >= INA238_KTIME_CAPTURE_AVERAGE)
			{
				job_capture_current.counter1 = 0;

				//Aqui ya tengo la primera muestra
				smoothVector[job_capture_current.counter0] = INA238_read_current_register();
				if (++job_capture_current.counter0 >= INA238_SMOOTHALG_MAXSIZE)
				{
					job_capture_current.counter0 = 0x00;
					job_capture_current.sm0++;//calcular smooth
				}
				else
				{
					job_capture_current.counter1 = 0;
				}
			}
		}
	}
	else if (job_capture_current.sm0 == 1)
	{
		#if INA238_SMOOTHALG_MAXSIZE <= 1
			*current = smoothVector[0] * INA238_CURRENT_LSB *1000.0f * CURRENT_FACTOR_CORRECTION;
			job_capture_current.sm0 = 0x0;
			return 1;
		#else
			if (smoothAlg_nonblock(&smoothAlg_current, smoothVector, INA238_SMOOTHALG_MAXSIZE, &smoothAnswer))
			{
				*current = smoothAnswer * INA238_CURRENT_LSB;

				*current *=1000.0f*CURRENT_FACTOR_CORRECTION;	//convert a miliamperios

//				if (*current <=0.002f)
//					{*current = 0.000f;}

				job_capture_current.sm0 = 0x0;
				return 1;
			}
		#endif
	}
	//
	return 0;
}
void windows0(void)
{
	lcdan_set_cursor_in_row0(6);
	lcdan_print_PSTRstring(PSTR("SISTEMA"));
	lcdan_set_cursor_in_row1(6);
	lcdan_print_PSTRstring(PSTR("CONTROL"));
	lcdan_set_cursor_in_row2(5);
	lcdan_print_PSTRstring(PSTR("INICIANDO"));
	lcdan_set_cursor_in_row3(5);
	lcdan_print_PSTRstring(PSTR("SERIE-1000"));

}
void windows1(void)
{
	lcdan_set_cursor_in_row0(0);
	lcdan_print_PSTRstring(PSTR("VOLTAJE (mV)"));
	lcdan_set_cursor_in_row1(4);
	lcdan_print_PSTRstring(PSTR("V:"));
	lcdan_set_cursor_in_row2(0);
	lcdan_print_PSTRstring(PSTR("CORRIENTE (mA)"));
	lcdan_set_cursor_in_row3(4);
	lcdan_print_PSTRstring(PSTR("I:"));
}


/* Define the cycle time for the capturing*/
#define BUZZER_TIME_BLINK_CYCLE 100E-3

/* Define the time for the tick of release*/
#define BUZZER_TIME_RELEASE 75E-3


float biased_voltage;

int main(void)
{


	int8_t c = 0;
	char str_formatted[12];
	char buff[30];

	PORTB = PORTC = PORTD= 0;

	ConfigOutputPin(CONFIGIOxRELAY1,PINxKB_RELAY1);
	ConfigOutputPin(CONFIGIOxRELAY2,PINxKB_RELAY2);
	ConfigOutputPin(CONFIGIOxRELAY3,PINxKB_RELAY3);
	//
	ConfigOutputPin(CONFIGIOxBUZZER, PINxBUZZER);
	indicator_setPortPin(&PORTWxBUZZER, PINxBUZZER);
	indicator_setKSysTickTime_ms(75E-3/SYSTICK);

	lcdan_init();
	ikb_init();
	//
	I2C_unimaster_init(100E3);//100KHz
	ADS1115_init();//ADS1115 in powerdown state
	INA238_init();//INA238_REG_CONFI to ± 40.96 mV --> INA238_1_LSB_STEPSIZE_ADCRANGE_40p96mV 1.25E-6 para todos los calculos

	//Atmega328P TCNT0 CTC mode
	TCNT0 = 0x0000;
	TCCR0A = (1 << WGM01);
	TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00); //CTC PRES = 64
	OCR0A = CTC_SET_OCR_BYTIME(1E-3, 64);
	//OCR0A = 249;//CTC_SET_OCR_BYTIME(SYSTICK, 64);
	TIMSK0 |= (1 << OCIE0A);
	//

	sei();

	windows0();
	__delay_ms(2000);
	lcdan_clear();

	windows1();


	//Capturing voltage
	ADS1115_setMuxChannel(MUX_AIN0_AIN3);//mv1
	ADS1115_setOS(1);//wakeup ADS1115
	ADS1115_setOperatingMode(CONTINUOUS_CONV);

	while (1)
	{
		if (isr_flag.tick)
		{
			isr_flag.tick = 0;
			main_flag.systick = 1;
		}
		//----------------------------------
		if (main_flag.systick)
		{
			if (++c >= (20E-3/SYSTICK) )
			{
				c = 0;
				//
				ikey_scan(key, KB_NUM_KEYS, 1);
				ikey_parsing(key, KB_NUM_KEYS);

				/*
				 * 	BOTON - P1
					EJEC 1: PRESIONA BOTON P1 UNA VEZ ACTIVA OUT 1 (ESTADO ALTO)
				*/
				if (ikb_key_is_ready2read(key,0))
				{
					ikb_key_was_read(key,0);



					if (!main_flag.sw1_lock)
					{
						//
						main_flag.sw1_toggle = !main_flag.sw1_toggle;

						if (main_flag.sw1_toggle)//active P1
						{
							job_cmd(&jobBotonP1, JOB_START);
							indicator_setKSysTickTime_ms(BUZZER_TIME_BLINK_CYCLE/SYSTICK);
							indicator_cycle_start();
							//
							main_flag.sw1_lock = 1;//bloquear hasta que termine la captura
						}
						else
						{
							job_cmd(&jobBotonP1, JOB_STOP);
							indicator_setKSysTickTime_ms(BUZZER_TIME_RELEASE/SYSTICK);
							indicator_On();
						}
					}
				}
			}
		}


		//---------------------------
		//Capturing current
		if (1)//main_flag.send_corriente)
		{
			if (IN238_capture_current(&current))
			{
				//USB_send_data(USB_DATACODE_CURRENT, current);
			}
		}

		main_flag.mv1_updated = ADS1115_capture_mvx(&mv1);
		if (main_flag.mv1_updated)//finalizo
		{
			mv1-=0.125;
			//
			if (mv1> 0.0001f)
			{
				//mv1+=MV1_KCORRECTION;
				//mv1-=0.100;
			}
			//
			//ADS1115_setOperatingMode(SINGLESHOT_POWERDOWN_CONV);


		}

		job_boton_p1();//update capturing



		//---------------------------

		//print V + I
		if (main_flag.freeze_capture_in_display == 0)//muestra directamente, y si es 1, lo ultimo que sacó en el LCD se muestra.
		{
			if (main_flag.mv1_updated)
			{
				if (jobBotonP1.f.capturing == 1)
				{
					mv1 -= mv1_last;
				}
				dtostrf(mv1, 0, 3, buff);//solo 3 decimales


				//lcdan_print_string(buff);
				lcdan_prepare_str_clearformat(str_formatted,12, buff,0);
				lcdan_set_cursor_in_row1(6);
				lcdan_print_string(str_formatted);
			}

			//
			dtostrf(current, 0, 3, buff);//current in mV + mA
			//lcdan_print_string(buff);
			lcdan_prepare_str_clearformat(str_formatted,12, buff,0);
			lcdan_set_cursor_in_row3(6);
			lcdan_print_string(str_formatted);

		}

		/* actua sobre el buzzer */
		indicator_job();

		//---------------------------
		//end of main while (1)
		main_flag.systick = 0;

	}

	return 0;
}

ISR(TIMER0_COMPA_vect)
{
    isr_flag.tick = 1;
}



int8_t smoothAlg_nonblock(struct _smoothAlg *smooth, int16_t *buffer, int SMOOTHALG_MAXSIZE, float *Answer)
{
//	static float average=0;
//	static int16_t Pos;	//# de elementos > que la media
//	static int16_t Neg;	//# de elementos > que la media
//	static float TD;	//Total Deviation
//

	//1- Calculate media
	if (smooth->sm0 == 0)
	{
		smooth->average = 0;
		smooth->counter0 = 0x0;
		smooth->sm0++;
	}
	if (smooth->sm0 == 1)
	{
		smooth->average +=buffer[smooth->counter0];

		if (++smooth->counter0 >= SMOOTHALG_MAXSIZE)
		{
			smooth->counter0 = 0x00;//bug fixed

			smooth->average /= SMOOTHALG_MAXSIZE;
			//
			smooth->Pos = 0;
			smooth->Neg = 0;
			smooth->TD = 0;
			smooth->sm0++;
		}
	}
	//2 - Find Pos and Neg + |Dtotal|
	else if (smooth->sm0 == 2)
	{
		if (buffer[smooth->counter0] > smooth->average)
		{
			smooth->Pos++;
			smooth->TD += ( ((float)(buffer[smooth->counter0]))-smooth->average);//Find |Dtotal|
		}
		if (buffer[smooth->counter0] < smooth->average)
		{
			smooth->Neg++;
		}
		//
		if (++smooth->counter0 >= SMOOTHALG_MAXSIZE)
		{
			smooth->counter0 = 0;
			smooth->sm0 = 0;
			//bug
			if (smooth->TD<0)
			{
				smooth->TD *= -1;//convirtiendo a positivo
			}
			//
			*Answer = smooth->average + ( ( (smooth->Pos-smooth->Neg) * smooth->TD )/ ( SMOOTHALG_MAXSIZE*SMOOTHALG_MAXSIZE) );
			return 1;
			//
		}
	}
	return 0;
}
