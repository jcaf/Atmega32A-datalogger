/*
 * main.c
 *
 *  Created on: Apr 13, 2021
 *      Author: jcaf

 1) programar fuse (PRESERVANDO EEPROM)
 avrdude -c usbasp -B5 -p m32 -U lfuse:w:0xbf:m -U hfuse:w:0xc7:m

 2)
 [root@JCAFPC Release]# avrdude -c usbasp -B5 -p m32 -U flash:w:.hex
 [root@JCAFPC Release]# avrdude -c usbasp -B0.3 -p m32 -V -U flash:w:.hex
 GRABAR LA EEPROM
 [jcaf@jcafpc Release]$ avrdude -c usbasp -B4 -p m32 -V -U eeprom:w:digitalFryer.eep

 * Verificar los fuses
 [jcaf@jcafpc Release]$ avrdude -c usbasp -B4 -p m32 -U lfuse:r:-:i -v

 Atmega32A proteger flash (modo 3): lectura y escritura
 avrdude -c usbasp -B10 -p m32 -U lock:w:0xFC:m

Consistenciado:
--------------
APAGAR OUT1, SOLO EN MENU 1 PRENDER
APAGAR OUT2, SOLO EN MENU 2 PRENDER
PRIMER TOQUE EN X3, HABILITA A,B,C, SEGUNDO TOQUE EN X3, DESH.A,B,C
MENU1-> OUT1
MENU2-> OUT2
X3 - > TERMINA TODA LA SECUENCIA Y DEJA OUT3,OUT4 A CERO
X3 - > BUCLE OK
X4 -> ACTIVA OUT1
X5 -> ACTIVA OUT2

 */
#include "system.h"
#include "types.h"
#include "lcdan/lcdan.h"
#include "lcdan/lcdan_aux.h"
#include "ikey/ikey.h"
#include "ikey/kb.h"
#include "adc/adc.h"
#include "main.h"
#include "pinGetLevel/pinGetLevel.h"

volatile struct _isr_flag
{
	unsigned f1ms :1;
	unsigned __a :7;
} isr_flag;

struct _main_flag
{
	unsigned f1ms :1;
	unsigned X1onoff:1;
	unsigned keysEnable:1;
	unsigned __a:5;

} main_flag;

struct _job
{
	int8_t sm0;//x jobs
	int8_t key_sm0;//x keys
	uint16_t counter;
	int8_t mode;

	struct _job_f
	{
		unsigned enable:1;
		unsigned job:1;
		unsigned lock:1;
		unsigned __a:5;
	}f;
};
//
#define BUZZERMODE_TACTSW 0
#define BUZZERMODE_X3_SEQUENCER 1

//static
const struct _job emptyJob;
struct _job keyX1, keyA,keyB,keyC,keyX2,keyX3,keyX4,keyX5;
struct _job measVoltBatt, measVoltGenerador;
struct _job keyP1, keyP2;
struct _job buzzer;
struct _job progressBar;
struct _job smoothAlgJob;
//
int8_t smoothAlg_nonblock(uint16_t *buffer, float *Answer);//Non-blocking
float smoothAlg(uint16_t *buffer);//blocking
#define SMOOTHALG_MAXSIZE 10
uint16_t VoltBatt_buffer[SMOOTHALG_MAXSIZE];
float VoltBatt = 0;
uint16_t VoltGenerator_buffer[SMOOTHALG_MAXSIZE];
float VoltGenerator = 0;

float batteryVoltType=0;

#define BATTERYVOLTTYPE_NUMMAX 2
float batteryVoltTypeArr[BATTERYVOLTTYPE_NUMMAX] = {12,24};//Solo 2 tipos de baterias
#define BATTERY_KDIFF_FLOAT 5	//+- 5 volts
char str_batteryVoltType[10];

#define BATTERYVOLT_MINIMUM_ALERT 10//10 Volts
/*
 * ADC CHANNEL 1
 * ADC calibration, Medir en ckto
 * AREF Nominal(5V), Medido =
 * R1 Nominal(100k), Medido =
 * R2 Nominal(510k), Medido =
 *
 * INTERNAL AREF NOMINAL(5)
 *
 * VCC
 * | (510K)
 * R2
 * |
 * +---->
 * |
 * R1 (100K)
 * |
 * GND
 *
 */
#define ADCH1_R1 100E3//100E3	//MEDIR 99.3
#define ADCH1_R2 510E3	//510E3MEDIR
#define ADC_AREF 5		//5V MEDIR
#define ADC_TOP 1023	//KTE
/*
 * update VoltBatt
 */
int8_t VoltBatt_get_nonblock(void)	//Non-blocking
{
	float smoothADCHL = 0;

	if (measVoltBatt.sm0 == 0)
	{
		VoltBatt_buffer[measVoltBatt.counter] = ADC_read(ADC_CH_1);
		if (++measVoltBatt.counter >= SMOOTHALG_MAXSIZE)
		{
			measVoltBatt.counter = 0x0000;
			measVoltBatt.sm0++;
		}
	}
	else if (measVoltBatt.sm0 == 1)
	{
		if (smoothAlg_nonblock(VoltBatt_buffer, &smoothADCHL))
		{
			measVoltBatt.sm0 = 0x0;
			//
			VoltBatt = (smoothADCHL * ADC_AREF * (ADCH1_R1 + ADCH1_R2) )/ (ADC_TOP * ADCH1_R1);
			return 1;
		}
	}
	return 0;
}

int8_t VoltBatt_get(void)//Blocking
{
	float smoothADCHL = 0;
	VoltBatt_buffer[measVoltBatt.counter] = ADC_read(ADC_CH_1);
	if (++measVoltBatt.counter >= SMOOTHALG_MAXSIZE)
	{
		measVoltBatt.counter = 0x0000;
		smoothADCHL= smoothAlg(VoltBatt_buffer);
		//
		VoltBatt = (smoothADCHL * ADC_AREF * (ADCH1_R1 + ADCH1_R2) )/ (ADC_TOP * ADCH1_R1);
		//
		return 1;
	}
	return 0;
}

/*
 * Voltage Generator 100v
 */
#define ADCH0_R1 33E3	//MEDIR
#define ADCH0_R2 6.8E6	//MEDIR
#define ADC_AREF 5		//5V MEDIR
#define ADC_TOP 1023	//KTE
float smoothGen_ADCHL=0;

//---------------------------------------------------------------------------------------------
//-------- Definicion del usuario de tiempos de manera general, excepto la secuencia para P1
#define KEY_TIMEPRESSING 	20//ms Tiempo de pulsacion
#define RELAY_TIMESWITCHING 40//ms Tiempo de conmutacion de Relays

//delay x menu intro
#define MENU_INTRODUCTION_AFTERDELAY_SEG 5//5 Seconds delay x Menu Introduccion

//buzzer delays
#define BUZZER_TACTSW_KTIME 50	//50ms
#define BUZZER_X3_KTIME 500		//500ms x Secuencia de P1

//---------------------------------------------------------------------------------------------

void lcdan_print_Introduction(void)
{
	char str[LCDAN_STR_MAXSIZE];
	lcdan_str_lineformat_align_P(str, PSTR("SISTEMA"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 0);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("CONTROL"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 1);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("INICIANDO"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 2);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("SERIE-200"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 3);	lcdan_print_string(str);
}

void lcdan_print_Menu1(void)
{
	char str[LCDAN_STR_MAXSIZE];
	char buff[LCDAN_STR_MAXSIZE];
	strcpy(buff,"BATERIA ");
	strcat(buff, str_batteryVoltType);
	lcdan_str_lineformat_align(str, buff, LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 0);	lcdan_print_string(str);

	lcdan_str_lineformat_align_P(str, PSTR(""), 0);
	lcdan_set_cursor(0, 1);	lcdan_print_string(str);

	lcdan_str_lineformat_align_P(str, PSTR("VOLTAJE:"), 0);
	lcdan_set_cursor(0, 2);	lcdan_print_string(str);

	lcdan_str_lineformat_align_P(str, PSTR(""), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 3);	lcdan_print_string(str);


}
void lcdan_print_Menu2(void)
{
	char str[LCDAN_STR_MAXSIZE];
	lcdan_str_lineformat_align_P(str, PSTR("CONTROL"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 0);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("LECTURA"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 1);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR(""), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 2);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("VOLTAJE:"), 0);
	lcdan_set_cursor(0, 3);	lcdan_print_string(str);
}
void lcdan_print_Menu3(void)
{
	char str[LCDAN_STR_MAXSIZE];
	lcdan_str_lineformat_align_P(str, PSTR("VERIFICACION"), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 0);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR(""), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 1);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("PROGRESO"), 0);
	lcdan_set_cursor(0, 2);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR(""), LCDAN_STR_FORMAT_ALIGN_CENTER);
	lcdan_set_cursor(0, 3);	lcdan_print_string(str);
}

void outputs_clear(void)
{
	PinTo0(PORTWxOUT_Z, PINxOUT_Z);
	PinTo0(PORTWxOUT_Y, PINxOUT_Y);
	PinTo0(PORTWxOUT_X, PINxOUT_X);

	PinTo0(PORTWxOUT_4, PINxOUT_4);
	PinTo0(PORTWxOUT_3, PINxOUT_3);
	PinTo0(PORTWxOUT_2, PINxOUT_2);
	PinTo0(PORTWxOUT_1, PINxOUT_1);

	PinTo0(PORTWxOUT_R, PINxOUT_R);
	PinTo0(PORTWxOUT_S, PINxOUT_S);
	PinTo0(PORTWxOUT_T, PINxOUT_T);
}
/*
 * Tiempos en milisegundos
 */
#define P1_T1 20
#define P1_T2 80
#define P1_T3 6000
#define P1_T4 6020
#define P1_T5 6120

//#define P1JOB_TOTALTIME (P1_T1 + P1_T2 + P1_T3 + P1_T4 + P1_T5)
#define P1JOB_TOTALTIME (P1_T5)

int8_t keyP1_job(void)//secuencia
{
	if (keyP1.sm0 == 0)
	{
		PinTo0(PORTWxOUT_R, PINxOUT_R);
		PinTo1(PORTWxOUT_S, PINxOUT_S);
		PinTo0(PORTWxOUT_T, PINxOUT_T);
		//
		keyP1.counter = 0x0000;
		keyP1.sm0++;
	}
	else if (keyP1.sm0 == 1)
	{
		if (main_flag.f1ms)
		{
			if (++keyP1.counter >= P1_T1)//20)
			{
				PinTo0(PORTWxOUT_S, PINxOUT_S);
				keyP1.sm0++;
			}
		}
	}
	else if (keyP1.sm0 == 2)
	{
		if (main_flag.f1ms)
		{
			if (++keyP1.counter >= P1_T2)//80)
			{
				PinTo1(PORTWxOUT_R, PINxOUT_R);
				keyP1.sm0++;
			}
		}
	}
	else if (keyP1.sm0 == 3)
	{
		if (main_flag.f1ms)
		{
			if (++keyP1.counter >= P1_T3)//6000)
			{
				PinTo1(PORTWxOUT_T, PINxOUT_T);
				keyP1.sm0++;
			}
		}
	}
	else if (keyP1.sm0 == 4)
	{
		if (main_flag.f1ms)
		{
			if (++keyP1.counter >= P1_T4)//6020)
			{
				PinTo0(PORTWxOUT_T, PINxOUT_T);
				keyP1.sm0++;
			}
		}
	}
	else if (keyP1.sm0 == 5)
	{
		if (main_flag.f1ms)
		{
			if (++keyP1.counter >= P1_T5)//6120)
			{
				PinTo0(PORTWxOUT_R, PINxOUT_R);
				//
				keyP1.counter = 0x0000;
				keyP1.sm0 = 0x00;

				keyP1.f.job = 0;

				return 1;
			}
		}
	}
	return 0;
}

int8_t keyP2_job(void)
{
	if (keyP2.sm0 == 0)
	{
		PinTo1(PORTWxOUT_S, PINxOUT_S);
		//
		keyP2.counter = 0x0000;
		keyP2.sm0++;
	}
	else if (keyP2.sm0 == 1)
	{
		if (main_flag.f1ms)
		{
			if (++keyP2.counter >= KEY_TIMEPRESSING)	//20)//20ms
			{
				PinTo0(PORTWxOUT_S, PINxOUT_S);
				//
				keyP2.counter = 0x0000;
				keyP2.sm0 = 0x00;

				keyP2.f.job = 0;

				return 1;
			}
		}
	}
	return 0;
}

#define P2JOB_TOTALTIME KEY_TIMEPRESSING

#define X3_DELAY_SECONDS_FROM_P1_TO_P2 8//Seg

#define X3JOB_TOTALTIME_MS ( RELAY_TIMESWITCHING + P1JOB_TOTALTIME + (X3_DELAY_SECONDS_FROM_P1_TO_P2*1000) + P2JOB_TOTALTIME)//millisec

#define PROGRESSBAR_SLOTTIME ((int)( (X3JOB_TOTALTIME_MS*1.0f) / LCD_COL))



//
void lcdan_progressbarHor(int8_t coli, int8_t colf, int8_t row)
{
	for (int i=coli; i<colf; i++)
	{
		lcdan_set_cursor(i, row);
		lcdan_write_data(0xff); //caracter barra
	}
}
void progressBarJob(void)
{
	if (progressBar.f.job)//progress bar
	{
		if (main_flag.f1ms)
		{
			progressBar.counter++;
			if ( progressBar.counter % PROGRESSBAR_SLOTTIME == 0)
			{

				lcdan_progressbarHor(0, (int8_t)(((progressBar.counter*1.0f) /X3JOB_TOTALTIME_MS ) * LCD_COL ), 3);


				if (progressBar.counter >= X3JOB_TOTALTIME_MS)
				{
					progressBar.counter = 0x00;
					progressBar.f.job = 0;
				}
			}
		}
	}
}


int main(void)
{
//	uint16_t VoltBatt_buffer[SMOOTHALG_MAXSIZE];
//	float VoltBatt = 0;
//	uint16_t VoltGenerator_buffer[SMOOTHALG_MAXSIZE];
//	float VoltGenerator = 0;
	//
	char str[LCDAN_STR_MAXSIZE];
	//

	int8_t kb_counter=0;

	//
	lcdan_init();
	lcdan_bklight_off();
	ConfigOutputPin(CONFIGIOxLCD_BKLIGHT, PINxLCD_BKLIGHT);

	pinGetLevel_init(); //with Changed=flag activated at initialization

	//Keypad matricial 4x2
	//pullup
	PinTo1(PORTWxKBCOL_1, PINxKBCOL_1);
	PinTo1(PORTWxKBCOL_2, PINxKBCOL_2);
	PinTo1(PORTWxKBCOL_3, PINxKBCOL_3);
	PinTo1(PORTWxKBCOL_4, PINxKBCOL_4);
	__delay_ms(1);
	key_initialization();
	//
	PinTo0(PORTWxBUZZER, PINxBUZZER);
	ConfigOutputPin(CONFIGIOxBUZZER, PINxBUZZER);
	//
	//Outputs
	outputs_clear();

	ConfigOutputPin(CONFIGIOxOUT_Z, PINxOUT_Z);
	ConfigOutputPin(CONFIGIOxOUT_Y, PINxOUT_Y);
	ConfigOutputPin(CONFIGIOxOUT_X, PINxOUT_X);

	ConfigOutputPin(CONFIGIOxOUT_4, PINxOUT_4);
	ConfigOutputPin(CONFIGIOxOUT_3, PINxOUT_3);
	ConfigOutputPin(CONFIGIOxOUT_2, PINxOUT_2);
	ConfigOutputPin(CONFIGIOxOUT_1, PINxOUT_1);

	ConfigOutputPin(CONFIGIOxOUT_R, PINxOUT_R);
	ConfigOutputPin(CONFIGIOxOUT_S, PINxOUT_S);
	ConfigOutputPin(CONFIGIOxOUT_T, PINxOUT_T);

	//Active pull-up x tact switches P1, P2
	PinTo1(PORTWxTACTSW_P1, PINxTACTSW_P1);
	ConfigInputPin(CONFIGIOxTACTSW_P1, PINxTACTSW_P1);
	PinTo1(PORTWxTACTSW_P2, PINxTACTSW_P2);
	ConfigInputPin(CONFIGIOxTACTSW_P2, PINxTACTSW_P2);
	//

	//Config to 1ms
	TCNT0 = 0x00;
	TCCR0 = (1 << WGM01) | (0 << CS02) | (1 << CS01) | (1 << CS00); //CTC, PRES=64
	OCR0 = CTC_SET_OCR_BYTIME(1e-3, 64);//1ms Exacto @PRES=64
	//
	TIMSK |= (1 << OCIE0);
	sei();
	//
	ADC_init(ADC_MODE_SINGLE_END);

	//Determinar el tipo de bateria de acuerdo al voltaje
	while (!VoltBatt_get())//update VoltBatt
	{;}
	for (int i=0; i<BATTERYVOLTTYPE_NUMMAX; i++)
	{
		batteryVoltType = VoltBatt;	//Por si no se encuentra dentro del rango, entonces sera del
									//voltaje que se haya medido, caso contrario se sobreescribira
		if ( (VoltBatt >= (batteryVoltTypeArr[i]-BATTERY_KDIFF_FLOAT)) && (VoltBatt <= (batteryVoltTypeArr[i]+BATTERY_KDIFF_FLOAT)))
		{
			batteryVoltType = batteryVoltTypeArr[i];
			break;
		}
	}
	strcpy(str_batteryVoltType,"[");
	dtostrf(batteryVoltType, 0, 0, str);
	strcat(str_batteryVoltType, str);
	strcat(str_batteryVoltType, "V]");
	//


	while (1)
	{
		if (isr_flag.f1ms)
		{
			isr_flag.f1ms = 0;
			main_flag.f1ms = 1;
		}
		//----------------------
		if (main_flag.f1ms)
		{
			if (++kb_counter >= 20)//20ms acceso al keyboard
			{
				kb_counter = 0x00;

				kb_job();
				pinGetLevel_job();

				/*
				 * DESHABILITAR LOS FLAGS CORRESPONDIENTES AL CONMUTAR DE TECLA HACIA
				 * LOS PROCESOS EN BACKGROUND COMO LAS LECTURAS DE LOS ADCS
				 */
				if (kb_key_is_ready2read(KB_LYOUT_KEY_X1))
				{
					if (!keyX1.f.lock)//X3 lock X1
					{
						//ON/OFF System
						main_flag.X1onoff = !main_flag.X1onoff;

						if (main_flag.X1onoff == 1)
						{
							lcdan_bklight_on();
							keyX1.f.job = 1;

							buzzer.mode = BUZZERMODE_TACTSW;
							buzzer.f.job = 1;
						}
						else
						{
							lcdan_clear();
							lcdan_bklight_off();
							//
							outputs_clear();
							keyA = keyB = keyC = keyX2 = keyX3 = keyX4 = keyX5 = emptyJob;
							measVoltBatt = measVoltGenerador = emptyJob;
							smoothAlgJob = emptyJob;
							progressBar  = emptyJob;

							keyP1 = keyP2 = emptyJob;
							PinTo0(PORTWxBUZZER, PINxBUZZER);
							buzzer = emptyJob;

							main_flag.keysEnable = 0;
						}
					}
				}
				if ( (main_flag.X1onoff == 1) && (main_flag.keysEnable) )
				{
					//A, B, C solo se activan cuando X2 y X3 estan activos
					if (kb_key_is_ready2read(KB_LYOUT_KEY_A))
					{
						if (keyA.f.enable)
						{
							keyA.f.job = 1;

							//Off other background/process/flags
							//keyP1.f.enable = 0;
							//keyP2.f.enable = 0;
							//
							buzzer.mode = BUZZERMODE_TACTSW;
							buzzer.f.job = 1;
						}
					}
					if (kb_key_is_ready2read(KB_LYOUT_KEY_B))
					{
						if (keyB.f.enable)
						{
							if (keyB.f.lock == 0)
							{
								keyB.f.lock = 1;//B locked
								keyC.f.lock = 0;//C unlock
								//
								keyB.f.job = 1;

								//
								buzzer.mode = BUZZERMODE_TACTSW;
								buzzer.f.job = 1;
							}

							//Off other background/process/flags
							//keyP1.f.enable = 0;
							//keyP2.f.enable = 0;
							//
						}
					}
					if (kb_key_is_ready2read(KB_LYOUT_KEY_C))
					{
						if (keyC.f.enable)
						{
							if (keyC.f.lock == 0)
							{
								keyC.f.lock = 1;//B locked
								keyB.f.lock = 0;//C unlock
								//
								keyC.f.job = 1;

								//
								buzzer.mode = BUZZERMODE_TACTSW;
								buzzer.f.job = 1;
							}
							//Off other background/process/flags
							//keyP1.f.enable = 0;
							//keyP2.f.enable = 0;
							//
						}
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X2))
					{
						if (!keyX2.f.lock)
						{
							if (!keyX3.f.job)//mientras no acabe el proceso de X3
							{

								PinTo0(PORTWxOUT_1, PINxOUT_1);
								PinTo0(PORTWxOUT_2, PINxOUT_2);
								//
								keyX2.f.lock = 1;//bloqueado
								keyX3 = keyX4 = keyX5 = emptyJob;
								measVoltBatt = measVoltGenerador = emptyJob;
								smoothAlgJob = emptyJob;
								progressBar = emptyJob;
								//
								keyX2.f.job = 1;
								//

								//enable A,B,C + BOTON P1 Y P2
								keyP1.f.enable = 1;
								keyP2.f.enable = 1;
								keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;
								//

								buzzer.mode = BUZZERMODE_TACTSW;
								buzzer.sm0 = 0;
								buzzer.f.job = 1;
							}
						}
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X3))
					{
						if ( (!keyX3.f.job) && (!keyP1.f.job) )
						{
							if (keyX3.key_sm0 == 0)
							{
								keyX2 = keyX4 = keyX5 = emptyJob;
								measVoltBatt = measVoltGenerador = emptyJob;
								smoothAlgJob = emptyJob;
								//
								PinTo0(PORTWxOUT_1, PINxOUT_1);
								PinTo0(PORTWxOUT_2, PINxOUT_2);
								//

								keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//enable A,B,C
								//disable P1,P2.
								//X3 NO bloquea aun las otras teclas X2/X4/X5, espera al segundo toque
								keyP1.f.enable = 0;
								keyP2.f.enable = 0;
								//
								keyX3.key_sm0++;
								//
								buzzer.mode = BUZZERMODE_TACTSW;
								buzzer.sm0 = 0;
								buzzer.f.job = 1;
								//
								lcdan_print_Menu3();
							}
							else if (keyX3.key_sm0 == 1)
							{
								keyA.f.enable = keyB.f.enable = keyC.f.enable = 0;//Disable A,B,C
								//OJO: Mantiene los niveles actuales en que quedaron
								//
								//Bloquea X1,X2,X4 y X5 hasta que termine todo el job
								//-->uso !job para bloquear

								//keyX3.key_sm0 = 0x00; //Reingresa habilitando A,B,C
								//
								keyX3.f.job = 1;
								keyX3.sm0 = 0;
								//
								buzzer.mode = BUZZERMODE_X3_SEQUENCER;
								buzzer.sm0 = 0;
								buzzer.f.job = 1;
								//
							}
						}
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X4))
					{
						if ((!keyX4.f.lock) && (!keyP1.f.job) )
						{
							if (!keyX3.f.job)//mientras no acabe el proceso de X3
							{
								keyX2 = keyX3 = keyX5 = emptyJob;
								measVoltBatt = measVoltGenerador = emptyJob;
								smoothAlgJob = emptyJob;
								progressBar = emptyJob;
								//
								PinTo0(PORTWxOUT_1, PINxOUT_1);
								PinTo0(PORTWxOUT_2, PINxOUT_2);
								//
								keyX3 = emptyJob;//X3 Clear all flags/states

								keyX4.f.lock = 1;
								keyX4.f.job = 1;

								//Off other background/process/flags
								keyP1.f.enable = 0;
								keyP2.f.enable = 0;
								//
								keyA.f.enable = keyB.f.enable = keyC.f.enable = 0;//Disable A,B,C

								//
								buzzer.mode = BUZZERMODE_TACTSW;
								buzzer.f.job = 1;
							}
						}
					}
					if (kb_key_is_ready2read(KB_LYOUT_KEY_X5))//X5 es el unico que puede volver a resetearse el mismo
					{
						if ( (!keyX3.f.job) && (!keyP1.f.job) )//mientras no acabe el proceso de X3
						{
							keyX2 = keyX3 = keyX4 = emptyJob;
							measVoltBatt = measVoltGenerador = emptyJob;
							smoothAlgJob = emptyJob;
							progressBar = emptyJob;

							//
							PinTo0(PORTWxOUT_1, PINxOUT_1);
							PinTo0(PORTWxOUT_2, PINxOUT_2);
							//
							keyX5.f.job = 1;

							//enable A,B,C + BOTON P1 Y P2
							keyP1.f.enable = 1;
							keyP2.f.enable = 1;
							keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;
							//
							//
							buzzer.mode = BUZZERMODE_TACTSW;
							buzzer.f.job = 1;
						}
					}

					// Test P1, P2
					//pinGetLevel_job();
					//---------------------------
					if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_P1))
					{
						if (keyP1.f.enable)
						{
							if (!keyP1.f.lock)
							{
								if (pinGetLevel_level(PGLEVEL_LYOUT_P1)== 0)	//active in low
								{
									keyP1.f.lock = 1;
									keyP2.f.lock = 1;

									keyA.f.enable = keyB.f.enable = keyC.f.enable = 0;//Disable A,B,C
									//
									keyP1.f.job = 1;

									//
									buzzer.mode = BUZZERMODE_X3_SEQUENCER;
									buzzer.sm0 = 0;
									buzzer.f.job = 1;
									//
								}
							}
						}
						//pinGetLevel_clearChange(PGLEVEL_LYOUT_P1);//-->clear flush in while-end
					}
					//---------------------------
					if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_P2))
					{
						if (keyP2.f.enable)
						{
							if (!keyP2.f.lock)
							{
								if (pinGetLevel_level(PGLEVEL_LYOUT_P2)== 0)	//active in low
								{
									keyP1.f.lock = 1;
									keyP2.f.lock = 1;
									//
									keyA.f.enable = keyB.f.enable = keyC.f.enable = 0;//Disable A,B,C
									//
									keyP2.f.job = 1;
									//
									buzzer.mode = BUZZERMODE_TACTSW;
									buzzer.f.job = 1;
								}
							}

						}
						//pinGetLevel_clearChange(PGLEVEL_LYOUT_P2);//-->clear flush in while-end
					}
					//
				}
			}
		}
		//++++++
		//progressBarJob();
		//++++++++


		if (main_flag.X1onoff == 1)
		{
			//On system
			if (keyX1.f.job == 1)
			{
				if (keyX1.sm0 == 0)
				{
					//PinTo1(PORTWxOUT_1, PINxOUT_1);
					measVoltGenerador.f.job = 0;//kill meas.volt
					lcdan_print_Introduction();	//x5seg
					//
					keyX1.counter = 0x00;
					keyX1.sm0++;
				}
				else if (keyX1.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyX1.counter >= 1000*MENU_INTRODUCTION_AFTERDELAY_SEG)//5s
						{
							keyX1.counter = 0x00;

							PinTo1(PORTWxOUT_1, PINxOUT_1);
							lcdan_print_Menu1();

							keyX1.sm0 = 0;
							keyX1.f.job = 0;

							//desencadena en leer voltaje de bateria 12-24V nominal
							measVoltBatt.f.job = 1;
							measVoltBatt.counter = 0;
							//
							smoothAlgJob = emptyJob;
							//Add
							main_flag.keysEnable = 1;
						}
					}
				}
			}


			//Activa OUT X 1 pulso = 20ms
			if (keyA.f.job)
			{
				if (keyA.sm0 == 0)
				{
					PinTo1(PORTWxOUT_X, PINxOUT_X);
					keyA.counter = 0;
					keyA.sm0++;
				}
				else if (keyA.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyA.counter >= KEY_TIMEPRESSING) //20)//20ms
						{
							PinTo0(PORTWxOUT_X, PINxOUT_X);
							keyA.counter = 0;
							keyA.sm0 = 0;
							keyA.f.job = 0;
						}
					}
				}
			}
			//Y=on after 40ms, Z=off
			if (keyB.f.job)
			{
				if (keyB.sm0 == 0)
				{
					PinTo0(PORTWxOUT_Z, PINxOUT_Z);
					keyB.counter = 0;
					keyB.sm0++;
				}
				else if (keyB.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyB.counter >= RELAY_TIMESWITCHING) //40)//40ms
						{
							PinTo1(PORTWxOUT_Y, PINxOUT_Y);
							keyB.counter = 0;
							keyB.sm0 = 0;
							keyB.f.job = 0;
						}
					}
				}
			}
			//
			//Z=on after +40ms, Y=off
			if (keyC.f.job)
			{
				if (keyC.sm0 == 0)
				{
					PinTo0(PORTWxOUT_Y, PINxOUT_Y);
					keyC.counter = 0;
					keyC.sm0++;
				}
				else if (keyC.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyC.counter >= RELAY_TIMESWITCHING)	//40)//40ms
						{
							PinTo1(PORTWxOUT_Z, PINxOUT_Z);
							keyC.counter = 0;
							keyC.sm0 = 0;
							keyC.f.job = 0;
						}
					}
				}
			}
			//
			if (keyX2.f.job)
			{
				if (keyX2.sm0 == 0)
				{
					PinTo1(PORTWxOUT_2, PINxOUT_2);
					lcdan_print_Menu2();

					keyX2.f.job = 0;

					measVoltBatt.f.job = 0;	//kill meas.volt
					measVoltGenerador.f.job = 1;
					measVoltGenerador.counter = 0x00;
					//
					smoothAlgJob = emptyJob;

					//Add new: run KeyB sequence
					keyB.f.lock = 1;//B locked
					keyC.f.lock = 0;//C unlock
					//
					keyB.f.job = 1;
					//

				}
			}
			//
			if (keyX3.f.job)
			{
				if (keyX3.sm0 == 0)
				{
					keyX1.f.lock = 1;
					//EJEC #1
					lcdan_print_Menu3();//--> Se muestra en el primer toque
										//pero vuelvo a imprimir para que limpie el progressBar
					//
					progressBar.f.job = 1;//progress bar

					PinTo1(PORTWxOUT_4, PINxOUT_4);

					//kill both
					measVoltBatt.f.job = 0;	//kill meas.volt
					measVoltGenerador.f.job = 0;//kill meas.volt

					smoothAlgJob = emptyJob;

					//
					keyX3.counter = 0x00;
					keyX3.sm0++;
				}
				else if (keyX3.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyX3.counter >= RELAY_TIMESWITCHING)//	40)//40ms
						{
							keyX3.counter = 0x00;
							PinTo1(PORTWxOUT_3, PINxOUT_3);
							//
							keyX3.sm0++;
						}
					}
				}
				else if (keyX3.sm0 == 2)
				{
					//EJEC #2
					if (keyP1_job())
					{
						keyX3.sm0++;
					}
				}
				else if (keyX3.sm0 == 3)
				{
					if (main_flag.f1ms)
					{
						//delay 8s + EJEC #3
						if (++keyX3.counter >= 1000*X3_DELAY_SECONDS_FROM_P1_TO_P2)
						{
							keyX3.counter = 0x0000;
							keyX3.sm0++;
						}
					}
				}
				else if (keyX3.sm0 == 4)
				{
					if (keyP2_job())
					{
						keyX3.f.job = 0;	//finish sequence

						keyX3.sm0 = 0;
						//
						PinTo0(PORTWxOUT_3, PINxOUT_3);
						PinTo0(PORTWxOUT_4, PINxOUT_4);
						//
						PinTo0(PORTWxBUZZER, PINxBUZZER);
						buzzer = emptyJob;

						//+ADD
						keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//enable A,B,C

						//ADD
						keyX1.f.lock = 0;

					}
				}
			}
			//
			if (keyX4.f.job)
			{
				if (keyX4.sm0 == 0)
				{
					lcdan_print_Menu1();
					PinTo1(PORTWxOUT_1, PINxOUT_1);
					//
					measVoltGenerador.f.job = 0;//kill meas.volt
					measVoltBatt.f.job = 1;
					measVoltBatt.counter = 0x0;

					smoothAlgJob = emptyJob;
					//
					keyX4.f.job = 0;
				}
			}
			//
			if (keyX5.f.job)
			{
				if (keyX5.sm0 == 0)
				{
					main_flag.keysEnable = 0;

					measVoltBatt.f.job = 0;
					lcdan_print_Introduction();	//x5seg

					//
					measVoltGenerador.f.job = 0;
					measVoltGenerador.counter = 0x00;
					//
					//
					keyX5.counter = 0x00;
					keyX5.sm0++;
				}
				else if (keyX5.sm0 == 1)
				{
					if (main_flag.f1ms)
					{
						if (++keyX5.counter >= 1000*MENU_INTRODUCTION_AFTERDELAY_SEG)//5s
						{
							main_flag.keysEnable = 1;

							lcdan_print_Menu2();
							PinTo1(PORTWxOUT_2, PINxOUT_2);
							//
							keyX5.counter = 0x00;
							keyX5.sm0 = 0;
							keyX5.f.job = 0;

							//desencadena en leer voltaje del generador
							measVoltGenerador.f.job = 1;
							measVoltGenerador.counter = 0x00;
							//
							smoothAlgJob = emptyJob;

							//Add new: run KeyB sequence
							keyB.f.lock = 1;//B locked
							keyC.f.lock = 0;//C unlock
							//
							keyB.f.job = 1;
							//
						}
					}
				}
			}

			/*P1 P2 Secuenciadores
			 *
			 *Una vez que sus jobs=1, terminan por si solos,
			 *aun asi cambiando de tecla
			 */
			/***************************
			 * SERIA BUENO SEPARAR KEYP1 DE OTRO SECUENCERxP1 Y SECUENCERxP2
			 **************************/
			if (keyP1.f.job)
			{
				if (keyP1_job())
				{
					keyP1.f.job = 0;
					//
					//keyP1.f.lock = 0;//P1 queda bloqueada y P2 lo desbloquea y viceversa
					keyP2.f.lock = 0;

					PinTo0(PORTWxBUZZER, PINxBUZZER);
					buzzer = emptyJob;
					//
					keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//Enable A,B,C
				}
			}
			if (keyP2.f.job)
			{
				if (keyP2_job())
				{
					keyP2.f.job = 0;
					//
					keyP1.f.lock = 0;//unlock P1
					//keyP2.f.lock = 0;
					//
					keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//Enable A,B,C
				}
			}

			//ADC Battery 12-24Vdc nomimal, 30Vdc MAX
			if (measVoltBatt.f.job)
			{
				if (VoltBatt_get())
				{
					//print VoltBatt
					dtostrf(VoltBatt, 0, 1, str);
					lcdan_set_cursor(9, 2);
					lcdan_print_string(str);
					//
					lcdan_set_cursor(0, 3);

					if ( VoltBatt <= BATTERYVOLT_MINIMUM_ALERT)
					{
						lcdan_str_lineformat_align_P(str, PSTR("RECARGAR BATERIA"), LCDAN_STR_FORMAT_ALIGN_CENTER);
						lcdan_print_string(str);
					}
					else
					{
						lcdan_str_lineformat_align_P(str, PSTR("BATERIA OK"), LCDAN_STR_FORMAT_ALIGN_CENTER);
						lcdan_print_string(str);
					}
				}
			}
			//
			//ADC Generator 1000Vdc
			if (measVoltGenerador.f.job)
			{
				VoltGenerator_buffer[measVoltGenerador.counter] = ADC_read(ADC_CH_0);

				if (++measVoltGenerador.counter >= SMOOTHALG_MAXSIZE)
				{
					measVoltGenerador.counter = 0x0000;
					smoothGen_ADCHL = smoothAlg(VoltGenerator_buffer);

					VoltGenerator = (smoothGen_ADCHL*ADC_AREF*(ADCH0_R1 + ADCH0_R2))/(ADC_TOP*ADCH0_R1);
					//
					//print VoltGenerator
					dtostrf(VoltGenerator, 0, 1, str);
					strcat(str, "V     ");
					lcdan_set_cursor(8, 3);
					lcdan_print_string(str);
				}
			}
			//Buzzer
			if (buzzer.f.job)
			{
				if (buzzer.mode == BUZZERMODE_TACTSW )
				{
					if (buzzer.sm0 == 0)
					{
						PinTo1(PORTWxBUZZER, PINxBUZZER);
						buzzer.counter = 0;
						buzzer.sm0++;
					}
					else if (buzzer.sm0 == 1)
					{
						if (main_flag.f1ms)
						{
							if (++buzzer.counter >= BUZZER_TACTSW_KTIME)
							{
								PinTo0(PORTWxBUZZER, PINxBUZZER);
								buzzer.counter = 0;
								buzzer.sm0 = 0x0;
								buzzer.f.job = 0;
							}
						}
					}
				}
				else if (buzzer.mode == BUZZERMODE_X3_SEQUENCER)
				{
					if (buzzer.sm0 == 0)
					{
						PinTo1(PORTWxBUZZER, PINxBUZZER);
						buzzer.counter = 0;
						buzzer.sm0++;
					}
					else if (buzzer.sm0 == 1)
					{
						if (main_flag.f1ms)
						{
							if (++buzzer.counter >= BUZZER_X3_KTIME )
							{
								PinToggle(PORTWxBUZZER, PINxBUZZER);
								buzzer.counter = 0;
								//buzzer.sm0 = 0x0;
								//buzzer.f.job = 0;
							}
						}
					}
				}
			}
			//
			progressBarJob();
			//

		}//if onoff

		//
		main_flag.f1ms = 0;
		//flush all key
		kb_flush();	//flush keypad 4x2
		pinGetLevel_clearChange(PGLEVEL_LYOUT_P1);	//flush P1, P2
		pinGetLevel_clearChange(PGLEVEL_LYOUT_P2);
	}//end while

	return 0;

}

ISR(TIMER0_COMP_vect)
{
	isr_flag.f1ms = 1;
}

//#define SMOOTHALG_MAXSIZE 10
//uint16_t smoothAlg_buffer[SMOOTHALG_MAXSIZE];
float smoothAlg(uint16_t *buffer)
{
	float average=0;
	int Pos;	//# de elementos > que la media
	int Neg;	//# de elementos > que la media
	float TD;	//Total Deviation
	float A;	//Correct answer

	//1- Calculate media
	average = 0;
	for (int i=0; i<SMOOTHALG_MAXSIZE; i++)
	{
		average +=buffer[i];
	}
	average /= SMOOTHALG_MAXSIZE;

	//2 - Find Pos and Neg + |Dtotal|
	Pos = 0;
	Neg = 0;
	TD = 0;
	for (int i=0; i<SMOOTHALG_MAXSIZE; i++)
	{
		if (buffer[i] > average)
		{
			Pos++;
			TD += (buffer[i]-average);//Find |Dtotal|
		}
		if (buffer[i] < average)
		{
			Neg++;
		}
	}
	//
	A = average + ( ( (Pos-Neg)*TD )/ (SMOOTHALG_MAXSIZE*SMOOTHALG_MAXSIZE));
	return A;
}

//

//#define SMOOTHALG_MAXSIZE 10
//uint16_t smoothAlg_buffer[SMOOTHALG_MAXSIZE];

/*
 * 1: job is finished
 *
 * buffer [IN]-> Puntero q contiene los datos almacenados
 * Answer [OUT] -> Puntero hacia dato de salida
 */
struct _job smoothAlgJob;
int8_t smoothAlg_nonblock(uint16_t *buffer, float *Answer)
{
	static float average=0;
	static int Pos;	//# de elementos > que la media
	static int Neg;	//# de elementos > que la media
	static float TD;	//Total Deviation
	//float A;	//Correct answer

	//1- Calculate media
	if (smoothAlgJob.sm0 == 0)
	{
		average = 0;
		smoothAlgJob.counter = 0x0;
		smoothAlgJob.sm0++;
	}
	if (smoothAlgJob.sm0 == 1)
	{
		average +=buffer[smoothAlgJob.counter];

		if (++smoothAlgJob.counter >= SMOOTHALG_MAXSIZE)
		{
			average /= SMOOTHALG_MAXSIZE;
			//
			Pos = 0;
			Neg = 0;
			TD = 0;
			smoothAlgJob.sm0++;
		}
	}
	//2 - Find Pos and Neg + |Dtotal|
	else if (smoothAlgJob.sm0 == 2)
	{
		if (buffer[smoothAlgJob.counter] > average)
		{
			Pos++;
			TD += (buffer[smoothAlgJob.counter]-average);//Find |Dtotal|
		}
		if (buffer[smoothAlgJob.counter] < average)
		{
			Neg++;
		}
		//
		if (++smoothAlgJob.counter >= SMOOTHALG_MAXSIZE)
		{
			smoothAlgJob.counter = 0;
			smoothAlgJob.sm0 = 0;
			//
			*Answer = average + ( ( (Pos-Neg)*TD )/ (SMOOTHALG_MAXSIZE*SMOOTHALG_MAXSIZE));
			return 1;
			//
		}
	}
	return 0;
}
