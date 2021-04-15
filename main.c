/*
 * main.c
 *
 *  Created on: Apr 13, 2021
 *      Author: jcaf
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
} isr_flag = { 0 };

struct _main_flag
{
	unsigned f1ms :1;
	unsigned X1onoff:1;

} main_flag = { 0 };

struct _job
{
	int8_t sm0;
	uint16_t counter;
	struct _job_f{
		unsigned enable:1;
		unsigned job:1;
		unsigned lock:1;
		unsigned __a:5;
	}f;
	int8_t mode;
};
//
#define BUZZERMODE_TACTSW 0
#define BUZZERMODE_X3_SEQUENCER 1

#define BUZZER_TACTSW_KTIME 100//100ms
#define BUZZER_X3_KTIME 300//300ms
//
static const struct _job emptyJob;
struct _job keyX1, keyA,keyB,keyC,keyX2,keyX3,keyX4,keyX5;
struct _job measVoltBatt, measVoltGenerador;
struct _job keyP1, keyP2;
struct _job buzzer;
//
float smoothAlg(uint16_t *buffer);
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

/*
 * return update VoltBatt
 */
int8_t VoltBatt_get(void)//Non-blocking
{
	VoltBatt_buffer[measVoltBatt.counter] = ADC_read(ADC_CH_1);
	if (++measVoltBatt.counter >= SMOOTHALG_MAXSIZE)
	{
		measVoltBatt.counter = 0x0000;
		VoltBatt = smoothAlg(VoltBatt_buffer);
		return 1;
	}
	return 0;
}

//---------------------------------------------------------------------------------------------
//-------- Definicion del usuario de tiempos de manera general, excepto la secuencia para P1
#define KEY_TIMEPRESSING 	20//ms Tiempo de pulsacion
#define RELAY_TIMESWITCHING 40//ms Tiempo de conmutacion de Relays
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
	lcdan_str_lineformat_align_P(str, PSTR("VOLTAJE:"), 0);
	lcdan_set_cursor(0, 1);	lcdan_print_string(str);
	lcdan_str_lineformat_align_P(str, PSTR("NIVEL"), 0);
	lcdan_set_cursor(0, 2);	lcdan_print_string(str);
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

int8_t keyP1_job(void)
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
			if (++keyP1.counter >= 20)
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
			if (++keyP1.counter >= 80)
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
			if (++keyP1.counter >= 6000)
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
			if (++keyP1.counter >= 6020)
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
			if (++keyP1.counter >= 6120)
			{
				PinTo0(PORTWxOUT_R, PINxOUT_R);
				//
				keyP1.counter = 0x0000;
				keyP1.sm0 = 0x00;
				//keyP1.f.job = 0;
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
	else if (keyP2.sm0 == 0)
	{
		if (main_flag.f1ms)
		{
			if (++keyP2.counter >= KEY_TIMEPRESSING)	//20)//20ms
			{
				PinTo0(PORTWxOUT_S, PINxOUT_S);
				//
				keyP2.counter = 0x0000;
				keyP2.sm0 = 0x00;
				//keyP2.f.job = 0;
				return 1;
			}

		}

	}
	return 0;
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
	TCCR0 = (1 << WGM01) | (1 << CS02) | (0 << CS01) | (1 << CS00); //CTC, PRES=1024
	OCR0 = CTC_SET_OCR_BYTIME(1e-3, 1024);
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
	strcat(str_batteryVoltType, " V]");
	//

	//lcdan_print_Introduction();
	//lcdan_print_Menu1();
	//while (1);

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
						keyP1 = keyP2 = emptyJob;
						buzzer = emptyJob;
					}

				}
				if (main_flag.X1onoff == 1)
				{
					//Test el resto de teclas del teclado matricial 4x2
					/*
					 * A, B, C solo se activan cuando X2 y X3 estan activos
					 */
					if (kb_key_is_ready2read(KB_LYOUT_KEY_A))
					{
						if (keyA.f.enable)
						{
							keyA.f.job = 1;

							//Off other background/process/flags
							keyP1.f.enable = 0;
							keyP2.f.enable = 0;
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
							keyP1.f.enable = 0;
							keyP2.f.enable = 0;
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
							keyP1.f.enable = 0;
							keyP2.f.enable = 0;
							//
						}
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X2))
					{
						if (!keyX3.f.job)//mientras no acabe el proceso de X3
						{
							keyX2.f.job = 1;

							keyP1.f.enable = 1;//HABILITA BOTON P1 Y P2
							keyP2.f.enable = 1;
							//
							keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//enable A,B,C

							//
							buzzer.mode = BUZZERMODE_TACTSW;
							buzzer.f.job = 1;
						}
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X3))
					{
						//reinicia proceso cada vez que es pulsado o se bloquea hasta que termine todo el job ???
						//si pasa a otra tarea, 'esta termina en su propia hilo? o es terminada al conmutar
						keyX3.f.job = 1;

						//Off other background/process/flags
						keyP1.f.enable = 0;
						keyP2.f.enable = 0;
						//
						keyA.f.enable = keyB.f.enable = keyC.f.enable = 1;//enable A,B,C

						//
						buzzer.mode = BUZZERMODE_X3_SEQUENCER;
						buzzer.f.job = 1;
					}

					if (kb_key_is_ready2read(KB_LYOUT_KEY_X4))
					{
						if (!keyX3.f.job)//mientras no acabe el proceso de X3
						{
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
					if (kb_key_is_ready2read(KB_LYOUT_KEY_X5))
					{
						if (!keyX3.f.job)//mientras no acabe el proceso de X3
						{
							keyX5.f.job = 1;

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

					// Test P1, P2
					//pinGetLevel_job();
					//---------------------------
					if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_P1))
					{
						if (keyP1.f.enable)
						{
							if (pinGetLevel_level(PGLEVEL_LYOUT_P1)== 0)	//active in low
							{
								keyP1.f.job = 1;
							}
						}
						//pinGetLevel_clearChange(PGLEVEL_LYOUT_P1);//-->clear flush in while-end
					}
					//---------------------------
					if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_P2))
					{
						if (keyP2.f.enable)
						{
							if (pinGetLevel_level(PGLEVEL_LYOUT_P2)== 0)	//active in low
							{
								keyP1.f.job = 1;
							}
						}
						//pinGetLevel_clearChange(PGLEVEL_LYOUT_P2);//-->clear flush in while-end
					}
					//
				}
			}
		}

		if (main_flag.X1onoff == 1)
		{
			//On system
			if (keyX1.f.job)
			{
				if (keyX1.sm0 == 0)
				{
					PinTo1(PORTWxOUT_1, PINxOUT_1);
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
						if (++keyX1.counter >= 1000*5)//5s
						{
							lcdan_print_Menu1();
							keyX1.counter = 0x00;
							keyX1.sm0 = 0;
							keyX1.f.job = 0;

							//desencadena en leer voltaje de bateria 12-24V nominal
							measVoltBatt.f.job = 1;
							measVoltBatt.counter = 0;
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
				}
			}
			//
			if (keyX3.f.job)
			{
				if (keyX3.sm0 == 0)
				{
					lcdan_print_Menu3();
					//
					PinTo1(PORTWxOUT_4, PINxOUT_4);

					//kill both
					measVoltBatt.f.job = 0;	//kill meas.volt
					measVoltGenerador.f.job = 0;//kill meas.volt

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
					if (keyP1_job())
					{
						keyX3.sm0++;
					}
				}
				else if (keyX3.sm0 == 3)
				{
					if (main_flag.f1ms)
					{
						if (++keyX3.counter >= 1000*8)
						{
							keyX3.counter = 0x0000;
						}
					}
				}
				else if (keyX3.sm0 == 4)
				{
					if (keyP2_job())
					{
						keyX3.f.job = 0;	//finish sequence
						buzzer = emptyJob;
					}
				}
			}
			//
			if (keyX4.f.job)
			{
				if (keyX4.sm0 == 0)
				{
					lcdan_print_Menu1();
					//
					measVoltGenerador.f.job = 0;//kill meas.volt
					measVoltBatt.f.job = 1;
					measVoltBatt.counter = 0x0;
					//
					keyX4.f.job = 0;
				}
			}
			//
			if (keyX5.sm0 == 0)
			{
				measVoltBatt.f.job = 0;
				lcdan_print_Introduction();	//x5seg
				//
				keyX5.counter = 0x00;
				keyX5.sm0++;
			}
			else if (keyX5.sm0 == 1)
			{
				if (main_flag.f1ms)
				{
					if (++keyX5.counter >= 1000*5)//5s
					{
						lcdan_print_Menu2();
						keyX5.counter = 0x00;
						keyX5.sm0 = 0;
						keyX5.f.job = 0;

						//desencadena en leer voltaje del generador
						measVoltGenerador.f.job = 1;
						measVoltGenerador.counter = 0x00;
					}
				}
			}

			/*P1 P2 Secuenciadores
			 *
			 *Una vez que sus jobs=1, terminan por si solos,
			 *aun asi cambiando de tecla
			 */
			if (keyP1.f.job)
			{
				if (keyP1_job())
				{
					keyP1.f.job = 0;
				}
			}
			if (keyP2.f.job)
			{
				if (keyP2_job())
				{
					keyP2.f.job = 0;
				}
			}

			//ADC Battery 12-24Vdc nomimal, 30Vdc MAX
			if (measVoltBatt.f.job)
			{
				if (VoltBatt_get())
				{
					//print VoltBatt
					dtostrf(VoltBatt, 0, 1, str);
					lcdan_set_cursor(9, 1); lcdan_print_string(str);
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
					VoltGenerator = smoothAlg(VoltGenerator_buffer);
					//
					//print VoltGenerator
					dtostrf(VoltGenerator, 0, 1, str);
					strcat(str, " V      ");
					lcdan_set_cursor(14, 3); lcdan_print_string(str);
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
								buzzer.sm0 = 0x0;
							}
						}
					}
				}
			}

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
