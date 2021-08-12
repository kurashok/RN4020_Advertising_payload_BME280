/*
 *
 * Created: 2020/05/20 11:07:32
 */ 
#define F_CPU 1000000

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include "iic.h"
#include "adc.h"
#include "eeprom.h"

uint16_t  uiTimer_0 ;
// 温度測定間隔(200ms単位)
#define MES_INTERVAL 28
// 温度変換時間(240ms以上)
#define CONVERSION_WAIT 2

// 測定インターバル中か、測定器の変換終了待ちかを示すフラグ
uint8_t bMes_cycle;

#define WDT_8s 9
#define WDT_4s 8
#define WDT_2s 7
#define WDT_1s 6
#define WDT_500ms 5
#define WDT_250ms 4
#define WDT_125ms 3

void setup_WDT( uint8_t delay )
{
	// ウォッチドッグタイマのタイムアウト設定
	if( delay > 9 ) delay = 9;
	uint8_t reg_data = delay & 7;
	if( delay & 0x8 )
	{
		reg_data |= 0x20;
	}

	MCUSR &= ~(1 << WDRF);
	//WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	//WDTCSR = reg_data | 1<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDCE) | (1 << WDE); // 設定変更許可
	WDTCSR = reg_data | 0<<WDE;	// タイムアウト設定
	WDTCSR |= (1 << WDIE); // 割り込み許可
}

//#define BAUD_PRESCALE 51 // U2X=0 8MHz 9600B
//#define BAUD_PRESCALE 12 // U2X=0 2MHz 9600B
#define BAUD_PRESCALE 12 // U2X=1 1MHz 9600B

volatile uint8_t bRecieve;
char *waiting_message;

void setup_USART(void){
	// Set baud rate
	UBRR0L = BAUD_PRESCALE;// Load lower 8-bits into the low byte of the UBRR register
	UBRR0H = (BAUD_PRESCALE >> 8);

	// 倍速ビットON
	UCSR0A = 2;
	// Enable receiver and transmitter and receive complete interrupt
	UCSR0B = ((1<<TXEN0)|(1<<RXEN0) | (1<<RXCIE0));
	UCSR0C = 0b00000110;
}

void USART_SendByte(uint8_t u8Data){

	// Wait until last byte has been transmitted
	while((UCSR0A &(1<<UDRE0)) == 0);

	// Transmit data
	UDR0 = u8Data;
	
	// clear tx complete flag
	UCSR0A |= (1<<TXC0);
}

void USART_SendStr(const char * str)
{
	uint8_t i = 0;
	while( str[i] != '\0' )
	{
		USART_SendByte(str[i++]);
	}
	
	// wait for tx complete
	while( (UCSR0A & (1<<TXC0)) == 0 );
}

#define RCV_SIZE 100
char RCV_BUF[RCV_SIZE];
uint8_t RCV_PTR = 0;

void check_command()
{
	if( strstr(RCV_BUF, waiting_message) != NULL )
	{
		bRecieve = 1;
	}
}

ISR (USART_RX_vect)
{
	char value = UDR0;             //read UART register into value
	if( value == '\x0d' )
	{
	}
	else if( value == '\x0a' )
	{
		RCV_BUF[RCV_PTR] = '\0';
		check_command();
		RCV_PTR = 0;
	}
	else
	{
		RCV_BUF[RCV_PTR] = value;
		if( RCV_PTR < RCV_SIZE-1 )
		{
			RCV_PTR++;
		}
	}
}

int write_reg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
	iic_start();
	int ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 10;
	}

	ack = iic_send(reg_addr);
	if( ack != 0 )
	{
		return 11;
	}

	ack = iic_send(data);
	if( ack != 0 )
	{
		return 12;
	}

	iic_stop();
	return 0;
}

void temp_to_string( double temp, char *buf )
{
	double t = temp + 0.05; // 小数第２位で四捨五入
	sprintf( buf, "%d.%01d", (int)t, abs((int)((t-(int)t)*10)) );
}

void disable_rxint()
{
	UCSR0B &= ~(1<<RXCIE0);
}

void enable_rxint()
{
	UCSR0B |= (1<<RXCIE0);
}

void send_RN4020_command( const char *cmd, const char *ack )
{
	if( cmd != NULL )
	{
		USART_SendStr(cmd);
	}
	if( ack != NULL )
	{
		waiting_message = (char*)ack;
		bRecieve = 0;
		while( bRecieve == 0 );
	}
}

void check_connection(const char *buf)
{
	static int prv_con = 0;
	if( PINB & 0x4 )
	{
		PORTD |= 0x10; // connect LED on
		PORTB |= 1; // operation mode
		PORTB |= 2; // MLDP mode
		_delay_ms(5);
		USART_SendStr( buf );
		//PORTB &= ~2; // CMD mode
		PORTB &= ~1; // sleep mode
		prv_con = 1;
	}
	else
	{
		PORTD &= ~0x10; // connect LED off
		PORTB |= 1; // operation mode

		if( prv_con == 1 )
		{
			PORTB &= ~2; // CMD mode
		}
		_delay_ms(5);
		send_RN4020_command("A,0064,03e8\x0d\x0a", NULL);
		//_delay_ms(100);
		PORTB &= ~1; // sleep mode

		prv_con = 0;
	}
}

void make_info_mes(int t, int p, int h, int v, char *buf)
{
	int pol = 0;
	if( t < 0 ) 
	{
		pol = 1;
		t *= -1;
	}

	uint16_t t_hd = t%10;
	t /= 10;
	t_hd |= (t%10)<<4;
	t /= 10;
	t_hd |= (t%10)<<8;
	t /= 10;
	t_hd |= t<<12;

	uint16_t p_hd = p%10;
	p /= 10;
	p_hd |= (p%10)<<4;
	p /= 10;
	p_hd |= (p%10)<<8;
	p /= 10;
	p_hd |= p<<12;

	uint16_t h_hd = h%10;
	h /= 10;
	h_hd |= (h%10)<<4;
	h /= 10;
	h_hd |= (h%10)<<8;
	h /= 10;
	h_hd |= h<<12;

	uint16_t v_hd = v%10;
	v /= 10;
	v_hd |= (v%10)<<4;
	v /= 10;
	v_hd |= (v%10)<<8;
	v /= 10;
	v_hd |= v<<12;

	int err = 0;
	sprintf( buf, "N,9999%04x%02x%04x%04x%04x%04x\x0d\x0a", err, pol, t_hd, v_hd, h_hd, p_hd );
}

void send_advertise(char *buf)
{
	PORTB |= 1; // operation mode
	PORTB &= ~2; // CMD mode

	_delay_ms(5);
	send_RN4020_command(buf, NULL);
	send_RN4020_command("A,0063,00c8\x0d\x0a", NULL);
//	send_RN4020_command("A,0064,03e8\x0d\x0a", NULL);

	PORTB &= ~1; // sleep mode
}

void setup_RN4020()
{
	// B0 : output : WAKE_SW
	DDRB |= 1;
	PORTB |= 1; // WAKE_SW = 1 operation mode

	// B1 : output : CMD_MLDP
	DDRB |= 2;
	PORTB &= ~2; // CMD_MLDP = 0 cmd mode

	// B2 : input : connect indicator
	DDRB &= ~4;

	// C0 : output : status led
	DDRD |= 0x10;
	PORTD |= 0x10; // led = 1

	PORTD |= 1; // RXD端子プルアップ

	_delay_ms(5000);
	send_RN4020_command("SS,00000007\x0d\x0a", NULL);
	send_RN4020_command("SR,12000000\x0d\x0a", NULL);
	send_RN4020_command("R,1\x0d\x0a", NULL);

	PORTD &= ~0x10; // led = 0
}

ISR (WDT_vect)
{
}

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
uint8_t dig_H1;
int16_t dig_H2;
uint8_t dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t dig_H6;
uint8_t config;
uint8_t ctrl_meas;
uint8_t osrs_h;

int set_one_shot()
{
	// set force mode
	write_reg(0x76,0xf4,ctrl_meas);

	return 0;
}

int setup_iic_read( uint8_t dev_addr, uint8_t reg_addr)
{
	int ack;

	iic_start();
	ack = iic_send(dev_addr<<1);
	if( ack != 0 )
	{
		return 1; // ack error
	}

	ack = iic_send(reg_addr);
	if( ack != 0 )
	{
		return 2; // ack error
	}
		
	iic_start();

	ack = iic_send(dev_addr<<1 | 1);
	if( ack != 0 )
	{
		return 3; // ack error
	}
	return 0;
}

void read_serise_addr(uint8_t dev_addr, uint8_t reg_addr, uint8_t count, uint8_t *d)
{
	setup_iic_read(dev_addr, reg_addr);
	for(uint8_t i=0; i<count-1; i++)
	{
		d[i] = iic_recv(0);
	}
	d[count-1] = iic_recv(1);
	iic_stop();
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t BME280_compensate_T_int32(int32_t adc_T)
{
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1))) >> 12) *
	((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

// Returns pressure in Pa as unsigned 32 bit integer. Output value of “96386” equals 96386 Pa = 963.86 hPa
uint32_t BME280_compensate_P_int32(int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;
	var1 = (((int32_t)t_fine)>>1) - (int32_t)64000;
	var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((int32_t)dig_P6);
	var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
	var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
	var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((int32_t)dig_P2) * var1)>>1))>>18;
	var1 =((((32768+var1))*((int32_t)dig_P1))>>15);
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = (((uint32_t)(((int32_t)1048576)-adc_P)-(var2>>12)))*3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dig_P9) * ((int32_t)(((p>>3) * (p>>3))>>13)))>>12;
	var2 = (((int32_t)(p>>2)) * ((int32_t)dig_P8))>>13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dig_P7) >> 4));
	return p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t bme280_compensate_H_int32(int32_t adc_H)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) * v_x1_u32r)) +
	((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r *
	((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
	((int32_t)dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

int read_val( int *t, int *p, int *h )
{
	uint8_t d[8];
	
	read_serise_addr(0x76,0xf7,8,d);
	
	int32_t adc_p = ((uint32_t)d[0] << 12) | ((uint32_t)d[1] << 4) | (d[2] >> 4);
	int32_t adc_t = ((uint32_t)d[3] << 12) | ((uint32_t)d[4] << 4) | (d[5] >> 4);
	int32_t adc_h = ((uint32_t)d[6] << 8) | d[7];

	int32_t val_t = BME280_compensate_T_int32(adc_t);
	uint32_t val_p = BME280_compensate_P_int32(adc_p);
	uint32_t val_h = bme280_compensate_H_int32(adc_h);
	
	// 32bitを16bitに丸める
	// 気温：単純にビットを削る。もともと0.01度単位
	*t = (int)val_t;
	// 湿度：1024で割って100をかけて0.01%単位にする
	*h = (unsigned int)(val_h / 10.24);
	// 気圧：100で割ってヘクトパスカル単位にする
	*p = (val_p+50)/100;

	return 0;
}

void setup_BME280()
{
	uint8_t d[32];

	read_serise_addr(0x76,0x88,24,d);
	read_serise_addr(0x76,0xa1,1,d+24);
	read_serise_addr(0x76,0xe1,7,d+25);

	//char buf[40];	
	//for(int i=0; i<32; i++ )
	//{
	//	sprintf( buf, "trim(%d)= 0x%x\x0d\x0a", i, d[i]);
	//	USART_SendStr(buf);
	//}
	
	dig_T1 = (d[1] << 8) | d[0];
	dig_T2 = (d[3] << 8) | d[2];
	dig_T3 = (d[5] << 8) | d[4];
	dig_P1 = (d[7] << 8) | d[6];
	dig_P2 = (d[9] << 8) | d[8];
	dig_P3 = (d[11] << 8) | d[10];
	dig_P4 = (d[13] << 8) | d[12];
	dig_P5 = (d[15] << 8) | d[14];
	dig_P6 = (d[17] << 8) | d[16];
	dig_P7 = (d[19] << 8) | d[18];
	dig_P8 = (d[21] << 8) | d[20];
	dig_P9 = (d[23] << 8) | d[22];
	dig_H1 = d[24];
	dig_H2 = (d[26] << 8) | d[25];
	dig_H3 = d[27];
	dig_H4 = (d[28] << 4) | (d[29] & 0xf);
	dig_H5 = (d[29] >> 4) | (d[30] << 4);
	dig_H6 = d[31];

	config = 0;
	uint8_t p_meas = 1;
	uint8_t t_meas = 1;
	uint8_t mode = 1;
	ctrl_meas = (p_meas << 5) | (t_meas << 2) | mode;
	osrs_h = 1;
	
	write_reg(0x76,0xf5,config);
	write_reg(0x76,0xf2,osrs_h);
}

char mes_buf[100];

void meas()
{
	uiTimer_0--;
	if( uiTimer_0 == 0 )
	{
		if( bMes_cycle == 0 )
		{
			set_one_shot();
			//if( err != 0 )
			//{
			//	sprintf( buf, "ERROR=%d\r\n", err );
			//	USART_SendStr( buf );
			//	uiTimer_0 = MES_INTERVAL;
			//}
			//else
			{
				bMes_cycle = 1;
				uiTimer_0 = CONVERSION_WAIT;
			}
		}
		else
		{
			uiTimer_0 = MES_INTERVAL;
			bMes_cycle = 0;

			int t, p, h;
			read_val(&t, &p, &h);
			enable_adc();
			setup_adc(0);
			uint16_t val = exec_adc();
			int vol = (int)(((double)val * (3.3/1024) +0.005) * 100);
			make_info_mes(t, p, h, vol, mes_buf);
		}
	}
	send_advertise(mes_buf);
}

int main(void)
{
	uint8_t cal = read_eeprom(0);
	if(cal != 0xff)
	{
		OSCCAL = cal;
	}
	CLKPR = 0x80;// クロック分周比変更許可ビットON
	CLKPR = 0x03;// 分周比1/8 : クロックは1MHz

	setup_USART();
	disable_rxint();
	DIDR0 = 0xf; // PC0,1,2,3はデジタル入力禁止。ADC入力専用
	PRR |= (1<<PRTWI) | (1<<PRTIM0) | (1<<PRTIM1) | (1<<PRTIM2);

	sei();			// 割込みを許可する。

	setup_RN4020();
	setup_iic();
	setup_BME280();

	bMes_cycle = 0;

	uiTimer_0 = 1;
	sprintf( mes_buf, "N,999900000000000000\x0d\x0a" );
	while (1)
	{
		setup_WDT(WDT_2s);
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_enable();
		// BOD禁止処理
		MCUCR |= (1<<BODSE) | (1<< BODS);
		MCUCR = (MCUCR & ~(1 << BODSE))|(1 << BODS);
		sleep_cpu();
		sleep_disable();
		//sleep_mode();
		meas();
		disable_adc();
	}
}
