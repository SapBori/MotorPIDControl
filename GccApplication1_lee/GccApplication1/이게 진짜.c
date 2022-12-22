/*
 * GccApplication1.c
 *
 * Created: 2019-08-28 오후 12:13:16
 * Author : CDSL
 */ 

#include "mcu_init.h"
#include "dataType.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////


volatile int32_t g_Cnt, g_preCnt;

volatile double g_Pdes, g_Ppre;
volatile double g_Pcur, g_Pvcur;
volatile double g_Perr;

volatile double g_Vcur;
volatile double g_Vdes;
volatile double g_Verr;
volatile double g_Vlimit = 1.;
volatile double g_Verr_sum;

volatile double g_Ccur;
volatile double g_Cdes;
volatile double g_Cerr;
volatile double g_Cerr_sum;
volatile double g_Climit = 1.;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control = 0;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;
// Encoder Position


void SetDutyCW(double v){
	
	while(TCNT1  == 0);

	int ocr = v * (200. / 24.) + 200;
	
	if(ocr > OCR_MAX)	ocr = OCR_MAX;
	else if(ocr < OCR_MIN)	ocr = OCR_MIN;
	//OCR1A = OCR1B = ocr;
	
	OCR1A = OCR3B = ocr + 8;		//1 H
	OCR1B = OCR3A = ocr - 8;		//1 L
}


void InitLS7366(){
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR0 | WR_REG);
	SPI_MasterSend(X4_QUAD | FREE_RUN | DISABLE_INDEX | SYNCHRONOUS_INDEX |FILTER_CDF_1);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_MDR1 | WR_REG);
	SPI_MasterSend(FOUR_BYTE_COUNT_MODE | ENABLE_COUNTING);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_CNTR | CLR_REG);
	PORTB = 0x01;
}



int getADC(char ch){

	ADMUX = (ADMUX & 0xf0) + ch;
	ADCSRA |= 0x40;
	while(!(ADCSRA & 0x10));
	return ADC;
}




ISR(USART0_RX_vect){

	g_buf[g_BufWriteCnt++] = UDR0;
}



ISR(TIMER0_OVF_vect){
	/*
	double La = 0.658/1000;
	double Ra = 1.76;
	double Kt = 68.3/1000;
	double J = 99.5/1000/1000000;
	double b = J/0.00376;

	double control_wave = 2000;
	double Cp_gain = control_wave*La;
	double Ci_gain = control_wave*Ra;
	double Cd_gain = control_wave;
	double Ca_gain = 1/Cp_gain;
	double wcs = control_wave/10;
	double Vp_gain = wcs*(J/Kt);
	double Vi_gain = wcs*(b/Kt);
	double Va_gain = 1/Vp_gain;
	double wcp = wcs/10;
		
	TCNT0 = 256 - 125;
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0);
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);
	PORTB = 0x01;
			
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
	PORTC = 0x03; 
	
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;
	// PID 제어 넣는 곳
	// MCH 16_000_000
	
	//TO DO
	if((g_TimerCnt % 100) == 0){
		g_Perr = g_Pdes- g_Pcur;
		double Pp_gain = wcp;
		double Pd_gain = wcp/wcs;
		double Pa_gain = 1/Pp_gain;
		
		Pp_gain = control_wave;
		Pd_gain = control_wave/wcs;
		g_pos_control = g_Perr*Pp_gain+ ((g_Perr-g_Ppre)/0.05)*Pd_gain;
		
		if(g_pos_control>642.6){
			g_pos_control = 642.6;
		}
		else if(g_pos_control< -642.6){
			g_pos_control = -642.6;
		}
		g_Ppre= g_Perr;
	}
	if((g_TimerCnt % 10) == 0){
		g_Vdes = g_pos_control;
		if(g_Vdes <-1.*(g_Vlimit)){
			g_Vdes =-1.*(g_Vlimit);}
		else if (g_Vdes > g_Vlimit){
			g_Vdes = g_Vlimit;}
		
		g_Vcur = (g_Pcur - g_Pvcur) /0.005;
		g_Verr = g_Vdes- g_Vcur;
		wcs = control_wave/10;
		Vp_gain = (J/Kt)*control_wave;
		Vi_gain = (b/Kt)*control_wave;
		Va_gain = 1/Vp_gain;
		
		g_vel_control = g_Verr*Vp_gain+g_Verr_sum*Vi_gain ;
		g_Verr_sum +=g_Verr;
		if(g_vel_control>2.08){
			g_Verr_sum -=(g_vel_control-2.08)*Va_gain;
			g_vel_control= 2.08;
		}
		else if(g_vel_control<-2.08){
			g_Verr_sum -=(g_vel_control+2.08)*Va_gain;
			g_vel_control = -2.08;
		}
		g_Pvcur= g_Pcur;
	}
	g_TimerCnt++;
	//Current 
	// Cdes= Current Desire
	// Cerr = Current Error
	g_Ccur = -(((g_ADC / 1024. * 5.) - 2.5) * 10.);
	g_Cdes = g_vel_control;
	if(g_Cdes <-1*g_Climit)
	g_Cdes= -1*g_Climit;
	else if (g_Cdes > g_Climit)
	g_Cdes = g_Climit;
	g_Cerr = g_Cdes - g_Ccur;
	cur_control = g_Cerr * Cp_gain+g_Cerr_sum*Ci_gain;
	cur_control += g_Vcur*0.0683;
	g_Cerr_sum +=g_Cerr;
	//g_Cerr = g_Cdes - g_Ccur;
	//g_Cerr_sum += g_Cerr;
	//I-term anti 
	if(cur_control > 24){
		cur_control = 24;
		g_Cerr_sum -= (cur_control - 24) * 1. / 0.0827 / 3.;
		
	}
	else if(cur_control < -24){
		cur_control = -24;
		g_Cerr_sum -= (cur_control + 24) * 1. / 0.0827 / 3.;
	}
	SetDutyCW(cur_control);*/
	TCNT0 = 256 - 125;
	//Read LS7366
	int32_t cnt;
	
	PORTC = 0x01;
	
	g_ADC = getADC(0);
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | LOAD_REG);
	PORTB = 0x01;
	
	PORTB = 0x00;
	SPI_MasterSend(SELECT_OTR | RD_REG);
	cnt = SPI_MasterRecv();		cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();	cnt = cnt<< 8;
	cnt |= SPI_MasterRecv();
	PORTB = 0x01;
	g_Cnt = -cnt;
	
	PORTC = 0x03;
	
	g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;
	
	//TO DO
	if((g_TimerCnt % 100) == 0){
		
		g_TimerCnt = 0;
		
	}
	if((g_TimerCnt % 10) == 0){
		
		g_Vcur = (g_Pcur - g_Pvcur) / 0.005;
		g_Pvcur = g_Pcur;
	}
	g_TimerCnt++;
	
	g_Cdes = -0.1;
	
	g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.5) * 10.);
	g_Cerr = g_Cdes - g_Ccur;

	cur_control = g_Cerr * 0.1 + g_Cerr_sum * 1.5;
	cur_control += g_Vcur * 0.0683;
	
	g_Cerr_sum += g_Cerr;
	
	//I-term anti
	if(cur_control > 24){
		cur_control = 24;
		g_Cerr_sum -= (cur_control - 24) * 1. / 0.0827 / 3.;
	}
	else if(cur_control < -24){
		cur_control = -24;
		g_Cerr_sum -= (cur_control + 24) * 1. / 0.0827 / 3.;
	}
	
	SetDutyCW(cur_control);
	
	/////////////////////////////////////////
	
	g_SendFlag++;

}



int main(void){
	
	Packet_t packet;
	packet.data.header[0] = packet.data.header[1] = packet.data.header[2] = packet.data.header[3] = 0xFE;
	
	InitIO();
	
	//Uart
	InitUart0();
	
	//SPI
	InitSPI();
	
	//Timer
	InitTimer0();
	InitTimer1();
	InitTimer3();


	TCNT1 = TCNT3 = 0;
	SetDutyCW(0.);
	
	//ADC
	InitADC();
	
	//LS7366
	InitLS7366();
	
	TCNT0 = 256 - 125;
	sei();

	unsigned char check = 0;
	
    while (1) {
		for(;g_BufReadCnt != g_BufWriteCnt; g_BufReadCnt++){
			
			switch(g_PacketMode){
			case 0:
				
				if (g_buf[g_BufReadCnt] == 0xFF) {
					checkSize++;
					if (checkSize == 4) {
						g_PacketMode = 1;
					}
				}
				else {
					checkSize = 0;
				}
				break;
				
			case 1:

				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				
				if (checkSize == 8) {
					if(g_PacketBuffer.data.id == g_ID){

						g_PacketMode = 2;
					}
					else{
						g_PacketMode = 0;
						checkSize = 0;
					}
				}

				break;
			
			case 2:
				
				g_PacketBuffer.buffer[checkSize++] = g_buf[g_BufReadCnt];
				check += g_buf[g_BufReadCnt];
				
				if (checkSize == g_PacketBuffer.data.size) {

					if(check == g_PacketBuffer.data.check){

						switch(g_PacketBuffer.data.mode){

							case 2:
							g_Pdes = g_PacketBuffer.data.pos*0.0174533 / 1000;
							g_Vlimit = g_PacketBuffer.data.velo*0.0174533/ 1000 ;
							g_Climit = g_PacketBuffer.data.cur / 1000;
							break;
							}
					}
					
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
				else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)) {
					TransUart0('f');
					check = 0;
					g_PacketMode = 0;
					checkSize = 0;
				}
			}
		}

		if(g_SendFlag > 19){
			g_SendFlag = 0;			

			packet.data.id = g_ID;
			packet.data.size = sizeof(Packet_data_t);
			packet.data.mode = 3;
			packet.data.check = 0;
			
			packet.data.pos = g_Pcur  * 1000 *57.2958 ; 
			packet.data.velo = g_Vcur * 1000 *57.2958;
			packet.data.cur = g_Ccur* 10000;  
			
			//packet.data.pos = 20 * 1000;
			//packet.data.velo = 20 * 1000;
			//packet.data.cur = 30 * 1000;
			
			for (int i = 8; i < sizeof(Packet_t); i++)
				packet.data.check += packet.buffer[i];
			
			for(int i=0; i<packet.data.size; i++){
				TransUart0(packet.buffer[i]);
			}
		}
	}
		
}
