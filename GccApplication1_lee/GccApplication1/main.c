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

volatile double g_Vcur, g_Vpre;
volatile double g_Vdes;
volatile double g_Verr;
volatile double g_Vlimit = 1.;


volatile double g_Ccur;
volatile double g_Cdes;
volatile double g_Cerr;
volatile double g_Cerr_sum;
volatile double g_Climit = 1.;

volatile double g_ADC;
volatile int g_SendFlag = 0;
volatile int g_Direction;

volatile int cur_control;
volatile double g_vel_control;
volatile double g_pos_control;
volatile unsigned char g_TimerCnt;

volatile unsigned char checkSize;
volatile unsigned char g_buf[256], g_BufWriteCnt, g_BufReadCnt;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID = 1;

// 게인 변수들
volatile double g_Pderr;
volatile double g_Verr_sum;
volatile double Kp_p = 11;
volatile double Kd_p = 0.7;

volatile double Kp_s = 0.32;
volatile double Ki_s = 6.5;

//volatile double Kp_p = 12.5;
//volatile double Kd_p = 0.15;
//
//volatile double Kp_s = 0.032;
//volatile double Ki_s = 8.6271;
//
//volatile double Kp_c = 0.826;
//volatile double Ki_c = 2211.61
volatile double Kp_c = 0.6;
volatile double Ki_c = 2150;


void SetDutyCW(double v){
   
   while(TCNT1  == 0);

   int ocr = v * (200. / 24.) + 200;
   
   if(ocr > OCR_MAX)   ocr = OCR_MAX;
   else if(ocr < OCR_MIN)   ocr = OCR_MIN;
   //OCR1A = OCR1B = ocr;
   
   OCR1A = OCR3B = ocr + 8;      //1 H
   OCR1B = OCR3A = ocr - 8;      //1 L
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
	// 안티 게인 설정
   double Ka_s = 1/2*Kp_s;
   double Ka_c = 1/2*Kp_c;
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
   cnt = SPI_MasterRecv();      cnt = cnt<< 8;
   cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
   cnt |= SPI_MasterRecv();   cnt = cnt<< 8;
   cnt |= SPI_MasterRecv();
   PORTB = 0x01;
   g_Cnt = -cnt;
   
   PORTC = 0x03;
   g_Pcur = (g_Cnt / (4096. * 81.)) * 2 * M_PI;
   //TO DO

   /////////////////////////////////////////
    //position control
    if((g_TimerCnt % 100) == 0){  
       // 위치 제어기는 전류제어기 주기의 1/100으로 잡는다.
	   g_TimerCnt = 0;
       if(g_Pdes < 0) g_Pdes + 2*M_PI;   // 음수 target시 0~360으로 표현
       
       g_Perr = g_Pdes - (g_Pcur);      // 목표 위치 값 - 현재 위치 값 = position error 
       g_Pderr = g_Perr - g_Ppre;      // 현재 에러 값 - 이전 에러 값 = position error_dot
       
       g_pos_control = g_Perr * Kp_p +  g_Pderr* Kd_p;      //PD 제어기
	   //
       if(g_pos_control > 642.65/81.){
          g_pos_control = 642.65/81.;
       }
       else if(g_pos_control < -642.65/81.){
          g_pos_control = -642.65/81.;
       }
       g_Ppre = g_Perr;      // 현재 위치 에러 값을 이전 위치 에러 값으로 저장
    }
      // speed control
    if((g_TimerCnt % 10) == 0){       
       
      //saturation
	  // 속도에서 입력받는 값은 속도의 최대값이므로 만약에 속도지령이 속도 최대값을 넘길경우 제한해줌
       if(g_pos_control > g_Vlimit){
          g_pos_control = g_Vlimit;
       }
       else if(g_pos_control < -g_Vlimit){
          g_pos_control = -g_Vlimit;
       }
       
       g_Vdes = g_pos_control;  // 목표하는 속도
       g_Vcur = (g_Pcur - g_Pvcur) / 0.005; //현재 속도 = 위치의 변화량 / (제어주기*10)
	   // 현재 제어주기가 0.000521 ~= 0.0005 이므로 시간변화량은 0.0005이다.
	   // 속도제어기의 제어주기는 전류제어기의 1/10 주기로 잡는다.   
       g_Pvcur = g_Pcur;           //
       g_Verr = g_Vdes - g_Vcur; 
       
       g_vel_control = g_Verr * Kp_s + g_Verr_sum * Ki_s * 0.005;      // PI 제어기
       // I-term은 에러의 합 * 시간변화량 * IGain 값이다.
       g_Verr_sum += g_Verr;   // 속도 값에 대한 에러 값을 계속 누적
       // 최대 허용 전류 값에 대한 saturation & anti-windup
       if(g_vel_control > 2.08){
		  g_Verr_sum -= (g_vel_control - 2.08) * 1. *Ka_s;   //  anti windup gain은 1/2Kps
		  g_vel_control = 2.08;
       }
       else if(g_vel_control < -2.08){
		  g_Verr_sum -= (g_vel_control + 2.08) * 1. *Ka_s; //  anti windup gain은 1/2Kps
		  g_vel_control = -2.08;
       }
    }
   g_TimerCnt++;
   //current control
   if(g_vel_control > g_Climit){
	   g_vel_control = g_Climit;
   }
   else if(g_vel_control < -g_Climit){
	   g_vel_control = -g_Climit;
   }
   g_Cdes = g_vel_control; // 전류지령은 속도제어기의 출력값,    
   
   g_Ccur = -( ((g_ADC / 1024. * 5.) - 2.5) * 10.);
   g_Cerr = g_Cdes - g_Ccur;

   cur_control = g_Cerr * Kp_c + g_Cerr_sum * Ki_c* 0.0005;
   cur_control += g_Vcur * 0.0683;
   
   g_Cerr_sum += g_Cerr;
   
   //I-term anti
   if(cur_control > 24){
	   g_Cerr_sum -= (cur_control - 24) * 1.*Ka_c;
	   cur_control = 24;
   }
   else if(cur_control < -24){
	  g_Cerr_sum += (cur_control + 24) * 1.*Ka_c;
	   cur_control = -24;
   }
   
   // 최종적으로 전류 제어 값으로 전압 값을 계산하기 위해 값을 넘겨 줌
   SetDutyCW(cur_control);            // target Voltage
   
   
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
                     g_Pdes = g_PacketBuffer.data.pos/ 1000.;
                     g_Vlimit = g_PacketBuffer.data.velo/ 1000.;
                     g_Climit = g_PacketBuffer.data.cur/ 1000.;
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
         
         //packet.data.pos = g_Pdes * 1000; 
         //packet.data.velo = g_Vlimit * 1000;
         //packet.data.cur = g_Climit * 1000;  
         
         packet.data.pos = g_Pcur *1000;
         packet.data.velo = g_Vcur * 1000;
         packet.data.cur = g_Ccur * 1000;
         
         for (int i = 8; i < sizeof(Packet_t); i++)
         packet.data.check += packet.buffer[i];
         
         for(int i=0; i<packet.data.size; i++){
            TransUart0(packet.buffer[i]);
         }
      }
   }
      
}
