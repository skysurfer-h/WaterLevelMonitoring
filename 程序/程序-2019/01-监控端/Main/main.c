//2019.09.26 ��
#include <stc8fxx.h>
#include "intrins.h"
#include "main.h"
#include "i2c.h"
#include "SSD1306.h"
#include "GUI.h"
#include "font.h"
#include "delay.h"

#define uchar unsigned char
#define uint unsigned int

void oled_display(void);
void key_scan();
void deal_date(); //���ݴ���
void buzzer_mode(uchar mode);//������
uint buzzer_count=0;
uchar buzzer_step=1;

//�˵�
uchar DIST[2]={0xe9,0x01};//489
uint DIST_temp=200;
uint water_height=10,pot_height=1200;
uchar water_height_score=50;

uchar cou=0,sec=0,min=0,SendDate_count=0;
uchar ruler_online_state=0;	 //������������߱�־
uint  ruler_online_time=0; //ˮ�����߼�ʱ
uchar pump_online_state=0;	 //ˮ�����߱�־
uint  pump_online_time=0;

uchar pump_power=0;//ˮ����ͣ��־
uchar pump_power_real=0;//Զ�̶�  ˮ�ù���״̬�ش�   

uchar down_state=0;	 //�������ر�־
uint down_state_time=0; //���߼�ʱ

uchar mode_state=0; //ģʽ 0 �Զ�   1�ֶ�
uchar message_state=1;//״̬��ʾ��
uchar menu=1;//�˵� 1 ��ҳ    2 ���á�1    3  ����-2
uchar pump_auto_sign=0;//�Զ���ˮ��־  1�� ��ˮǰ������ ȫ�Զ�
uchar set_state=1;//���ò˵��ڹ�����־
uint DIST_H=150;DIST_L=600;//ˮλ������

uchar pump_error_time_set=30;//ˮ���쳣��ʱʱ���趨ֵ	 ��ʼ��Ϊ30��  30��ˮλδ��������Ϊ�쳣
uchar pump_error_time=0;	 //ˮ���쳣��ʱʱ��
uint  DIST_last=0;			 
uint  pump_error_count=0;
uchar buzzer_off=0;//������־

uchar send_add=0;//�������� Ŀ���豸��ַ
uchar add=0;

sbit key1=P1^6;
sbit key2=P1^7;
sbit key3=P5^4;
sbit buzzer=P1^2;
sbit test=P3^6;
sbit set_jdy40=P1^3;
//sbit key5=P3^6;
void oled_display(void);
void SendDate();
void RecDate();
void time_init (void);
void state_deal();
/*******************************uart*************************************/
bit busy,uart_state=0;
uchar wptr=0,temp;
uchar cou_uart=0;
uchar rptr;
uchar  ture_state=0;
uchar buffer[32],Date_Send[3];//���պͷ��͵���������
uchar date_now=0x00,date_last=0x00;
uchar RX_begin=0;RX_finish=0;
uchar date_finish_now=0x00;date_finish_last=0x00;
uchar buffer_begin=0;  //��ǰ֡������ʼλ��

void UartIsr() interrupt 4 using 1
{
    if (TI)
    {
        TI = 0;
        busy = 0;
    }
    if (RI)
    {
		RI = 0;
        date_now = SBUF;
        if((date_now==0x5a)&&(date_last==0xaa)){	  //��ʼ��Ϊ  aa   5a
			RX_begin=1;			   //�յ���ʼ��
			buffer_begin=wptr+1;//�������ݵ�һ�ֽ�λ��
     	}
		if(RX_begin==1){
			buffer[wptr]=date_now;
		}
		if(wptr>4){
			date_finish_now=buffer[wptr];date_finish_last=buffer[wptr-1];
			if((date_finish_now==0xc3)&&(date_finish_last==0xcc)){//������  cc  c3
				RX_begin=0;//����������һ֡����
				wptr=0;	
				RX_finish=1;//��ɽ���һ֡����
			}
		}
		wptr++;
		if(wptr>30){wptr=0;}
		date_last=date_now;
	    cou_uart=0;	 
    }
}

void UartInit()	//9600bps@24.000MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x01;		//����1ѡ��ʱ��2Ϊ�����ʷ�����
	AUXR |= 0x04;		//��ʱ��2ʱ��ΪFosc,��1T
	T2L = 0x8F;		//�趨��ʱ��ֵ
	T2H = 0xFD;		//�趨��ʱ��ֵ
	AUXR |= 0x10;		//������ʱ��2
    wptr = 0x00;
    rptr = 0x00;
    busy = 0;
}

void UartSend(char dat)
{
    while (busy);
    busy = 1;
    SBUF = dat;
}

void UartSendStr(char *p)
{
    while (*p)
    {
        UartSend(*p++);
    }
}
/****************************************  EEPROM���  ***************************************************/
#define WD1	0x5a	
#define WD2	0xa5

#define ENABLE_ISP 0x82 
typedef unsigned char      INT8U;         
typedef unsigned int   INT16U;

union union_temp16                               
{
    INT16U un_temp16;
    INT8U  un_temp8[2];
}my_unTemp16;

INT8U Byte_Read(INT16U add);             
void Byte_Program(INT16U add, INT8U ch);  
void Sector_Erase(INT16U add);            
void IAP_Disable(); 
uchar first_online=0;  
/*************************************  eeprom����Ͷ�ȡ  ***************************************************/
void eeprom_read(){
uchar date_L=0,date_H=0;//������ʱ�Ĵ�
	pump_auto_sign  = Byte_Read(0x3501);    //��EEPROM��ֵ
	mode_state      = Byte_Read(0x3502);    //��EEPROM��ֵ
	pump_error_time_set = Byte_Read(0x3503);    //��EEPROM��ֵ 
	date_L          = Byte_Read(0x3504);    
	date_H          = Byte_Read(0x3505);  
	DIST_L=date_H;						 //��ˮλ��ֵ����
	DIST_L=(DIST_L<<8)+date_L;
	date_L          = Byte_Read(0x3506);     
	date_H          = Byte_Read(0x3507);  
	DIST_H=date_H;						 //��ˮλ��ֵ����
	DIST_H=(DIST_H<<8)+date_L;
	first_online    = Byte_Read(0x3508); //�״��ϵ��־
} 
void eeprom_save(){
uchar date_L=0,date_H=0;
	Sector_Erase(0x3501);           //������������
	Byte_Program(0x3501, pump_auto_sign);//��ˮǰ���ѱ�־
	Byte_Program(0x3502, mode_state);	 //ģʽ 
	Byte_Program(0x3503, pump_error_time_set);	//�쳣���ʱ��
	date_L=DIST_L;	  //�ֽ�uint
	date_H=(DIST_L>>8);
	Byte_Program(0x3504, date_L);		   //��ˮλ��ֵ����
	Byte_Program(0x3505, date_H);
	date_L=DIST_H;
	date_H=(DIST_H>>8);
	Byte_Program(0x3506, date_L);			//��ˮλ��ֵ����
	Byte_Program(0x3507, date_H);
	Byte_Program(0x3508, 0xab);
}
void date_init(){
	eeprom_read();
	if(first_online!=0xab){
		pump_auto_sign=0;
		mode_state=0;
		pump_error_time_set=30;
		DIST_L=600;
		DIST_H=150;
		eeprom_save();
	}	
}
/************************************************������**********************************************************************/
void main()
{
	set_jdy40=1;//͸��ģʽ
	date_init();//���ݳ�ʼ��
	buzzer_mode(1);//��������ʼ����������
	P_SW2 = 0x80;//ʹ�ܷ�����չSFR 	//P_SW2 = 0x00;//�رշ�����չSFR 
	delay_ms(200);
	I2C_Init();//I2C���߳�ʼ��
	delay_ms(200);
	OLED_Init(); //OLED��ʼ��
	delay_ms(200);

    Show_Str(48,1,"��ӭ",16,0);
    OLED_P8x16Str(32,4,"WELCOME!");
	delay_ms(800);OLED_Clear();delay_ms(200);	
	UartInit();
    ES = 1;
    EA = 1;
	time_init ();

	while(1)
	{   
	    delay_ms(20);
		RecDate();	//������յ�������
		deal_date();//���ݴ���
		state_deal();//״̬ʶ��,���Ĺ����߼�		
	    oled_display();	//��ʾ
	    key_scan();		//����ɨ��
	    if(SendDate_count>=10){SendDate_count=0;SendDate();}//x*25ms ����һ��

	}	
}
/*********************************************״̬�жϣ������߼�***************************************************/
void working_auto();   //�Զ�����ģʽ�����߼�
void working_manual(); //�ֶ�����ģʽ�����߼�
void pump_error_detect();//��ˮ�쳣���
void state_deal(){								//ģʽ�ж�
	if(ruler_online_state==0){
		message_state=7;//"������δ����"
	}else{
		if(pump_online_state==0&&mode_state==0){   //ˮ�ÿ��ƶ�δ�������Զ�ģʽ
			message_state=3;//"ˮ�ÿ�����δ����"
		}else if(pump_online_state==1&&mode_state==0){ //ˮ�ÿ��ƶ��������Զ�ģʽ---��������_�Զ�ģʽ
			working_auto();//�Զ�ģʽ��������	
		}else if(pump_online_state==1&&mode_state==1){ //�ֶ�ģʽ ˮ�ÿ��������� ---��ʾ��ʹ���Զ�ģʽ
			message_state=8;//"ˮ�ÿ����������ӣ���ʹ���Զ�ģʽ"
		}else if(pump_online_state==0&&mode_state==1){ //ˮ�ÿ��ƶ����� �ֶ�ģʽ ---��������_�ֶ�ģʽ
			working_manual();//�ֶ�ģʽ��������
		}
	}	
}
void working_auto(){								//�Զ�ģʽ
	if(DIST_temp>DIST_L){ //ˮλ��������ˮλ 
		if(pump_power==0&&pump_power_real==0){
			if(pump_auto_sign==0){				   //��ˮǰ����
				message_state=6;//"�밴[��/ͣ]��ˮ" 
				buzzer_mode(2);//������ 0-�� 1-���� 2-���� 
			}else{
				pump_power=1;		  //����ˮ��
			}
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"ˮ��������..."
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==1){
//			message_state=5;//"��ˮ��..."
			pump_error_detect();//��ˮ�쳣���
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"ˮ��ֹͣ��..."
			buzzer_mode(0);
		}
	}else if((DIST_temp<=DIST_L)&&(DIST_temp>=DIST_H)){ //����ˮλ
		buzzer_mode(0);
		if(pump_power==0&&pump_power_real==0){
			message_state=1;//"ˮλ����" 
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"ˮ��������..."
		}else if(pump_power==1&&pump_power_real==1){
//			message_state=5;//"��ˮ��..."
			pump_error_detect();//��ˮ�쳣���
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"ˮ��ֹͣ��..."
		}		
	}else if((DIST_temp<DIST_H)){						 //ˮλ��
		pump_power=0;  //�ر�ˮ��
		if(pump_power==0&&pump_power_real==0){
			message_state=11;//"ˮλ��"
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"ˮ��������..."
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==1){
			message_state=5;//"��ˮ��..."
			buzzer_mode(1);//����
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"ˮ��ֹͣ��..."
			buzzer_mode(0);
		} 
	}		
}
void working_manual(){								   //�ֶ�ģʽ
	if(DIST_temp>DIST_L){ //ˮλ��������ˮλ 
		if(pump_power==0){
			buzzer_mode(2);//������ 0-�� 1-���� 2-���� 
			message_state=2;//"ˮλ��"
		}else{
//			message_state=5;//"��ˮ��..."
			pump_error_detect();//��ˮ�쳣���
		}
		buzzer_off=0;//������־  ������ ---��������
	}else if((DIST_temp<=DIST_L)&&(DIST_temp>=DIST_H)){ //����ˮλ 
		if(pump_power==0){
			buzzer_mode(0);
			message_state=1;//"ˮλ����"
		}else{
			pump_error_detect();//��ˮ�쳣���   ��ˮ�С�����
		}
		buzzer_off=0;//������־		
	}else if((DIST_temp<DIST_H)){						 //ˮλ��
		if(buzzer_off==0){		  //δ����
			buzzer_mode(1);//����
			message_state=11;//"ˮλ��"		
		}else{
			buzzer_mode(0);//����
			message_state=11;//"ˮλ��"			
		} 
	}
}
void pump_error_detect(){//��ˮ�쳣���	 
	if(pump_error_count>39){//40*25ms=1s  ��ʱ��
		pump_error_count=0;
		if((DIST_temp+5)<DIST_last){	//ˮλ����5mm(�������3mm)����ʱ����
			pump_error_time=0;	   
		}else{
			pump_error_time++;    //�����ʱ��1
		}
		DIST_last=DIST_temp;//һ�� ����ˮλ
	}
	if(pump_error_time>=pump_error_time_set){  //��ʱ�����趨ֵ
		buzzer_mode(1);//����
		message_state=4;//"ˮ���쳣��ˮ����"
	}else{
		buzzer_mode(0);
		message_state=5;//"��ˮ��..."
	}			
}
/*********************************************oled��ʾ����***************************************************/
void oled_display(void)
{  
/**************************************������************************************************/    
	if(menu==1){	    
		if(down_state==1){Show_Str(8,0,"��",16,0);}	  //���桰����                        //��һ��
		else{Show_Str(8,0,"  ",16,0);}
		if(ruler_online_state==1){Show_Str(24,0,"��",16,0);}	 //���� ������ͼ��
		else{Show_Str(24,0,"  ",16,0);}
		if(pump_online_state==1){Show_Str(48,0,"��",16,0);}	    //���� ˮ��ͼ��
		else{Show_Str(48,0,"  ",16,0);}
		if(mode_state==1){Show_Str(88,0,"�ֶ�",16,0);}	    //��ʾ �Զ�ģʽ or �ֶ�
		else{Show_Str(88,0,"�Զ�",16,0);}
	
		if(message_state==1){													              //�ڶ���
			Show_Str_Center(0,2,"    ˮλ�泣    ",16,0);   //������ʾ	���桯���桮����
		}else if(message_state==2){
			Show_Str_Center(0,2,"     ˮλ��     ",16,0);	
		}else if(message_state==3){
			Show_Str_Center(0,2,"ˮ�ÿ�����δ����",16,0);
		}else if(message_state==4){
			Show_Str_Center(0,2,"ˮ���쳣��ˮ����",16,0);
		}else if(message_state==5){
			Show_Str_Center(0,2,"    ��ˮ��...   ",16,0);
		}else if(message_state==6){
			Show_Str_Center(0,2,"�밴[��/ͣ]��ˮ ",16,0);
		}else if(message_state==7){
			Show_Str_Center(0,2,"  ������δ����  ",16,0);
		}else if(message_state==8){
			Show_Str_Center(0,2," ��ʹ���Զ�ģʽ ",16,0);
		}else if(message_state==9){
			Show_Str_Center(0,2," ˮ��������...  ",16,0);
		}else if(message_state==10){
			Show_Str_Center(0,2," ˮ��ֹͣ��...  ",16,0);
		}else if(message_state==11){
			Show_Str_Center(0,2,"     ˮλ��     ",16,0);	
		}
	
		Show_Str(0,4,"ʵʱ���:",16,0);						  //������
		if(ruler_online_state==1){
			OLED_ShowNum(72,4,DIST_temp,4,16,0);//���֣�����
		}else{
			Show_Str(72,4," ---",16,0);
		} 
		Show_Str(104,4,"mm",16,0);
	
		
		Show_Str(0,6,"����",16,0);							   //������
		Show_Str(44,6,"��/ͣ",16,0);   
		Show_Str(95,6,"ģʽ",16,0);
/**************************************���� �˵�1************************************************/    
	}else if(menu==2){  
		if(set_state==1){									  //��һ��
			Show_Str(0,0,"1. ��ˮλ",16,1);					  //��ɫ��ʾ
			OLED_ShowNum(72,0,DIST_H,4,16,1);//���֣�����
			Show_Str(104,0,"mm ",16,1); 
		}else{
			Show_Str(0,0,"1. ��ˮλ",16,0);
			OLED_ShowNum(72,0,DIST_H,4,16,0);//���֣�����
			Show_Str(104,0,"mm ",16,0);		
		}
		if(set_state==2){									  //�ڶ���
			Show_Str(0,2,"2. ��ˮλ",16,1);					  
			OLED_ShowNum(72,2,DIST_L,4,16,1);//���֣�����
			Show_Str(104,2,"mm ",16,1); 
		}else{
			Show_Str(0,2,"2. ��ˮλ",16,0);
			OLED_ShowNum(72,2,DIST_L,4,16,0);//���֣�����
			Show_Str(104,2,"mm ",16,0);		
		}
		if(set_state==3){									  //������
			Show_Str(0,4,"3. ��ˮǰ���� ",16,1);					  
			if(pump_auto_sign==0){Show_Str(112,4,"��",16,1);}else{Show_Str(112,4,"��",16,1);} 
		}else{
			Show_Str(0,4,"3. ��ˮǰ���� ",16,0);					  
			if(pump_auto_sign==0){Show_Str(112,4,"��",16,0);}else{Show_Str(112,4,"��",16,0);}		
		}
		Show_Str(8,6,"-",16,0);Show_Str(40,6,"��һ��",16,0);Show_Str(112,6,"+",16,0); //������
/**************************************���� �˵�2************************************************/  
	}else if(menu==3){
		if(set_state==1){									  //��һ��
			Show_Str(0,0,"4. �쳣��� ",16,1);			
			OLED_ShowNum(96,0,pump_error_time_set,2,16,1);//���֣�����
			Show_Str(112,0,"��",16,1); 
		}else{
			Show_Str(0,0,"4. �쳣��� ",16,0);			
			OLED_ShowNum(96,0,pump_error_time_set,2,16,0);//���֣�����
			Show_Str(112,0,"��",16,0); 		
		}
		if(set_state==2){									  //�ڶ���
			Show_Str(0,2,"5. ��  ��       ",16,1);					  
		}else{
			Show_Str(0,2,"5. ��  ��       ",16,0);	
		}
		if(set_state==3){									  //������
			Show_Str(0,4,"6. ��  ��       ",16,1);					  
		}else{
			Show_Str(0,4,"6. ��  ��       ",16,0);	
		}
		Show_Str(8,6,"-",16,0);Show_Str(40,6,"��һ��",16,0);Show_Str(112,6,"+",16,0); //������		
	}
}
/***************************************�������******************************************************/
void key_scan(){
/*************************************HOME*****************************************/
	if(menu==1){
		if(key1==0){		
		  DelayMs(15);
//		  buzzer_mode(3);//��һ��
		  if(key1==0){
		     while(key1==0){Show_Str(0,6,"����",16,1);}//�ȴ������ɿ�
			 menu=2; //�������ò˵�
			 buzzer_off=1;//����
			 OLED_Clear();delay_ms(200);
		  }
		}
		if(key2==0){		 
		  DelayMs(15);
		  if(key2==0){
		     while(key2==0){Show_Str(44,6,"��/ͣ",16,1);}//�ȴ������ɿ�
			 pump_power=!pump_power;//ˮ����ͣ��־
			 buzzer_off=1;//����
		  }
		}
		if(key3==0){
			DelayMs(15);
			if(key3==0){
				while(key3==0){Show_Str(95,6,"ģʽ",16,1);}
				mode_state=!mode_state;
				pump_power=0;//�л�ģʽ�� ˮ�ñ�־����
				buzzer_off=1;//����
				eeprom_save();
			}
		}
/************************************SET_1*************************************/
	}else if(menu==2){
		if(set_state==1){   //��ˮλ
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
				 DIST_H-=10;if(DIST_H<=50){DIST_H=50;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=2;//�л���һ��
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
					DIST_H+=10;if(DIST_H>=DIST_L){DIST_H=DIST_L-10;}
				}
			}
		}
		if(set_state==2){	   //��ˮλ
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
				 DIST_L-=10;if(DIST_L<=DIST_H){DIST_L=DIST_H+10;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=3;//�л���һ��
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
					DIST_L+=10;if(DIST_L>=2500){DIST_L=2500;}
				}
			}
		}
		if(set_state==3){		//��ˮǰ����
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
				 pump_auto_sign=!pump_auto_sign;
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=1;//�л���һ��
				 menu=3;OLED_Clear();delay_ms(200);
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
					pump_auto_sign=!pump_auto_sign;
				}
			}
		}		
/************************************SET_2***************************************/
	}else if(menu==3){	   //�쳣���ʱ��
		if(set_state==1){
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
				 pump_error_time_set--;if(pump_error_time_set>100){pump_error_time_set=0;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=2;//�л���һ��
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
					pump_error_time_set++;if(pump_error_time_set>99){pump_error_time_set=99;}
				}
			}
		}
		if(set_state==2){	      //����
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
			
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=3;//�л���һ��
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
				
				}
			}
		}
		if(set_state==3){	  //�˳�
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//�ȴ������ɿ�
				 menu=1;set_state=1;OLED_Clear();delay_ms(200);eeprom_save();
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"��һ��",16,1);}//�ȴ������ɿ�
				 set_state=1;//�л���һ��
				 menu=2;OLED_Clear();delay_ms(200);
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
					menu=1;set_state=1;OLED_Clear();delay_ms(200);eeprom_save();
				}
			}
		}			
	}

}
/***************************************������******************************************************/
void buzzer_mode(uchar buz_mode){//	 0-�� 1-���� 2-����  3-��һ��   ��ʱ���� buzzer_count  25ms
	if(buz_mode==0){			 //��
		buzzer=1;
		buzzer_step=1;
	}else if(buz_mode==1){		 //����
		if(buzzer_count>19){   //���20*25ms=500ms
			buzzer=!buzzer;
			buzzer_count=0;
		}
	}else if(buz_mode==2){		  //����
		if(buzzer_step==1){
			buzzer=1;
			buzzer_step=2;
		}else if(buzzer_step==2){
			buzzer=0;
			buzzer_step=3;
			buzzer_count=0;
		}else if(buzzer_step==3&&buzzer_count>11){ //12*25ms �� 300ms
			buzzer=1;
			buzzer_step=4;
			buzzer_count=0;
		}else if(buzzer_step==4&&buzzer_count>59){	//60*25ms  ����1500ms
			buzzer_step=1;		
		}
	}else if(buz_mode==3){	  //��һ��
		buzzer=0;
		delay_ms(100);
		buzzer=1;
	}
}
/*****************************************�������ݡ��������ݴ���***********************************************************/
void SendDate(){
	uchar a,b;
	if(menu==1){Show_Str(0,0,"��",16,0);}
	if(send_add==0){		  //���͸�������
	    Date_Send[0]=0xab;	 //���豸��ַAB
		Date_Send[1]=0xcd;	 //Ŀ���豸��ַ
		Date_Send[2]=0x00;  
	}else if(send_add==1){	  //���͸�ˮ�ÿ�����
	    Date_Send[0]=0xab;	  
		Date_Send[1]=0xef;	  //Ŀ���豸��ַ
		Date_Send[2]=pump_power;
	}
	UartSend(0xaa);//��ʼ��
    UartSend(0x5a);
	for(b=0;b<3;b++){			//��������
	    for (a=0; a<3; a++)
	    {
	       UartSend(Date_Send[a]);
	    }
	}
	UartSend(0xcc);//������
	UartSend(0xc3); 
	DelayMs(15);
	send_add=!send_add; //�л���ǰͨѶ�豸
	if(menu==1){Show_Str(0,0,"  ",16,0);}     
}
void RecDate(){
uchar i=0;
i=buffer_begin;
  if(RX_finish==1){
	   //�ظ�У��
	   if(buffer[i]==buffer[i+3]&&buffer[i]==buffer[i+6]&&buffer[i+1]==buffer[i+4]&&buffer[i+1]==buffer[i+7]&&buffer[i+2]==buffer[i+5]&&buffer[i+2]==buffer[i+8])//У������
	   {
	   	  test=!test;
		  down_state=1;	//��ʾ��������־ 
		  add=buffer[i];
	      if(add==0xcd){			//����һλ����Ϊ 0xcd��Ϊ�����˴���  0xef Ϊˮ�ÿ��ƶ�
			  DIST[0]=buffer[i+1];	    //�������������
			  DIST[1]=buffer[i+2];
	          ruler_online_state=1;   //��ʾ������ͼ��
		      ruler_online_time=0;//��ʾ������ͼ��  ��ʱ    ����յ�һ����������Ϊ�Ѿ�����		  
		  }else if(add==0xef){
		      pump_power_real=buffer[i+1];//ˮ��ʵ��״̬�ش�
		  	  pump_online_state=1;   //��ʾˮ������ͼ��
			  pump_online_time=0;   //ˮ������ͼ��  ��ʱ
		  }   
	   }
	   wptr=0;
	   RX_finish=0; //�����յ����ݱ�־ ���� �ȴ��´ν��յ�����
   }
}
void deal_date(){	 //���ݴ���
	DIST_temp=DIST[1];				   //�ϲ�������� ����mm
	DIST_temp=(DIST_temp<<8)+DIST[0];   
//	water_height=pot_height-DIST_temp;	//����ˮλ�߶�
//	water_height_score=(float)((float)water_height*100)/pot_height; //����ˮλ�ٷֱ�
	
}
/******************************************��ʱ����ʼ��*******************************************************/
void time_init (void){						
	TMOD = 0x11;         // ��ʱ/������0,1�����ڷ�ʽ1     
    TH0 = 0x3c;          // Ԥ�ò���25msʱ���ź�   
    TL0 = 0xb0;   
    EA = 1;              // �����ж�   
    ET0 = 1;             // ��ʱ/������0�����ж�   
    TR0 = 1;             // ���ն�ʱ/������0   
}
/*****************************************�жϴ������**********************************************/	
void tiem0(void) interrupt 1{   // T/C0�жϷ������(����25msʱ���ź�)   
    cou++; 			 // ���������1
	                     
	SendDate_count++;				   //�������� ��ʱ��ʱ 
	if(SendDate_count>200){SendDate_count=0;} 

	cou_uart++;				   //���� ����0.5��û�������������ͷ��ʼ
	if(cou_uart>=100){wptr=0;}

	buzzer_count++;
	if(buzzer_count>20000){buzzer_count=0;}

	ruler_online_time++;	  
	if(ruler_online_time>160){ruler_online_state=0;}//4�����ź���Ϊ�������Ͽ�
	pump_online_time++;	  
	if(pump_online_time>160){pump_online_state=0;}//4�����ź���Ϊ ˮ�� �Ͽ�

	down_state_time++;if(down_state_time>12){down_state=0;}//0.3�롰������ʧ

	pump_error_count++;								  //ˮ���쳣��� ʱ��
	if(pump_error_count>20000){pump_error_count=0;}

    if(cou > 39){                 // 40*25=1000ms(1s)   
        cou = 0;               // �����������   
        sec++;                 // ���������1(��λ10ms*100=1s) 
//		send_add=!send_add; //�л���ǰͨѶ�豸
     if(sec > 59){          // �����ֵ��60    
            sec = 0;           // �����������   
            min++;             // �ּ�������1(��λ60s=1m)  
			if(min>59){min=0;}
        }
  
    }   
    TH0 = 0x3c;                // ��ʱ����װ  
    TL0 = 0xb0;   
}
/********************************  EEPORM���  ************************************************/
/*********************************  ��һ�ֽ�  *************************************************/
//��һ�ֽڣ�����ǰ���IAP ���ܣ����:DPTR = �ֽڵ�ַ������:A = �����ֽ�
INT8U Byte_Read(INT16U add)
{
    IAP_DATA = 0x00;
    IAP_CONTR = ENABLE_ISP;         //��IAP ����, ����Flash �����ȴ�ʱ��
    IAP_CMD = 0x01;                 //IAP/ISP/EEPROM �ֽڶ�����

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //����Ŀ�굥Ԫ��ַ�ĸ�8 λ��ַ
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //����Ŀ�굥Ԫ��ַ�ĵ�8 λ��ַ

    //EA = 0;
    IAP_TRIG = WD1;   //���� WD1,����WD2 ��ISP/IAP �����Ĵ���,ÿ�ζ������
    IAP_TRIG = WD2;   //����WD2 ��ISP/IAP ����������������
    _nop_();
    //EA = 1;
    IAP_Disable();  //�ر�IAP ����, ����ص����⹦�ܼĴ���,ʹCPU ���ڰ�ȫ״̬,
                    //һ��������IAP �������֮����ر�IAP ����,����Ҫÿ�ζ���
    return (IAP_DATA);
}
/*******************************  �ֽڱ��  ********************************************************/
//�ֽڱ�̣�����ǰ���IAP ���ܣ����:DPTR = �ֽڵ�ַ, A= �����ֽڵ�����
void Byte_Program(INT16U add, INT8U ch)
{
    IAP_CONTR = ENABLE_ISP;         //�� IAP ����, ����Flash �����ȴ�ʱ��
    IAP_CMD = 0x02;                 //IAP/ISP/EEPROM �ֽڱ������

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //����Ŀ�굥Ԫ��ַ�ĸ�8 λ��ַ
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //����Ŀ�굥Ԫ��ַ�ĵ�8 λ��ַ

    IAP_DATA = ch;                  //Ҫ��̵��������ͽ�IAP_DATA �Ĵ���
    //EA = 0;
    IAP_TRIG = WD1;   //���� WD1,����WD2 ��ISP/IAP �����Ĵ���,ÿ�ζ������
    IAP_TRIG = WD2;   //����WD2 ��ISP/IAP ����������������
    _nop_();
    //EA = 1;
    IAP_Disable();  //�ر�IAP ����, ����ص����⹦�ܼĴ���,ʹCPU ���ڰ�ȫ״̬,
                    //һ��������IAP �������֮����ر�IAP ����,����Ҫÿ�ζ���
}
/***************************************  ��������  ************************************************/
//��������, ���:DPTR = ������ַ
void Sector_Erase(INT16U add)
{
    IAP_CONTR = ENABLE_ISP;         //��IAP ����, ����Flash �����ȴ�ʱ��
    IAP_CMD = 0x03;                 //IAP/ISP/EEPROM ������������

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //����Ŀ�굥Ԫ��ַ�ĸ�8 λ��ַ
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //����Ŀ�굥Ԫ��ַ�ĵ�8 λ��ַ

    //EA = 0;
    IAP_TRIG = WD1;   //���� WD1,����WD2 ��ISP/IAP �����Ĵ���,ÿ�ζ������
    IAP_TRIG = WD2;   //����WD2 ��ISP/IAP ����������������
    _nop_();
    //EA = 1;
    IAP_Disable();  //�ر�IAP ����, ����ص����⹦�ܼĴ���,ʹCPU ���ڰ�ȫ״̬,
                    //һ��������IAP �������֮����ر�IAP ����,����Ҫÿ�ζ���
}
/*************************************  �ر�IAP ����  ********************************************/
void IAP_Disable()
{
    //�ر�IAP ����, ����ص����⹦�ܼĴ���,ʹCPU ���ڰ�ȫ״̬,
    //һ��������IAP �������֮����ر�IAP ����,����Ҫÿ�ζ���
    IAP_CONTR = 0;      //�ر�IAP ����
    IAP_CMD   = 0;      //������Ĵ���,ʹ����Ĵ���������,�˾�ɲ���
    IAP_TRIG  = 0;      //��������Ĵ���,ʹ������Ĵ����޴���,�˾�ɲ���
    IAP_ADDRH = 0;
    IAP_ADDRL = 0;
}