#include <stc8fxx.h>       //������Ƭ���Ĵ�����ͷ�ļ�
#include "intrins.h"//ͷ�ļ� 
#define uchar unsigned char
#define uint  unsigned int
#define MAIN_Fosc		11059200L	//������ʱ��

sbit led=P1^6; 
sbit test=P1^5;
sbit pump_L=P1^7; 
sbit pump_N=P5^4;

sbit set_jdy40=P1^4;  

uchar cou=0,sec=0,min=0,cou2=0;

uchar power_state=0;
uchar add=0;//�������� �豸��ַ �ݴ�
uchar add2=0;

uchar online=0;		 //���߱�־
uint online_time=0;	  //������ʱ
uchar send_state=0;
uint send_time=0;

void SendDate();
void RecDate();
void time_init (void);
void dist_init();
void dist_get();
/******************************��ʱ****************************************/
void  delay_ms(unsigned int ms)
{
   unsigned int i;
	 do{
	      i = MAIN_Fosc / 12000;
		  while(--i)	;   //14T per loop
     }while(--ms);
}
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
//			test=!test;
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
//				test=0;
			}
		}
		wptr++;
		if(wptr>30){wptr=0;}
		date_last=date_now;
	    cou_uart=0;	 
    }
}

void UartInit(void)		//9600bps@11.0592MHz
{
	SCON = 0x50;		//8λ����,�ɱ䲨����
	AUXR |= 0x01;		//����1ѡ��ʱ��2Ϊ�����ʷ�����
	AUXR |= 0x04;		//��ʱ��2ʱ��ΪFosc,��1T
	T2L = 0xE0;		//�趨��ʱ��ֵ
	T2H = 0xFE;		//�趨��ʱ��ֵ
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

/*********************************************������*****************************************************************/
/*********************************************������*****************************************************************/
void main()
{
   	unsigned char i=0,j=0,a=0,b=0,c=0;
	set_jdy40=1;//͸��ģʽ
	led=0;pump_L=0;pump_N=0;test=0;
	delay_ms(1000);
	led=1;test=1;

	UartInit();
    ES = 1;
    EA = 1;
	time_init();

	while(1)
	{
	   RecDate();		//��������
	   if(online==1){   
			if(power_state==0){pump_L=0;pump_N=0;test=1;}else{pump_L=1;pump_N=1;test=0;}
	   		if(send_state==1){	   	 //������ܵ�һ�η��͸�����������  �򷵻�һ������ 
				delay_ms(10);		 //�ȴ����Ͷ�����ģ��ָ�����״̬ 
				SendDate();
				send_state=0;
			}
		}else{
			pump_L=0;pump_N=0;//������ر�ˮ��	 ���߳�30����ر�ˮ��
			test=1;
		}
	}	
}
/*****************************************�������ݴ���***********************************************************/
void SendDate(){
	uchar a,b;
	led=0;
    Date_Send[0]=0xef;	 //�豸��ַef
	Date_Send[1]=power_state; //ˮ��״̬
	Date_Send[2]=0x00;  //����
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
	delay_ms(15); 
	led=1;  
}
void RecDate(){
	uchar i=0;
	i=buffer_begin;	//��Ч��������������ʼλ��
	if(RX_finish==1){
	   //�ظ�У��
	   if(buffer[i]==buffer[i+3]&&buffer[i]==buffer[i+6]&&buffer[i+1]==buffer[i+4]&&buffer[i+1]==buffer[i+7]&&buffer[i+2]==buffer[i+5]&&buffer[i+2]==buffer[i+8])//У������
	   {
	      add=buffer[i];
		  add2=buffer[i+1];
		  if(add==0xab&&add2==0xef){		   //ֻ���ܼ�ض�����
			  online=1;		   //����
		  	  online_time=0;
			  send_state=1;		//�����ͱ�־
//			  send_time=0;
		  	  power_state=buffer[i+2];  //ˮ�ÿ��ر�־		  
		  }
	   }
	   RX_finish=0; //�����յ����ݱ�־ ���� �ȴ��´ν��յ�����
	   wptr=0;
	}
}
/******************************************��ʱ����ʼ��*******************************************************/
void time_init (void){		//5����@11.0592MHz				
//	TMOD &= 0x01;         // ��ʱ/������0 �����ڷ�ʽ1     
//	TL0 = 0x78;		//���ö�ʱ��ֵ
//	TH0 = 0xEC;		//���ö�ʱ��ֵ
	AUXR |= 0x80;		//��ʱ��ʱ��1Tģʽ
	TMOD &= 0xF0;		//���ö�ʱ��ģʽ
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0x28;		//���ö�ʱ��ֵ

    EA = 1;              // �����ж�   
    ET0 = 1;             // ��ʱ/������0�����ж�   
    TR0 = 1;             // ���ն�ʱ/������0  
}
/*****************************************�жϴ������**********************************************/	
void tiem0(void) interrupt 1{   // T/C0�жϷ������(����5msʱ���ź�)   
    cou++;                      // ���������1 
	cou2++;
	if(cou2>200){cou2=0;}//ר�ü�ʱ �������ݼ��ʱ�� 
	cou_uart++;
	if(cou_uart>=100){wptr=0;} //����0.5��û�������������ͷ��ʼ
//	send_time++;
//	if(send_time>200){send_time=0;send_state=0;}

    if(cou > 200){                 // 200*5=1000ms(1s)   
        cou = 0;               // �����������   
        sec++;                 // ���������1(��λ10ms*100=1s) 
//		test=!test;
		online_time++;								//����30��
		if(online_time>29){online_time=0;online=0;}
     if(sec > 59){          // �����ֵ��60    
            sec = 0;           // �����������   
            min++;             // �ּ�������1(��λ60s=1m)  
			if(min>59){min=0;}
        }
  
    }   
//	TL0 = 0x78;		//���ö�ʱ��ֵ
//	TH0 = 0xEC;		//���ö�ʱ��ֵ  
	TL0 = 0x00;		//���ö�ʱ��ֵ
	TH0 = 0x28;		//���ö�ʱ��ֵ
}
