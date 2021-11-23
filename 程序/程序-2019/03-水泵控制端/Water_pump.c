#include <stc8fxx.h>       //包含单片机寄存器的头文件
#include "intrins.h"//头文件 
#define uchar unsigned char
#define uint  unsigned int
#define MAIN_Fosc		11059200L	//定义主时钟

sbit led=P1^6; 
sbit test=P1^5;
sbit pump_L=P1^7; 
sbit pump_N=P5^4;

sbit set_jdy40=P1^4;  

uchar cou=0,sec=0,min=0,cou2=0;

uchar power_state=0;
uchar add=0;//接收数据 设备地址 暂存
uchar add2=0;

uchar online=0;		 //在线标志
uint online_time=0;	  //工作计时
uchar send_state=0;
uint send_time=0;

void SendDate();
void RecDate();
void time_init (void);
void dist_init();
void dist_get();
/******************************延时****************************************/
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
uchar buffer[32],Date_Send[3];//接收和发送的数据数组
uchar date_now=0x00,date_last=0x00;
uchar RX_begin=0;RX_finish=0;
uchar date_finish_now=0x00;date_finish_last=0x00;
uchar buffer_begin=0;  //当前帧数据起始位置

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
        if((date_now==0x5a)&&(date_last==0xaa)){	  //起始码为  aa   5a
			RX_begin=1;			   //收到起始码
			buffer_begin=wptr+1;//数据内容第一字节位置
//			test=!test;
     	}
		if(RX_begin==1){
			buffer[wptr]=date_now;
		}
		if(wptr>4){
			date_finish_now=buffer[wptr];date_finish_last=buffer[wptr-1];
			if((date_finish_now==0xc3)&&(date_finish_last==0xcc)){//结束码  cc  c3
				RX_begin=0;//结束接收这一帧数据
				wptr=0;	
				RX_finish=1;//完成接收一帧数据
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
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0xE0;		//设定定时初值
	T2H = 0xFE;		//设定定时初值
	AUXR |= 0x10;		//启动定时器2

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

/*********************************************主函数*****************************************************************/
/*********************************************主函数*****************************************************************/
void main()
{
   	unsigned char i=0,j=0,a=0,b=0,c=0;
	set_jdy40=1;//透传模式
	led=0;pump_L=0;pump_N=0;test=0;
	delay_ms(1000);
	led=1;test=1;

	UartInit();
    ES = 1;
    EA = 1;
	time_init();

	while(1)
	{
	   RecDate();		//接收数据
	   if(online==1){   
			if(power_state==0){pump_L=0;pump_N=0;test=1;}else{pump_L=1;pump_N=1;test=0;}
	   		if(send_state==1){	   	 //如果接受到一次发送给本机的数据  则返回一次数据 
				delay_ms(10);		 //等待发送端无线模块恢复接收状态 
				SendDate();
				send_state=0;
			}
		}else{
			pump_L=0;pump_N=0;//离线则关闭水泵	 离线超30秒则关闭水泵
			test=1;
		}
	}	
}
/*****************************************发送数据处理***********************************************************/
void SendDate(){
	uchar a,b;
	led=0;
    Date_Send[0]=0xef;	 //设备地址ef
	Date_Send[1]=power_state; //水泵状态
	Date_Send[2]=0x00;  //保留
	UartSend(0xaa);//起始码
	UartSend(0x5a);
	for(b=0;b<3;b++){			//发送三遍
	    for (a=0; a<3; a++)
	    {
	       UartSend(Date_Send[a]);
	    }
	}
	UartSend(0xcc);//结束码
	UartSend(0xc3);
	delay_ms(15); 
	led=1;  
}
void RecDate(){
	uchar i=0;
	i=buffer_begin;	//有效数据在数组中起始位置
	if(RX_finish==1){
	   //重复校验
	   if(buffer[i]==buffer[i+3]&&buffer[i]==buffer[i+6]&&buffer[i+1]==buffer[i+4]&&buffer[i+1]==buffer[i+7]&&buffer[i+2]==buffer[i+5]&&buffer[i+2]==buffer[i+8])//校验无误
	   {
	      add=buffer[i];
		  add2=buffer[i+1];
		  if(add==0xab&&add2==0xef){		   //只接受监控端数据
			  online=1;		   //在线
		  	  online_time=0;
			  send_state=1;		//允许发送标志
//			  send_time=0;
		  	  power_state=buffer[i+2];  //水泵开关标志		  
		  }
	   }
	   RX_finish=0; //串口收到数据标志 清零 等待下次接收到数据
	   wptr=0;
	}
}
/******************************************定时器初始化*******************************************************/
void time_init (void){		//5毫秒@11.0592MHz				
//	TMOD &= 0x01;         // 定时/计数器0 工作于方式1     
//	TL0 = 0x78;		//设置定时初值
//	TH0 = 0xEC;		//设置定时初值
	AUXR |= 0x80;		//定时器时钟1T模式
	TMOD &= 0xF0;		//设置定时器模式
	TL0 = 0x00;		//设置定时初值
	TH0 = 0x28;		//设置定时初值

    EA = 1;              // 开总中断   
    ET0 = 1;             // 定时/计数器0允许中断   
    TR0 = 1;             // 开闭定时/计数器0  
}
/*****************************************中断处理程序**********************************************/	
void tiem0(void) interrupt 1{   // T/C0中断服务程序(产生5ms时基信号)   
    cou++;                      // 软计数器加1 
	cou2++;
	if(cou2>200){cou2=0;}//专用计时 发送数据间隔时间 
	cou_uart++;
	if(cou_uart>=100){wptr=0;} //超过0.5秒没接收完数据则从头开始
//	send_time++;
//	if(send_time>200){send_time=0;send_state=0;}

    if(cou > 200){                 // 200*5=1000ms(1s)   
        cou = 0;               // 软计数器清零   
        sec++;                 // 秒计数器加1(进位10ms*100=1s) 
//		test=!test;
		online_time++;								//在线30秒
		if(online_time>29){online_time=0;online=0;}
     if(sec > 59){          // 秒计数值到60    
            sec = 0;           // 秒计数器清零   
            min++;             // 分计数器加1(进位60s=1m)  
			if(min>59){min=0;}
        }
  
    }   
//	TL0 = 0x78;		//设置定时初值
//	TH0 = 0xEC;		//设置定时初值  
	TL0 = 0x00;		//设置定时初值
	TH0 = 0x28;		//设置定时初值
}
