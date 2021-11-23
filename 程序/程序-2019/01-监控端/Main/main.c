//2019.09.26 侯
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
void deal_date(); //数据处理
void buzzer_mode(uchar mode);//蜂鸣器
uint buzzer_count=0;
uchar buzzer_step=1;

//菜单
uchar DIST[2]={0xe9,0x01};//489
uint DIST_temp=200;
uint water_height=10,pot_height=1200;
uchar water_height_score=50;

uchar cou=0,sec=0,min=0,SendDate_count=0;
uchar ruler_online_state=0;	 //超声波测距在线标志
uint  ruler_online_time=0; //水泵在线计时
uchar pump_online_state=0;	 //水泵在线标志
uint  pump_online_time=0;

uchar pump_power=0;//水泵启停标志
uchar pump_power_real=0;//远程端  水泵工作状态回传   

uchar down_state=0;	 //数据下载标志
uint down_state_time=0; //在线计时

uchar mode_state=0; //模式 0 自动   1手动
uchar message_state=1;//状态提示栏
uchar menu=1;//菜单 1 主页    2 设置—1    3  设置-2
uchar pump_auto_sign=0;//自动抽水标志  1则 抽水前不提醒 全自动
uchar set_state=1;//设置菜单内滚动标志
uint DIST_H=150;DIST_L=600;//水位上下限

uchar pump_error_time_set=30;//水泵异常延时时间设定值	 初始化为30秒  30秒水位未上升则视为异常
uchar pump_error_time=0;	 //水泵异常延时时间
uint  DIST_last=0;			 
uint  pump_error_count=0;
uchar buzzer_off=0;//消音标志

uchar send_add=0;//发送数据 目标设备地址
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
	SCON = 0x50;		//8位数据,可变波特率
	AUXR |= 0x01;		//串口1选择定时器2为波特率发生器
	AUXR |= 0x04;		//定时器2时钟为Fosc,即1T
	T2L = 0x8F;		//设定定时初值
	T2H = 0xFD;		//设定定时初值
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

void UartSendStr(char *p)
{
    while (*p)
    {
        UartSend(*p++);
    }
}
/****************************************  EEPROM相关  ***************************************************/
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
/*************************************  eeprom保存和读取  ***************************************************/
void eeprom_read(){
uchar date_L=0,date_H=0;//数据临时寄存
	pump_auto_sign  = Byte_Read(0x3501);    //读EEPROM的值
	mode_state      = Byte_Read(0x3502);    //读EEPROM的值
	pump_error_time_set = Byte_Read(0x3503);    //读EEPROM的值 
	date_L          = Byte_Read(0x3504);    
	date_H          = Byte_Read(0x3505);  
	DIST_L=date_H;						 //低水位阈值数据
	DIST_L=(DIST_L<<8)+date_L;
	date_L          = Byte_Read(0x3506);     
	date_H          = Byte_Read(0x3507);  
	DIST_H=date_H;						 //高水位阈值数据
	DIST_H=(DIST_H<<8)+date_L;
	first_online    = Byte_Read(0x3508); //首次上电标志
} 
void eeprom_save(){
uchar date_L=0,date_H=0;
	Sector_Erase(0x3501);           //擦除整个扇区
	Byte_Program(0x3501, pump_auto_sign);//抽水前提醒标志
	Byte_Program(0x3502, mode_state);	 //模式 
	Byte_Program(0x3503, pump_error_time_set);	//异常检测时间
	date_L=DIST_L;	  //分解uint
	date_H=(DIST_L>>8);
	Byte_Program(0x3504, date_L);		   //低水位阈值数据
	Byte_Program(0x3505, date_H);
	date_L=DIST_H;
	date_H=(DIST_H>>8);
	Byte_Program(0x3506, date_L);			//高水位阈值数据
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
/************************************************主进程**********************************************************************/
void main()
{
	set_jdy40=1;//透传模式
	date_init();//数据初始化
	buzzer_mode(1);//蜂鸣器初始化（静音）
	P_SW2 = 0x80;//使能访问扩展SFR 	//P_SW2 = 0x00;//关闭访问扩展SFR 
	delay_ms(200);
	I2C_Init();//I2C总线初始化
	delay_ms(200);
	OLED_Init(); //OLED初始化
	delay_ms(200);

    Show_Str(48,1,"欢迎",16,0);
    OLED_P8x16Str(32,4,"WELCOME!");
	delay_ms(800);OLED_Clear();delay_ms(200);	
	UartInit();
    ES = 1;
    EA = 1;
	time_init ();

	while(1)
	{   
	    delay_ms(20);
		RecDate();	//处理接收到的数据
		deal_date();//数据处理
		state_deal();//状态识别,核心工作逻辑		
	    oled_display();	//显示
	    key_scan();		//按键扫描
	    if(SendDate_count>=10){SendDate_count=0;SendDate();}//x*25ms 发送一次

	}	
}
/*********************************************状态判断，核心逻辑***************************************************/
void working_auto();   //自动工作模式核心逻辑
void working_manual(); //手动工作模式核心逻辑
void pump_error_detect();//抽水异常检测
void state_deal(){								//模式判断
	if(ruler_online_state==0){
		message_state=7;//"测量端未连接"
	}else{
		if(pump_online_state==0&&mode_state==0){   //水泵控制端未连接且自动模式
			message_state=3;//"水泵控制器未连接"
		}else if(pump_online_state==1&&mode_state==0){ //水泵控制端在线且自动模式---正常工作_自动模式
			working_auto();//自动模式正常工作	
		}else if(pump_online_state==1&&mode_state==1){ //手动模式 水泵控制器在线 ---提示可使用自动模式
			message_state=8;//"水泵控制器已连接，可使用自动模式"
		}else if(pump_online_state==0&&mode_state==1){ //水泵控制端离线 手动模式 ---正常工作_手动模式
			working_manual();//手动模式正常工作
		}
	}	
}
void working_auto(){								//自动模式
	if(DIST_temp>DIST_L){ //水位低于设置水位 
		if(pump_power==0&&pump_power_real==0){
			if(pump_auto_sign==0){				   //抽水前提醒
				message_state=6;//"请按[启/停]抽水" 
				buzzer_mode(2);//蜂鸣器 0-关 1-报警 2-提醒 
			}else{
				pump_power=1;		  //启动水泵
			}
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"水泵启动中..."
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==1){
//			message_state=5;//"抽水中..."
			pump_error_detect();//抽水异常检测
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"水泵停止中..."
			buzzer_mode(0);
		}
	}else if((DIST_temp<=DIST_L)&&(DIST_temp>=DIST_H)){ //正常水位
		buzzer_mode(0);
		if(pump_power==0&&pump_power_real==0){
			message_state=1;//"水位正常" 
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"水泵启动中..."
		}else if(pump_power==1&&pump_power_real==1){
//			message_state=5;//"抽水中..."
			pump_error_detect();//抽水异常检测
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"水泵停止中..."
		}		
	}else if((DIST_temp<DIST_H)){						 //水位高
		pump_power=0;  //关闭水泵
		if(pump_power==0&&pump_power_real==0){
			message_state=11;//"水位高"
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==0){
			message_state=9;//"水泵启动中..."
			buzzer_mode(0);
		}else if(pump_power==1&&pump_power_real==1){
			message_state=5;//"抽水中..."
			buzzer_mode(1);//报警
		}else if(pump_power==0&&pump_power_real==1){
			message_state=10;//"水泵停止中..."
			buzzer_mode(0);
		} 
	}		
}
void working_manual(){								   //手动模式
	if(DIST_temp>DIST_L){ //水位低于设置水位 
		if(pump_power==0){
			buzzer_mode(2);//蜂鸣器 0-关 1-报警 2-提醒 
			message_state=2;//"水位低"
		}else{
//			message_state=5;//"抽水中..."
			pump_error_detect();//抽水异常检测
		}
		buzzer_off=0;//消音标志  不消音 ---按键消音
	}else if((DIST_temp<=DIST_L)&&(DIST_temp>=DIST_H)){ //正常水位 
		if(pump_power==0){
			buzzer_mode(0);
			message_state=1;//"水位正常"
		}else{
			pump_error_detect();//抽水异常检测   抽水中。。。
		}
		buzzer_off=0;//消音标志		
	}else if((DIST_temp<DIST_H)){						 //水位高
		if(buzzer_off==0){		  //未消音
			buzzer_mode(1);//报警
			message_state=11;//"水位高"		
		}else{
			buzzer_mode(0);//消音
			message_state=11;//"水位高"			
		} 
	}
}
void pump_error_detect(){//抽水异常检测	 
	if(pump_error_count>39){//40*25ms=1s  定时器
		pump_error_count=0;
		if((DIST_temp+5)<DIST_last){	//水位上升5mm(测量误差3mm)，计时清零
			pump_error_time=0;	   
		}else{
			pump_error_time++;    //否则计时加1
		}
		DIST_last=DIST_temp;//一秒 更新水位
	}
	if(pump_error_time>=pump_error_time_set){  //计时到达设定值
		buzzer_mode(1);//报警
		message_state=4;//"水泵异常或水不足"
	}else{
		buzzer_mode(0);
		message_state=5;//"抽水中..."
	}			
}
/*********************************************oled显示界面***************************************************/
void oled_display(void)
{  
/**************************************主界面************************************************/    
	if(menu==1){	    
		if(down_state==1){Show_Str(8,0,"三",16,0);}	  //代替“↓”                        //第一行
		else{Show_Str(8,0,"  ",16,0);}
		if(ruler_online_state==1){Show_Str(24,0,"四",16,0);}	 //代替 超声波图标
		else{Show_Str(24,0,"  ",16,0);}
		if(pump_online_state==1){Show_Str(48,0,"六",16,0);}	    //代替 水泵图标
		else{Show_Str(48,0,"  ",16,0);}
		if(mode_state==1){Show_Str(88,0,"手动",16,0);}	    //显示 自动模式 or 手动
		else{Show_Str(88,0,"自动",16,0);}
	
		if(message_state==1){													              //第二行
			Show_Str_Center(0,2,"    水位真常    ",16,0);   //居中显示	‘真’代替‘正’
		}else if(message_state==2){
			Show_Str_Center(0,2,"     水位低     ",16,0);	
		}else if(message_state==3){
			Show_Str_Center(0,2,"水泵控制器未连接",16,0);
		}else if(message_state==4){
			Show_Str_Center(0,2,"水泵异常或水不足",16,0);
		}else if(message_state==5){
			Show_Str_Center(0,2,"    抽水中...   ",16,0);
		}else if(message_state==6){
			Show_Str_Center(0,2,"请按[启/停]抽水 ",16,0);
		}else if(message_state==7){
			Show_Str_Center(0,2,"  测量端未连接  ",16,0);
		}else if(message_state==8){
			Show_Str_Center(0,2," 可使用自动模式 ",16,0);
		}else if(message_state==9){
			Show_Str_Center(0,2," 水泵启动中...  ",16,0);
		}else if(message_state==10){
			Show_Str_Center(0,2," 水泵停止中...  ",16,0);
		}else if(message_state==11){
			Show_Str_Center(0,2,"     水位高     ",16,0);	
		}
	
		Show_Str(0,4,"实时测距:",16,0);						  //第三行
		if(ruler_online_state==1){
			OLED_ShowNum(72,4,DIST_temp,4,16,0);//数字，长度
		}else{
			Show_Str(72,4," ---",16,0);
		} 
		Show_Str(104,4,"mm",16,0);
	
		
		Show_Str(0,6,"设置",16,0);							   //第四行
		Show_Str(44,6,"启/停",16,0);   
		Show_Str(95,6,"模式",16,0);
/**************************************设置 菜单1************************************************/    
	}else if(menu==2){  
		if(set_state==1){									  //第一行
			Show_Str(0,0,"1. 高水位",16,1);					  //反色显示
			OLED_ShowNum(72,0,DIST_H,4,16,1);//数字，长度
			Show_Str(104,0,"mm ",16,1); 
		}else{
			Show_Str(0,0,"1. 高水位",16,0);
			OLED_ShowNum(72,0,DIST_H,4,16,0);//数字，长度
			Show_Str(104,0,"mm ",16,0);		
		}
		if(set_state==2){									  //第二行
			Show_Str(0,2,"2. 低水位",16,1);					  
			OLED_ShowNum(72,2,DIST_L,4,16,1);//数字，长度
			Show_Str(104,2,"mm ",16,1); 
		}else{
			Show_Str(0,2,"2. 低水位",16,0);
			OLED_ShowNum(72,2,DIST_L,4,16,0);//数字，长度
			Show_Str(104,2,"mm ",16,0);		
		}
		if(set_state==3){									  //第三行
			Show_Str(0,4,"3. 抽水前提醒 ",16,1);					  
			if(pump_auto_sign==0){Show_Str(112,4,"开",16,1);}else{Show_Str(112,4,"关",16,1);} 
		}else{
			Show_Str(0,4,"3. 抽水前提醒 ",16,0);					  
			if(pump_auto_sign==0){Show_Str(112,4,"开",16,0);}else{Show_Str(112,4,"关",16,0);}		
		}
		Show_Str(8,6,"-",16,0);Show_Str(40,6,"下一项",16,0);Show_Str(112,6,"+",16,0); //第四行
/**************************************设置 菜单2************************************************/  
	}else if(menu==3){
		if(set_state==1){									  //第一行
			Show_Str(0,0,"4. 异常检测 ",16,1);			
			OLED_ShowNum(96,0,pump_error_time_set,2,16,1);//数字，长度
			Show_Str(112,0,"秒",16,1); 
		}else{
			Show_Str(0,0,"4. 异常检测 ",16,0);			
			OLED_ShowNum(96,0,pump_error_time_set,2,16,0);//数字，长度
			Show_Str(112,0,"秒",16,0); 		
		}
		if(set_state==2){									  //第二行
			Show_Str(0,2,"5. 关  于       ",16,1);					  
		}else{
			Show_Str(0,2,"5. 关  于       ",16,0);	
		}
		if(set_state==3){									  //第三行
			Show_Str(0,4,"6. 退  出       ",16,1);					  
		}else{
			Show_Str(0,4,"6. 退  出       ",16,0);	
		}
		Show_Str(8,6,"-",16,0);Show_Str(40,6,"下一项",16,0);Show_Str(112,6,"+",16,0); //第四行		
	}
}
/***************************************按键相关******************************************************/
void key_scan(){
/*************************************HOME*****************************************/
	if(menu==1){
		if(key1==0){		
		  DelayMs(15);
//		  buzzer_mode(3);//响一声
		  if(key1==0){
		     while(key1==0){Show_Str(0,6,"设置",16,1);}//等待按键松开
			 menu=2; //进入设置菜单
			 buzzer_off=1;//消音
			 OLED_Clear();delay_ms(200);
		  }
		}
		if(key2==0){		 
		  DelayMs(15);
		  if(key2==0){
		     while(key2==0){Show_Str(44,6,"启/停",16,1);}//等待按键松开
			 pump_power=!pump_power;//水泵启停标志
			 buzzer_off=1;//消音
		  }
		}
		if(key3==0){
			DelayMs(15);
			if(key3==0){
				while(key3==0){Show_Str(95,6,"模式",16,1);}
				mode_state=!mode_state;
				pump_power=0;//切换模式后 水泵标志清零
				buzzer_off=1;//消音
				eeprom_save();
			}
		}
/************************************SET_1*************************************/
	}else if(menu==2){
		if(set_state==1){   //高水位
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
				 DIST_H-=10;if(DIST_H<=50){DIST_H=50;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=2;//切换下一项
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
		if(set_state==2){	   //低水位
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
				 DIST_L-=10;if(DIST_L<=DIST_H){DIST_L=DIST_H+10;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=3;//切换下一项
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
		if(set_state==3){		//抽水前提醒
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
				 pump_auto_sign=!pump_auto_sign;
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=1;//切换下一项
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
	}else if(menu==3){	   //异常检测时间
		if(set_state==1){
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
				 pump_error_time_set--;if(pump_error_time_set>100){pump_error_time_set=0;}
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=2;//切换下一项
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
		if(set_state==2){	      //关于
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
			
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=3;//切换下一项
			  }
			}
			if(key3==0){
				DelayMs(15);
				if(key3==0){
					while(key3==0){Show_Str(112,6,"+",16,1);}
				
				}
			}
		}
		if(set_state==3){	  //退出
			if(key1==0){		 
			  DelayMs(15);
			  if(key1==0){	 //-
			     while(key1==0){Show_Str(8,6,"-",16,1);}//等待按键松开
				 menu=1;set_state=1;OLED_Clear();delay_ms(200);eeprom_save();
			  }
			}
			if(key2==0){		
			  DelayMs(15);
			  if(key2==0){
			     while(key2==0){Show_Str(40,6,"下一项",16,1);}//等待按键松开
				 set_state=1;//切换下一项
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
/***************************************蜂鸣器******************************************************/
void buzzer_mode(uchar buz_mode){//	 0-关 1-报警 2-提醒  3-响一声   定时器： buzzer_count  25ms
	if(buz_mode==0){			 //关
		buzzer=1;
		buzzer_step=1;
	}else if(buz_mode==1){		 //报警
		if(buzzer_count>19){   //间隔20*25ms=500ms
			buzzer=!buzzer;
			buzzer_count=0;
		}
	}else if(buz_mode==2){		  //提醒
		if(buzzer_step==1){
			buzzer=1;
			buzzer_step=2;
		}else if(buzzer_step==2){
			buzzer=0;
			buzzer_step=3;
			buzzer_count=0;
		}else if(buzzer_step==3&&buzzer_count>11){ //12*25ms 响 300ms
			buzzer=1;
			buzzer_step=4;
			buzzer_count=0;
		}else if(buzzer_step==4&&buzzer_count>59){	//60*25ms  静音1500ms
			buzzer_step=1;		
		}
	}else if(buz_mode==3){	  //响一声
		buzzer=0;
		delay_ms(100);
		buzzer=1;
	}
}
/*****************************************发送数据、接收数据处理***********************************************************/
void SendDate(){
	uchar a,b;
	if(menu==1){Show_Str(0,0,"↑",16,0);}
	if(send_add==0){		  //发送给测量端
	    Date_Send[0]=0xab;	 //本设备地址AB
		Date_Send[1]=0xcd;	 //目标设备地址
		Date_Send[2]=0x00;  
	}else if(send_add==1){	  //发送给水泵控制器
	    Date_Send[0]=0xab;	  
		Date_Send[1]=0xef;	  //目标设备地址
		Date_Send[2]=pump_power;
	}
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
	DelayMs(15);
	send_add=!send_add; //切换当前通讯设备
	if(menu==1){Show_Str(0,0,"  ",16,0);}     
}
void RecDate(){
uchar i=0;
i=buffer_begin;
  if(RX_finish==1){
	   //重复校验
	   if(buffer[i]==buffer[i+3]&&buffer[i]==buffer[i+6]&&buffer[i+1]==buffer[i+4]&&buffer[i+1]==buffer[i+7]&&buffer[i+2]==buffer[i+5]&&buffer[i+2]==buffer[i+8])//校验无误
	   {
	   	  test=!test;
		  down_state=1;	//显示“↓”标志 
		  add=buffer[i];
	      if(add==0xcd){			//若第一位数据为 0xcd则为测量端传来  0xef 为水泵控制端
			  DIST[0]=buffer[i+1];	    //超声波测距数据
			  DIST[1]=buffer[i+2];
	          ruler_online_state=1;   //显示超声波图标
		      ruler_online_time=0;//显示超声波图标  计时    如果收到一次数据则认为已经连接		  
		  }else if(add==0xef){
		      pump_power_real=buffer[i+1];//水泵实际状态回传
		  	  pump_online_state=1;   //显示水泵在线图标
			  pump_online_time=0;   //水泵在线图标  计时
		  }   
	   }
	   wptr=0;
	   RX_finish=0; //串口收到数据标志 清零 等待下次接收到数据
   }
}
void deal_date(){	 //数据处理
	DIST_temp=DIST[1];				   //合并测距数据 精度mm
	DIST_temp=(DIST_temp<<8)+DIST[0];   
//	water_height=pot_height-DIST_temp;	//计算水位高度
//	water_height_score=(float)((float)water_height*100)/pot_height; //计算水位百分比
	
}
/******************************************定时器初始化*******************************************************/
void time_init (void){						
	TMOD = 0x11;         // 定时/计数器0,1工作于方式1     
    TH0 = 0x3c;          // 预置产生25ms时基信号   
    TL0 = 0xb0;   
    EA = 1;              // 开总中断   
    ET0 = 1;             // 定时/计数器0允许中断   
    TR0 = 1;             // 开闭定时/计数器0   
}
/*****************************************中断处理程序**********************************************/	
void tiem0(void) interrupt 1{   // T/C0中断服务程序(产生25ms时基信号)   
    cou++; 			 // 软计数器加1
	                     
	SendDate_count++;				   //发送数据 延时计时 
	if(SendDate_count>200){SendDate_count=0;} 

	cou_uart++;				   //串口 超过0.5秒没接收完数据则从头开始
	if(cou_uart>=100){wptr=0;}

	buzzer_count++;
	if(buzzer_count>20000){buzzer_count=0;}

	ruler_online_time++;	  
	if(ruler_online_time>160){ruler_online_state=0;}//4秒无信号认为超声波断开
	pump_online_time++;	  
	if(pump_online_time>160){pump_online_state=0;}//4秒无信号认为 水泵 断开

	down_state_time++;if(down_state_time>12){down_state=0;}//0.3秒“↓”消失

	pump_error_count++;								  //水泵异常检测 时基
	if(pump_error_count>20000){pump_error_count=0;}

    if(cou > 39){                 // 40*25=1000ms(1s)   
        cou = 0;               // 软计数器清零   
        sec++;                 // 秒计数器加1(进位10ms*100=1s) 
//		send_add=!send_add; //切换当前通讯设备
     if(sec > 59){          // 秒计数值到60    
            sec = 0;           // 秒计数器清零   
            min++;             // 分计数器加1(进位60s=1m)  
			if(min>59){min=0;}
        }
  
    }   
    TH0 = 0x3c;                // 定时器重装  
    TL0 = 0xb0;   
}
/********************************  EEPORM相关  ************************************************/
/*********************************  读一字节  *************************************************/
//读一字节，调用前需打开IAP 功能，入口:DPTR = 字节地址，返回:A = 读出字节
INT8U Byte_Read(INT16U add)
{
    IAP_DATA = 0x00;
    IAP_CONTR = ENABLE_ISP;         //打开IAP 功能, 设置Flash 操作等待时间
    IAP_CMD = 0x01;                 //IAP/ISP/EEPROM 字节读命令

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //设置目标单元地址的高8 位地址
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //设置目标单元地址的低8 位地址

    //EA = 0;
    IAP_TRIG = WD1;   //先送 WD1,再送WD2 到ISP/IAP 触发寄存器,每次都需如此
    IAP_TRIG = WD2;   //送完WD2 后，ISP/IAP 命令立即被触发起动
    _nop_();
    //EA = 1;
    IAP_Disable();  //关闭IAP 功能, 清相关的特殊功能寄存器,使CPU 处于安全状态,
                    //一次连续的IAP 操作完成之后建议关闭IAP 功能,不需要每次都关
    return (IAP_DATA);
}
/*******************************  字节编程  ********************************************************/
//字节编程，调用前需打开IAP 功能，入口:DPTR = 字节地址, A= 须编程字节的数据
void Byte_Program(INT16U add, INT8U ch)
{
    IAP_CONTR = ENABLE_ISP;         //打开 IAP 功能, 设置Flash 操作等待时间
    IAP_CMD = 0x02;                 //IAP/ISP/EEPROM 字节编程命令

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //设置目标单元地址的高8 位地址
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //设置目标单元地址的低8 位地址

    IAP_DATA = ch;                  //要编程的数据先送进IAP_DATA 寄存器
    //EA = 0;
    IAP_TRIG = WD1;   //先送 WD1,再送WD2 到ISP/IAP 触发寄存器,每次都需如此
    IAP_TRIG = WD2;   //送完WD2 后，ISP/IAP 命令立即被触发起动
    _nop_();
    //EA = 1;
    IAP_Disable();  //关闭IAP 功能, 清相关的特殊功能寄存器,使CPU 处于安全状态,
                    //一次连续的IAP 操作完成之后建议关闭IAP 功能,不需要每次都关
}
/***************************************  擦除扇区  ************************************************/
//擦除扇区, 入口:DPTR = 扇区地址
void Sector_Erase(INT16U add)
{
    IAP_CONTR = ENABLE_ISP;         //打开IAP 功能, 设置Flash 操作等待时间
    IAP_CMD = 0x03;                 //IAP/ISP/EEPROM 扇区擦除命令

    my_unTemp16.un_temp16 = add;
    IAP_ADDRH = my_unTemp16.un_temp8[0];    //设置目标单元地址的高8 位地址
    IAP_ADDRL = my_unTemp16.un_temp8[1];    //设置目标单元地址的低8 位地址

    //EA = 0;
    IAP_TRIG = WD1;   //先送 WD1,再送WD2 到ISP/IAP 触发寄存器,每次都需如此
    IAP_TRIG = WD2;   //送完WD2 后，ISP/IAP 命令立即被触发起动
    _nop_();
    //EA = 1;
    IAP_Disable();  //关闭IAP 功能, 清相关的特殊功能寄存器,使CPU 处于安全状态,
                    //一次连续的IAP 操作完成之后建议关闭IAP 功能,不需要每次都关
}
/*************************************  关闭IAP 功能  ********************************************/
void IAP_Disable()
{
    //关闭IAP 功能, 清相关的特殊功能寄存器,使CPU 处于安全状态,
    //一次连续的IAP 操作完成之后建议关闭IAP 功能,不需要每次都关
    IAP_CONTR = 0;      //关闭IAP 功能
    IAP_CMD   = 0;      //清命令寄存器,使命令寄存器无命令,此句可不用
    IAP_TRIG  = 0;      //清命令触发寄存器,使命令触发寄存器无触发,此句可不用
    IAP_ADDRH = 0;
    IAP_ADDRL = 0;
}