/***********************************************************************************************
                        杭州电子科技大学 智能车（光电）K66例程（V1.0）
												                     发布时间：2017年11月19日
须知：1、本例程适用K66FX1M0VLQ18芯片，其他K66系列芯片暂未测试。
			2、本例程通过逐飞科技K66MDK库更改而成，并借鉴了超核K60库的相关文件。
			3、目前已知bug：（1）目前SD卡仅可录图、在OLED上显示之前的图像，传图像到上位机请使用K60。
											（2）FTM模块仅可以使用FTM0 FTM1 FTM2 模块，FTM3模块不可用。
											（3）SD卡使用标准SD卡协议，可能时序较慢。
更新说明：1、测试ADC模块，并修改时钟分频；（2018.2.26）
					2、替换了逐飞oled函数库，使用旧库oled函数，优化了时序；（2018.2.26）
					3、移植旧库flash读取函数，但尚未测试。（2018.2.26）
***********************************************************************************************/  
#include "headfile.h"
#include "extern.h"
#include "NRF2401.H"
#include "Wireless.h"
#include "image_process.h"
#include "encoder.h"
#include "menu.h"
#include "image_process.h"//图像处理(上位机图像)
#include "my_imagetransfer.h"	//sd卡
//unlock kinetis
/*         外部引用参数          */
#if(1)
extern byte cut_filed;
extern 	int Motorshuru,MotorshuchuP,MotorshuchuD;
extern int duoji_InitalDuty_Max,duoji_InitalDuty_Min;
extern uint8_t stop_side;
extern uint16_t uwb_Distance;
extern byte podao_flag;
extern byte Last_Line;
extern byte straight_road;
extern byte xuxian;
extern uint8_t RowNum,L_black[70],R_black[70],LCenter[70];
extern uint8_t binaryzation;
extern byte small_s;
extern byte xuxianjilu;
extern int LPulse,Dulse;
extern byte longroad;
extern byte shortroad;
extern byte differential;//方向 0 右 1 左
#endif
int CNT = 0;//帧数记录
/*        电感值,数组             */
#if(1)
#define   ADN      7 
#define precision  1000
int LLeft[20];
int Left[20];
int Right[20];
int RRight[20];
int Center[20]; 
int UCenter[20]; 
int Lxie[20];
int Rxie[20];
int16_t EV_VALUE1 ,EV_VALUE2 ,EV_VALUE3,EV_VALUE4,EV_VALUE5,EV_VALUE6,EV_VALUE7,EV_VALUE8;
int f_k[8]={1000,1000,1000,1000,1000,1000,1000,1000};
#endif
/*          偏差及各参数             */
#if(1)
double lasterror=0; 
double last_giveerror=0;
double last_secondE=0;
double secondEC=0;
double firstEC=0;
double nowError=0;
double DEVIATE=0;
double kp=0;
double kd=0;
#endif
/*         继承数组及标志位         */
#if(1)
int AD_cunchu[20][7]={0};
int shizi_cunchu[20]={0};
int shurupiancha=0;
uint8_t inherit_flag=0;
uint16_t ADC_buffer[8];
uint16_t inherit_uwb_Distance[10];
#endif
/*           falsh数组及存储的参数               */
#if(1)
uint16_t falsh_buffer[80];
uint16_t P_rule_buffer[25];
uint16_t D_rule_buffer[25];
uint16_t ff_buffer[25];
uint16_t LP=54;//左拐P
uint16_t D=45;//D项
uint8_t shuruP=29;//输入系数
uint8_t RP=54;//右P
uint8_t P5;//待用
uint8_t secP=28;//第二段函数系数
int change_point=200;
int Schange_point=550;
uint8_t protect_flag=0;//保护标志位
uint16_t duojiZhongZhi=1500;//舵机中值
uint8_t sdMode=1;//sd卡模式
uint16_t straightSpeed=3000;//直道速度
uint16_t shizizhuanPwm;//弯道速度
uint8_t chasup=6;//差速系数
#endif
/*             环岛参数         */
#if(1)
uint8_t big_island_add=0;
uint8_t big_island_num=0;
uint8_t judge_huan=0;
uint8_t youhuan=0;
uint8_t zuohuan=0;
uint8_t zuohuannbiaoji=0;
uint8_t zuohuanbegin=0;
uint8_t youhuanbegin=0;
uint8_t huanrightP=0;
uint8_t chuhuanrightP=0;
uint8_t huanneiLP=0;
uint8_t huanneiRP=0;
uint8_t huanleftP=0;
uint8_t chuhuanleftP=0;
int16_t huanzhong=0;
int16_t huanhou=0;
uint8_t huanjian=0;
uint8_t huandaodirect=0;
uint8_t image_chuhuanwide=0;
uint8_t huandaoquan=0;
uint8_t shihuan=0;
uint8_t imageP=0;
uint8_t imageD=0;
uint8_t jiluhuan=0;
uint8_t yith=0;
uint8_t erth=0;
int last_imageerror=0;
int island_third_count=0;
int out_huan_count=0;
uint8_t longadd=0;
uint8_t shortadd=0;
uint8_t islandcut=0;
uint8_t podaocut=0;
//uint8_t out_island_line=0;

int connectcut=0;
int huanbegin=0;
int fixed_error;
int fixed_duoji;
int direction=0;
int count_after_huiche=0;
int error[10]={0};
int duoji[10]={0};
//0表示50——90 1表示60——90  2表示70——90 3表示80——90 4表示90——90 5表示100——90
extern int chuhuanerror[5];
uint8_t huanNum[5]={0};
uint8_t huanlarge=0;
uint8_t huancount=0;
uint8_t huanallcount=0;
uint8_t out_island=0;
uint8_t out_island_count=0;
uint8_t in_island_count=0;

uint8_t out_island_line=0;
uint8_t quanNum=0;
uint8_t huannum=0;
uint8_t quanDirection=0;

uint8_t ruhuanP[5];
uint8_t inhuanP[5];
uint8_t chuhuanP[5];
uint8_t midruhuanP[5];
uint8_t midinhuanP[5];
uint8_t midchuhuanP[5];
uint8_t midhuanNum[5];


#endif
/*             偏差等参数                    */
#if(1)
uint8_t motor_stop=0;//停车标志位
uint32_t duoji_InitalDuty;//舵机占空比
uint8 change_flag = 0,count_flag = 0;//中断标志
uint8_t flash_flag=0;//存储数据标志
uint8_t Menu_Mode = 0;//进程序
uint8_t oled_flag=0;//显示屏标志
int function_second=0;//第二段函数数值
int function_first=0;//第二段函数数值
#endif
/*            赛道类型               */
#if(1)
uint8_t flag=0;
uint8_t shizi=0;
uint8_t wans_connect=0;
uint8_t ruwan=0;
uint8_t shizizhuan=0;
uint8_t waitF=0;
uint8_t backF=0;
uint8_t beginF=0;
int stop_count=0;
int no_startline_stop_count=0;
#endif
/*            模糊表微操参数                        */
#if(1)
int pff1,pff2,pff3,pff4,pff5;
int up1,up2,up3,up4,up5;
int ud1,ud2,ud3,ud4,ud5;
#endif
/*             直道条件         */
#if(1)
int16_t shizizhuanStop=200;
uint8_t zhidao=0;
/*              十字上下限及降速参数         */
int16_t shiziH=1023;
int16_t shiziL=600;
uint8_t long_stop_filed=0;
uint8_t long_stop_speed=0;
byte dizen=0;
int dizen_count=0;
#endif
/*               会车参数                 */
#if(1)
uint8_t CarFlag=2;
uint8_t Car=0;
int stop_line_cnt=0;
uint8_t Stop_Line_Huiche=0,huichejuli=0,huiche=0;
uint8_t shizihuiche=0;
uint8_t uwbhuiche=0;
uint8_t shiziNum=0;
uint8_t shizicount=0;
uint8_t shizicountflag=0;
int jichang=0; 
int tingjichang=0;
int daojichang=0;
int behind_speed=0;
int ahead_speed=0;
int behinderror=0;
int aheaderror=0;
uint8_t car_come=0;
uint8_t fache=0;
uint8_t already_huiche=0;
uint8_t huicheflag=0;
int center_cut=0;
int begin_cut=0;
int end_cut=0;
int behindcount=0;
int aheadcount=0;
int aheadjichang=0;
uint8_t behind=0;
uint8_t ahead=0;
uint8_t encoderI=0;
uint8_t encoderP=0;
int huichecount=0;
int tingchecount=0;
uint8_t huicheNum=0;
uint8_t Car0_MiddleStop_Count=0;
uint8_t Car0_EndStop_Count=0;
uint8_t stop_Menu_count=0;
uint8_t stopline_count_one=0;
uint8_t stopline_count_two=0;
int huiche_Menu_error=0;
int Car0_second_error=0;
int Car1_second_error=0;
int Car0_third_error=0;
int Car1_third_error=0;
int huiche_second_error=0;
int huiche_Menu_uwb=0;
int huiche_second_count=0;
int huiche_third_count=0;
int jianzhi=0;
#endif
/*               模糊表菜单切换flag           */
//uint8_t sd_Flag=0;
/*               摄像头           */
#if(1)
uint8_t yuzhi=80;
uint8_t encoder_LP=0;
uint8_t encoder_RP=0;
uint8_t encoder_LI=0;
uint8_t encoder_RI=0;
uint8_t camera_Flag=0;
uint8_t sd_Flag=0;
#endif
/*               无线模块         */
#if(1)
uint8 Receive_Flag=0,WiFi=0;
uint8 dangche=0;
uint8_t swm=0;
uint8 check=0;
#endif
void menu_read()
{
	FLASH_ReadByte(223,160,(uint8_t *) falsh_buffer);
	LP=falsh_buffer[0];
	D=falsh_buffer[1];
	shuruP=falsh_buffer[2];
	RP=falsh_buffer[3];
	protect_flag=falsh_buffer[4];
	duojiZhongZhi=falsh_buffer[5];
	sdMode=falsh_buffer[6];
  straightSpeed=falsh_buffer[7];
	chuhuanleftP=falsh_buffer[8];	
  chasup=falsh_buffer[9];
	chuhuanrightP=falsh_buffer[10];//
	camera_Flag=falsh_buffer[11];//
	oled_flag=falsh_buffer[12];
	secP=falsh_buffer[13];
	shiziH=falsh_buffer[14];
	shiziL=falsh_buffer[15];
	image_chuhuanwide=falsh_buffer[16];
	change_point=falsh_buffer[17];
 
	sd_Flag=falsh_buffer[19];//
	imageP=falsh_buffer[20];//
	imageD=falsh_buffer[21];//
	quanNum=falsh_buffer[22];
	huannum=falsh_buffer[23];
	quanDirection=falsh_buffer[24];
	yuzhi=falsh_buffer[25];
	encoder_LP=falsh_buffer[26];
	encoder_RP=falsh_buffer[27];
  encoder_LI=falsh_buffer[28];
  encoder_RI=falsh_buffer[29];
	huanNum[0]=falsh_buffer[30];
	huanNum[1]=falsh_buffer[31];
	long_stop_filed=falsh_buffer[32];
	long_stop_speed=falsh_buffer[33];
	out_island_line=falsh_buffer[34];
	Car0_second_error=falsh_buffer[18];
	Car1_second_error=falsh_buffer[35];
	dangche=falsh_buffer[36];
	binaryzation=falsh_buffer[37];
	Car0_MiddleStop_Count=falsh_buffer[38];
	Car0_EndStop_Count=falsh_buffer[39];
  Car0_third_error=falsh_buffer[40];
  Car1_third_error=falsh_buffer[41];
	huiche_Menu_uwb=falsh_buffer[42];
	aheaderror=falsh_buffer[43];
	encoderI=falsh_buffer[44];
	encoderP=falsh_buffer[45];
	out_island=falsh_buffer[46];
	Car=falsh_buffer[47];
	Motorshuru=falsh_buffer[48];
	MotorshuchuP=falsh_buffer[49];
	MotorshuchuD=falsh_buffer[50];
  longadd=falsh_buffer[51];
  shortadd=falsh_buffer[52];
  islandcut=falsh_buffer[53];
  podaocut=falsh_buffer[54];
  ruhuanP[0]=falsh_buffer[55];
	ruhuanP[1]=falsh_buffer[56];
	ruhuanP[2]=falsh_buffer[57];
	ruhuanP[3]=falsh_buffer[58];
	ruhuanP[4]=falsh_buffer[59];
	
	inhuanP[0]=falsh_buffer[60];
	inhuanP[1]=falsh_buffer[61];
	inhuanP[2]=falsh_buffer[62];
	inhuanP[3]=falsh_buffer[63];
	inhuanP[4]=falsh_buffer[64];
	
	chuhuanP[0]=falsh_buffer[65];
	chuhuanP[1]=falsh_buffer[66];
	chuhuanP[2]=falsh_buffer[67];
	chuhuanP[3]=falsh_buffer[68];
	chuhuanP[4]=falsh_buffer[69];
	huanNum[2]=falsh_buffer[70];
	huanNum[3]=falsh_buffer[71];
	huanNum[4]=falsh_buffer[72];
	stopline_count_one=falsh_buffer[73];
  stopline_count_two=falsh_buffer[74];
	big_island_add=falsh_buffer[75];
	if(duojiZhongZhi>1000)
		duojiZhongZhi=1000;
	if(straightSpeed>100)
		straightSpeed=50;
}
void menu_write()
{
	falsh_buffer[0]=LP;
	falsh_buffer[1]=D;
	falsh_buffer[2]=shuruP;
	falsh_buffer[3]=RP;
	falsh_buffer[4]=protect_flag;
	falsh_buffer[5]=duojiZhongZhi;
	falsh_buffer[6]=sdMode;
  falsh_buffer[7]=straightSpeed;
	falsh_buffer[8]=chuhuanleftP;	
  falsh_buffer[9]=chasup;
  falsh_buffer[10]=chuhuanrightP;		
  falsh_buffer[11]=camera_Flag;	
  falsh_buffer[12]=oled_flag;
  falsh_buffer[13]=secP;
	falsh_buffer[14]=shiziH;
	falsh_buffer[15]=shiziL;	
	falsh_buffer[16]=image_chuhuanwide;
	falsh_buffer[17]=change_point;
	falsh_buffer[19]=sd_Flag;
	falsh_buffer[20]=imageP;
	falsh_buffer[21]=imageD;
  falsh_buffer[22]=quanNum;
	falsh_buffer[23]=huannum;
	falsh_buffer[24]=quanDirection;
	falsh_buffer[25]=yuzhi;
	falsh_buffer[26]=encoder_LP;
	falsh_buffer[27]=encoder_RP;
  falsh_buffer[28]=encoder_LI;
  falsh_buffer[29]=encoder_RI;
	falsh_buffer[30]=huanNum[0];
	falsh_buffer[31]=huanNum[1];
	falsh_buffer[32]=long_stop_filed;
	falsh_buffer[33]=long_stop_speed;
	falsh_buffer[34]=out_island_line;
	falsh_buffer[18]=Car0_second_error;
	falsh_buffer[35]=Car1_second_error;
	falsh_buffer[36]=dangche;
	falsh_buffer[37]=binaryzation;
	falsh_buffer[38]=Car0_MiddleStop_Count;
	falsh_buffer[39]=Car0_EndStop_Count;
	falsh_buffer[40]=Car0_third_error;
  falsh_buffer[41]=Car1_third_error;
	falsh_buffer[42]=huiche_Menu_uwb;
	falsh_buffer[43]=aheaderror;
	falsh_buffer[44]=encoderI;
	falsh_buffer[45]=encoderP;
	falsh_buffer[46]=out_island;
	falsh_buffer[47]=Car;
	falsh_buffer[48]=Motorshuru;
	falsh_buffer[49]=MotorshuchuP;
	falsh_buffer[50]=MotorshuchuD;
	falsh_buffer[51]=longadd;
  falsh_buffer[52]=shortadd;
  falsh_buffer[53]=islandcut;
	falsh_buffer[54]=podaocut;
  falsh_buffer[55]=ruhuanP[0];
	falsh_buffer[56]=ruhuanP[1];
	falsh_buffer[57]=ruhuanP[2];
	falsh_buffer[58]=ruhuanP[3];
	falsh_buffer[59]=ruhuanP[4];
	
	falsh_buffer[60]=inhuanP[0];
	falsh_buffer[61]=inhuanP[1];
	falsh_buffer[62]=inhuanP[2];
	falsh_buffer[63]=inhuanP[3];
	falsh_buffer[64]=inhuanP[4];
	
	falsh_buffer[65]=chuhuanP[0];
	falsh_buffer[66]=chuhuanP[1];
	falsh_buffer[67]=chuhuanP[2];
	falsh_buffer[68]=chuhuanP[3];
	falsh_buffer[69]=chuhuanP[4];	
	falsh_buffer[70]=huanNum[2];
	falsh_buffer[71]=huanNum[3];
	falsh_buffer[72]=huanNum[4];
	falsh_buffer[73]=stopline_count_one;
  falsh_buffer[74]=stopline_count_two;
	falsh_buffer[75]=big_island_add;
	FLASH_EraseSector(223);
	FLASH_WriteSector(223, (const uint8 *) falsh_buffer,160,0);		
}
void ADC_read()
{
	uint8_t i=0;
	FLASH_ReadByte(233,16,(uint8_t *)ADC_buffer);
  for(i=0;i<ADN;i++)
	{
	 f_k[i] =ADC_buffer[i]; 	
	}
}
void ADC_write()
{
	uint8_t i=0;
  for(i=0;i<ADN;i++)
	{
	  ADC_buffer[i]=f_k[i]; 	
	}
	FLASH_EraseSector(233);
	FLASH_WriteSector(233,(const uint8 *) ADC_buffer,16,0);	
}
void  inherit()
{
	uint8_t i,y=0;
	for(i=1;i<10;i++)
	{
		for(y=0;y<ADN;y++)
	{
		AD_cunchu[i][y]=AD_cunchu[i-1][y];
	}
	inherit_uwb_Distance[i]=inherit_uwb_Distance[i-1];
	}
	AD_cunchu[0][0]=EV_VALUE1;
  AD_cunchu[0][1]=EV_VALUE2;
  AD_cunchu[0][2]=EV_VALUE3;
  AD_cunchu[0][3]=EV_VALUE4; 
  AD_cunchu[0][4]=EV_VALUE5;	
	AD_cunchu[0][5]=EV_VALUE6;
  AD_cunchu[0][6]=EV_VALUE7;
	inherit_uwb_Distance[0]=uwb_Distance;
}
void ADC_init()
{
	if(Car==1)
	{
	adc_init(ADC1_SE12);
	adc_init(ADC1_SE10);
	adc_init(ADC0_SE12);
	adc_init(ADC0_SE8);
	adc_init(ADC1_SE13);
	adc_init(ADC1_SE11);
	adc_init(ADC1_SE9);
	adc_init(ADC0_SE13);
	}
	else 	if(Car==0)
	{
	adc_init(ADC1_SE10);
	adc_init(ADC0_SE13);
	adc_init(ADC1_SE11);
	adc_init(ADC0_SE8);
	adc_init(ADC0_SE12);
	adc_init(ADC1_SE12);
	adc_init(ADC1_SE9);
	}
}
void ADC_Get()
{
  uint8_t num=0;
  int AD_ALL[8]={0,0,0,0,0,0,0},biggest[8],minist[8];
		if(Car==1)
	{
		for(num=0;num<20;num++)
	{
	LLeft[num]=adc_once(ADC1_SE12,ADC_10bit);
	Left[num]=adc_once(ADC1_SE10,ADC_10bit);
	Right[num]=adc_once(ADC0_SE12,ADC_10bit);
	RRight[num]=adc_once(ADC0_SE8,ADC_10bit);
  Center[num]=adc_once(ADC1_SE13,ADC_10bit);
	Lxie[num]=adc_once(ADC1_SE11,ADC_10bit);
	Rxie[num]=adc_once(ADC1_SE9,ADC_10bit);	
	UCenter[num]=adc_once(ADC0_SE13,ADC_10bit);
	}
				for(num=0;num<20;num++)
	{
			if(num==0)
		{
			biggest[0]=LLeft[num];
			biggest[1]=Left[num];
		  biggest[2]=Right[num];
			biggest[3]=RRight[num];
			biggest[4]=Center[num];
			biggest[5]=Lxie[num];
			biggest[6]=Rxie[num];
			biggest[7]=UCenter[num];
			minist[0]=LLeft[num];
			minist[1]=Left[num];
		  minist[2]=Right[num];
			minist[3]=RRight[num];	
      minist[4]=Center[num];
			minist[5]=Lxie[num];	
      minist[6]=Rxie[num];
      minist[7]=UCenter[num];			
		}
		else if(num>0)
		{
			if(LLeft[num]>biggest[0])
			{
			biggest[0]=LLeft[num];
			}
			if(Left[num]>biggest[1])
			{
			biggest[1]=Left[num];
			}
			if(Right[num]>biggest[2])
			{
			biggest[2]=Right[num];
			}
			if(RRight[num]>biggest[3])
			{
			biggest[3]=RRight[num];
			}
		  if(Center[num]>biggest[4])
			{
			biggest[4]=Center[num];
			}
			if(Lxie[num]>biggest[5])
			{
			biggest[5]=Lxie[num];
			}
			if(Rxie[num]>biggest[6])
			{
			biggest[6]=Rxie[num];
			}
			if(UCenter[num]>biggest[7])
			{
			biggest[7]=UCenter[num];
			}
			if(LLeft[num]<minist[0])
			{
			minist[0]=LLeft[num];
			}
			if(Left[num]<minist[1])
			{
			minist[1]=Left[num];
			}
			if(Right[num]<minist[2])
			{
			minist[2]=Right[num];
			}
			if(RRight[num]<minist[3])
			{
			minist[3]=RRight[num];
			}
			if(Center[num]<minist[4])
			{
			minist[4]=Center[num];
			}
			if(Lxie[num]<minist[5])
			{
			minist[5]=Lxie[num];
			}
			if(Rxie[num]<minist[6])
			{
			minist[6]=Rxie[num];
			}
			if(UCenter[num]<minist[7])
			{
			minist[7]=UCenter[num];
			}
	  }
	AD_ALL[0]+=LLeft[num];
	AD_ALL[1]+=Left[num];	
	AD_ALL[2]+=Right[num];
	AD_ALL[3]+=RRight[num];	
  AD_ALL[4]+=Center[num];
	AD_ALL[5]+=Lxie[num];
	AD_ALL[6]+=Rxie[num];
	AD_ALL[7]+=UCenter[num];
	}
	   /*****************去掉最大值最小值,求平均值**************/
		EV_VALUE1=(double)(AD_ALL[0]-biggest[0]-minist[0])/18;
	EV_VALUE2=(double)(AD_ALL[1]-biggest[1]-minist[1])/18;
	EV_VALUE3=(double)(AD_ALL[2]-biggest[2]-minist[2])/18;
  EV_VALUE4=(double)(AD_ALL[3]-biggest[3]-minist[3])/18;
	EV_VALUE5=(double)(AD_ALL[4]-biggest[4]-minist[4])/18;
	EV_VALUE6=(double)(AD_ALL[5]-biggest[5]-minist[5])/18;
	EV_VALUE7=(double)(AD_ALL[6]-biggest[6]-minist[6])/18;
	EV_VALUE8=(double)(AD_ALL[7]-biggest[7]-minist[7])/18;
//	{
//	EV_VALUE1=(double)(AD_ALL[0]-biggest[0]-minist[0])*f_k[0]/18000;
//	EV_VALUE2=(double)(AD_ALL[1]-biggest[1]-minist[1])*f_k[1]/18000;
//	EV_VALUE3=(double)(AD_ALL[2]-biggest[2]-minist[2])*f_k[2]/18000;
//  EV_VALUE4=(double)(AD_ALL[3]-biggest[3]-minist[3])*f_k[3]/18000;
//	EV_VALUE5=(double)(AD_ALL[4]-biggest[4]-minist[4])*f_k[4]/18000;
//	EV_VALUE6=(double)(AD_ALL[5]-biggest[5]-minist[5])*f_k[5]/18000;
//	EV_VALUE7=(double)(AD_ALL[6]-biggest[6]-minist[6])*f_k[6]/18000;
//	}
  if(EV_VALUE1<30&&EV_VALUE4<30)
	{
	motor_stop=1;
	}
  }
	else 	if(Car==0)
	{
			for(num=0;num<20;num++)
	{
    LLeft[num]=adc_once(ADC1_SE10,ADC_10bit);//4
		Left[num]=adc_once(ADC0_SE13,ADC_10bit);//3
		Right[num]=adc_once(ADC0_SE8,ADC_10bit);//0
		RRight[num]=adc_once(ADC1_SE9,ADC_10bit);	//1
		Center[num]=adc_once(ADC0_SE12,ADC_10bit);//2
		Lxie[num]=adc_once(ADC1_SE12,ADC_10bit);//6
		Rxie[num]=adc_once(ADC1_SE11,ADC_10bit);//5
	}
			for(num=0;num<20;num++)
	{
			if(num==0)
		{
			biggest[0]=LLeft[num];
			biggest[1]=Left[num];
		  biggest[2]=Right[num];
			biggest[3]=RRight[num];
			biggest[4]=Center[num];
			biggest[5]=Lxie[num];
			biggest[6]=Rxie[num];
			minist[0]=LLeft[num];
			minist[1]=Left[num];
		  minist[2]=Right[num];
			minist[3]=RRight[num];	
      minist[4]=Center[num];
			minist[5]=Lxie[num];	
      minist[6]=Rxie[num];			
		}
		else if(num>0)
		{
			if(LLeft[num]>biggest[0])
			{
			biggest[0]=LLeft[num];
			}
			if(Left[num]>biggest[1])
			{
			biggest[1]=Left[num];
			}
			if(Right[num]>biggest[2])
			{
			biggest[2]=Right[num];
			}
			if(RRight[num]>biggest[3])
			{
			biggest[3]=RRight[num];
			}
		  if(Center[num]>biggest[4])
			{
			biggest[4]=Center[num];
			}
			if(Lxie[num]>biggest[5])
			{
			biggest[5]=Lxie[num];
			}
			if(Rxie[num]>biggest[6])
			{
			biggest[6]=Rxie[num];
			}
			if(LLeft[num]<minist[0])
			{
			minist[0]=LLeft[num];
			}
			if(Left[num]<minist[1])
			{
			minist[1]=Left[num];
			}
			if(Right[num]<minist[2])
			{
			minist[2]=Right[num];
			}
			if(RRight[num]<minist[3])
			{
			minist[3]=RRight[num];
			}
			if(Center[num]<minist[4])
			{
			minist[4]=Center[num];
			}
			if(Lxie[num]<minist[5])
			{
			minist[5]=Lxie[num];
			}
			if(Rxie[num]<minist[6])
			{
			minist[6]=Rxie[num];
			}
	  }
	AD_ALL[0]+=LLeft[num];
	AD_ALL[1]+=Left[num];	
	AD_ALL[2]+=Right[num];
	AD_ALL[3]+=RRight[num];	
  AD_ALL[4]+=Center[num];
	AD_ALL[5]+=Lxie[num];
	AD_ALL[6]+=Rxie[num];
	}
	   /*****************去掉最大值最小值,求平均值**************/
	EV_VALUE1=(double)(AD_ALL[0]-biggest[0]-minist[0])/18;
	EV_VALUE2=(double)(AD_ALL[1]-biggest[1]-minist[1])/18;
	EV_VALUE3=(double)(AD_ALL[2]-biggest[2]-minist[2])/18;
  EV_VALUE4=(double)(AD_ALL[3]-biggest[3]-minist[3])/18;
	EV_VALUE5=(double)(AD_ALL[4]-biggest[4]-minist[4])/18;
	EV_VALUE6=(double)(AD_ALL[5]-biggest[5]-minist[5])/18;
	EV_VALUE7=(double)(AD_ALL[6]-biggest[6]-minist[6])/18;
  if(EV_VALUE1<30&&EV_VALUE4<30)
	{
	motor_stop=1;
	}
	}
}
void get_error()
{
	/*      第一段     */
    {
		lasterror=function_first;
    function_first=(sqrt(EV_VALUE1)-sqrt(EV_VALUE4))/(EV_VALUE1+EV_VALUE4)*1250*shuruP;
	  firstEC=function_first-lasterror;
		}
	/*      第二段     */
    {
		 last_secondE=function_second;
		if(EV_VALUE2+EV_VALUE3<800)
		{
			shurupiancha=(800-EV_VALUE2-EV_VALUE3);
			if(((EV_VALUE2<300&&EV_VALUE3<300)||EV_VALUE2+EV_VALUE3<500)&&zuohuan!=7&&youhuan!=7)
				function_second=(800-EV_VALUE2-EV_VALUE3);//*secP/20
			else function_second=(800-EV_VALUE2-EV_VALUE3);//
		}
			else 
			{
				function_second=0;
				shurupiancha=0;
			}			
			secondEC=function_second-last_secondE;			
		}
	/*      方向判断   */
		{
			
	    if(EV_VALUE2+EV_VALUE3>350 && EV_VALUE1+EV_VALUE4>320)  //
    {
      if(EV_VALUE1-EV_VALUE4+(EV_VALUE2-EV_VALUE3)/4>0)  
          direction=1;
      else if(EV_VALUE1-EV_VALUE4+(EV_VALUE2-EV_VALUE3)/4/4<0) 
          direction=-1;
    }		
		if(CNT<10)
		{
		if(EV_VALUE1-EV_VALUE4>0)
			direction=1;
		else if(EV_VALUE1-EV_VALUE4<0)
			direction=-1;
		}
		}
	/*      求偏差     */	
		{
		if(direction==1)
		{
		if(function_second-change_point>function_first)
		 { 
			 nowError=function_second;
			 DEVIATE=secondEC;	
		 }
		 else 
		 {
				nowError=function_first;
				DEVIATE=firstEC;	
		 }
		}
		else if(direction==-1)
		{
		if(function_second-change_point>-function_first)
		 { 
			 nowError=-function_second;
			 DEVIATE=-secondEC;	
		 }
		 else 
		 {
				nowError=function_first;
				DEVIATE=firstEC;		
		 }
		  
		
		}
		else 
		{
				nowError=function_first;
				DEVIATE=firstEC;	
		}
	  }
 /*       各种操作    */	
		if(CNT>200)
		{
			if((shizi==1&&zuohuan==0&&youhuan==0)||podao_flag==1||podao_flag==2||small_s==1)//
			{
				if(small_s==0)
				{
					nowError=nowError/2; 
					DEVIATE=DEVIATE/2;
				}		
			 else 
				{
					nowError=nowError/4; 
					DEVIATE=DEVIATE/4;
				}		
			}
			else if(out_huan_count>0&&out_huan_count<100)
			{
				nowError=nowError/2; 
				DEVIATE=DEVIATE/2;
			}	
			else if(longroad==1||zhidao==1)
			{
				nowError=nowError*0.9; 
				DEVIATE=DEVIATE*0.6;		
			}
			else if(shortroad==1)
			{
				nowError=nowError*0.9; 
				DEVIATE=DEVIATE*0.7;		
			}
		}
     
   

		/*                     双车各状态确定                     */
		if(dangche==0)
		{			
		/*               中间会车                               */
		{
		if(xuxianjilu==0)//第一次区分前后车
		{
			if(Wireless(RX,0)==2 && already_huiche ==0)
				already_huiche = 1;
		}	
		if(xuxianjilu==1)//进入中间会车区
		{
			  if(CarFlag==2)//确定前后车 中间会车后already_huiche一直是2
			{
				if(already_huiche==0)
				{
					NRF2401_SetMode(TX);
					CarFlag = 0;
				}
			  else if(already_huiche==1)
				{
					CarFlag = 1;	
				}
			}				
				if(CarFlag==0&&already_huiche==0)//前车
			{				 
				 if(huiche==0)//发一次 如果没有发送成功 下次中断继续发
				{					
					Wireless(TX,2);					
					gpio_init(A19,GPO,1);
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						NRF2401_SetMode(RX);
						huiche=1;	
						gpio_init(A19,GPO,0);
					}					
				}
        if(huiche	==1)			
				{
					if(Wireless(RX,0)==2)//后车到达收2
				{
				already_huiche=2;
				huichejuli=0;
				huiche=0;
				huicheNum++;
			  }	
			  }
			}						
			  if(CarFlag==1&&already_huiche==1)//后车
			{								
				if(uwb_Distance<huiche_Menu_uwb&&huichejuli==0)//根据uwb来确定后车是否已靠近前车
				{
					huichejuli=1;
					NRF2401_SetMode(TX);
				}
				if(huichejuli==1)
				{
					
					Wireless(TX,2);			
					gpio_init(A19,GPO,1);
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)	//发送成功		
					{
						already_huiche=2;		
						huichejuli=0;
				    huiche=0;
				    huicheNum++;
						gpio_init(A19,GPO,0);	
						NRF2401_SetMode(RX);					
					}					
				}
			}
		}
		}
	  /*                最后回车                              */
		{		
		if(already_huiche==2&&Stop_Line_Huiche==0&&xuxianjilu==1&&CarFlag ==2)//第二次区分前后车
		{
			 if(Wireless(RX,0)==3)
				Stop_Line_Huiche = 1;
		}
		if(xuxianjilu==2)//进入最终会车区
		{
			if(CarFlag==2)//确定前后车
			{
				if(Stop_Line_Huiche==0)
				{
					CarFlag = 0;
					NRF2401_SetMode(TX);
				}
			  else if(Stop_Line_Huiche==1)
				CarFlag = 1;	
		  }			
			if(CarFlag==0&&Stop_Line_Huiche==0)
			{				 
				 if(huiche==0)//发一次 没成功则下次中断继续发送
				{
					
					Wireless(TX,3);					
					gpio_init(A19,GPO,1);
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)
					{
						huiche=1;
						NRF2401_SetMode(RX);
					}
					gpio_init(A19,GPO,0);
				}	
        if(huiche==1)				
				{
				if(Wireless(RX,0)==3)
				{
					Stop_Line_Huiche=2;
					huichejuli=0;
					huiche=0;
					huicheNum++;
			  }
			  }
			}
			if(CarFlag==1&&Stop_Line_Huiche==1)
			{									
				if(uwb_Distance<huiche_Menu_uwb&&huichejuli==0)
				{
					huichejuli=1;
					NRF2401_SetMode(TX);
				}
				if(huichejuli==1)//发一次 没成功则下次中断继续发送
				{				
					Wireless(TX,3);			
					gpio_init(A19,GPO,1);
					if(NRF2401_SendData(NRF2401TXBuffer) == TX_OK)			
					{
						Stop_Line_Huiche=2;
						gpio_init(A19,GPO,0);	
						NRF2401_SetMode(RX);			
						huichejuli=0;
			      huiche=0;
			      huicheNum++;					
					}					
				}
			}	
		}	  
	  }		
		}		
		/*                     双车打脚策略	                      */
		if(dangche==0)
		{
		/*    起始会车          */
		{
			if(CNT<100)
		{
			nowError-=750;
		}
		else 
		{
			if(750-(CNT-100)*10>0)
			{
				nowError-=(750-(CNT-100)*10);
			}				
		}
		}  					
		/*    中间会车          */
		if(xuxianjilu==1)
		{		
    if(CarFlag==0)	//前车操作		
		{
			if(already_huiche==0)//停的过程中直接给最大偏差
		{
		nowError-=Car0_second_error;		
		}
		else if(already_huiche==2)//复位过程缓慢复位
		{
			huiche_second_count++;
			if(Car0_second_error-huiche_second_count*10>0)
			{
				nowError-=(Car0_second_error-huiche_second_count*10);
			}
      else	
			CarFlag=2;				
		}	
	  }
		else if(CarFlag==1)//若为后车
		{
			if(already_huiche==1)//一开始先给最大偏差 最大偏差较小
		{
		nowError-=Car1_second_error;		
		}
		else if(already_huiche==2)//复位过程缓慢复位
		{
			huiche_second_count++;
			if(Car1_second_error-huiche_second_count*10>0)
			{
				nowError-=(Car1_second_error-huiche_second_count*10);
			}
      else	
			CarFlag=2;				
		}
		}					
		}
		/*    最后会车          */
		else if(xuxianjilu==2)
		{
    if(CarFlag==0)	//前车操作		
		{
		   if(Stop_Line_Huiche==0)//停的过程中直接给最大偏差 最大偏差最大
			{
			nowError-=Car0_third_error;		
			}
			else if(Stop_Line_Huiche==2)//复位过程缓慢复位  
			{
				huiche_third_count++;
				if(Car0_third_error-huiche_third_count*10>0)
				{
					nowError-=(Car0_third_error-huiche_third_count*10);
				} 				
			}	
	  }
    else if(CarFlag==1)	//后车操作		
		{
		   if(Stop_Line_Huiche==1)//停的过程中直接给最大偏差 最大偏差最大
			{
			nowError-=Car1_third_error;		
			}
			 else if(Stop_Line_Huiche==2)//复位过程缓慢复位 
			{
				huiche_third_count++;
				if(Car1_third_error-huiche_third_count*10>0)
				{
					nowError-=(Car1_third_error-huiche_third_count*10);
				}			
			}	
	  }		
		}		
		}
		
		
				
		/*      求舵机值  */ 
		{
	  if((zuohuan==0&&youhuan==0)||zuohuan==3||youhuan==3)
		{
		kp=Fuzzy_control(shurupiancha,secondEC,1,0);
	  kd=Fuzzy_control(shurupiancha,secondEC,0,0);
		}
		else //出环口
		{
		kp=Fuzzy_control(nowError,DEVIATE,1,0);
	  kd=Fuzzy_control(nowError,DEVIATE,0,0);
		}
		if(direction==1)
		duoji_InitalDuty=duojiZhongZhi+nowError*kp*LP/10000.0+kd*DEVIATE*D/100.0;
		else if(direction==-1)
		duoji_InitalDuty=duojiZhongZhi+nowError*kp*RP/10000.0+kd*DEVIATE*D/100.0;
		else //预防特殊情况的出现
		duoji_InitalDuty=duojiZhongZhi+nowError*kp*LP/10000.0+kd*DEVIATE*D/100.0;
		}
}
void track_judge()//判定赛道类型
{
	/*          十字口判断及计数                   */
  {
			if(EV_VALUE5>1020&&zuohuan==0&&youhuan==0)//十字判定 计数
			{
				if(shizi==0)
				{
				shizicount++;
				}
				shizi=1;
			}
			if(shizi==1&&EV_VALUE5<600)//此处的600待定
			{
				shizi=0;
			}
  }
	
	
	/*          采用状态机方式过环                
  1: 入环前 识别出方向
  2：单电感打脚
  3：环内正常跑
	4：出环 单电感打脚
	5：切线上 正常跑
	*/
	{
	/*              起始判环                       */
	{
		if(youhuan==0&&zuohuan==0&&podao_flag==0&&shizi==0)
	{
		/*              环岛状态1 判出环及其方向              */
		if(EV_VALUE1>500&&EV_VALUE2>500&&EV_VALUE3>500&&EV_VALUE4>500&&EV_VALUE5>80)
		{			
			if(EV_VALUE6-EV_VALUE7>100&&EV_VALUE6>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				zuohuan=1;        
        huanlarge=huanNum[huancount];		
				huancount++;	
        huanallcount++;	
				judge_huan=0;
				}					
			}
			else if(EV_VALUE6-EV_VALUE7<-100&&EV_VALUE7>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				youhuan=1;	
        huanlarge=huanNum[huancount];		
				huancount++;
        huanallcount++;
				judge_huan=0;
				}					
			}
		}
		else  if(EV_VALUE1>700&&EV_VALUE2>700&&EV_VALUE3>400&&EV_VALUE4>400&&EV_VALUE5>80)
		{			
			if(EV_VALUE6-EV_VALUE7>200&&EV_VALUE6>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				zuohuan=1;        
        huanlarge=huanNum[huancount];		
				huancount++;	
        huanallcount++;	
				judge_huan=0;
				}			
			}
			else if(EV_VALUE6-EV_VALUE7<-200&&EV_VALUE7>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				youhuan=1;	
        huanlarge=huanNum[huancount];		
				huancount++;
        huanallcount++;
				judge_huan=0;
				}		
			}
		}
		else  if(EV_VALUE1>400&&EV_VALUE2>400&&EV_VALUE3>700&&EV_VALUE4>700&&EV_VALUE5>80)
		{			
			if(EV_VALUE6-EV_VALUE7>200&&EV_VALUE6>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				zuohuan=1;        
        huanlarge=huanNum[huancount];		
				huancount++;	
        huanallcount++;	
				judge_huan=0;
				}			
			}
			else if(EV_VALUE6-EV_VALUE7<-200&&EV_VALUE7>400)
			{
				judge_huan++;
				if(judge_huan>3)
				{
				youhuan=1;	
        huanlarge=huanNum[huancount];		
				huancount++;
        huanallcount++;
				judge_huan=0;
				}			
			}
		}
		else judge_huan=0;
	}	
	
  }
	/*               左环                          */
	{
	if(zuohuan==1)
	{
	  if(EV_VALUE7-EV_VALUE6>-100)
			if(EV_VALUE7<AD_cunchu[0][6])
			zuohuan=2;//开始打脚 环状态2
	}	
	else if(zuohuan==2)//环岛判断
	 {
    in_island_count++;	
   if(in_island_count>120)
	 {
		 in_island_count=0;
		 zuohuan=3;
	 }
//		if(nowError>340)					
//				zuohuan=3;//在环内						
//		 else if(EV_VALUE4<200&&EV_VALUE1>400)
//		{
//			{
//				zuohuan=3;//在环内					
//			}
//		}
	 }
	 
	 else if(zuohuan==4)
	{
	   out_island_count++;
		if(out_island_count>huanNum[huancount-1])
		{
		zuohuan=5;
		out_island_count=0;
		}
	}
	 
	 if(zuohuan==5)
	{
	if(EV_VALUE1<550&&EV_VALUE2<550&&EV_VALUE3<550&&EV_VALUE4<550)
	{
		in_island_count=0;
		zuohuan=0;
		jiluhuan=0;
		out_huan_count=0;
	}
	}
  }
	/*               右环                          */
	{
		if(youhuan==1)
	{
		if(EV_VALUE6-EV_VALUE7>-100)
			if(EV_VALUE6<AD_cunchu[0][5])
		youhuan=2;
	  if(EV_VALUE6+EV_VALUE7<700&&EV_VALUE6<AD_cunchu[0][5]&&EV_VALUE7<AD_cunchu[0][6])
			youhuan=2;	
	}
  else if(youhuan==2)
	 {	
    in_island_count++;	
   if(in_island_count>120)
	 {
		 youhuan=3;	
		 in_island_count=0;
	 }		 
//			if(nowError<-340)
//			{
//					{
//						yith=1;
//						youhuan=3;				
//					}
//			}
//       else if(EV_VALUE1<200&&EV_VALUE4>400)
//			{
//					{
//						erth=1;
//						youhuan=3;//在环内					
//					}
//			}			
//	 
	 
	 }
	else if(youhuan==4)
	{
	   out_island_count++;
		if(out_island_count>huanNum[huancount-1])
		{
		youhuan=5;
		out_island_count=0;
		}
	}
	 if(youhuan==5)
	{
	if(EV_VALUE1<550&&EV_VALUE2<550&&EV_VALUE3<550&&EV_VALUE4<550)  
	{
		youhuan=0;
		jiluhuan=0;
		out_huan_count=0;
		in_island_count=0;
	}
	}
  }	
  	if(quanNum>1)
	{
		if(huancount==huannum&&(zuohuan==0&&youhuan==0))
		{
			huancount=0;
		}
	}
	
		if(zuohuan==5||youhuan==5)
	{
   out_huan_count++;
	}	
	
	}
}


void Servo_pid_cal()
{
	/*             环特殊处理                      */	
	{
		if(zuohuan==2)
	{
		if(EV_VALUE2>EV_VALUE1)
		{
			if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE2*ruhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE2*ruhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE1*ruhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE1*ruhuanP[huancount-1]/100;
		}
	}
	else if(zuohuan==3)
	{
		if(EV_VALUE2>EV_VALUE1)
		{
			if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE2*inhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE2*inhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE1*inhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE1*inhuanP[huancount-1]/100;
		}
	}
	else if(zuohuan==4||zuohuan==6)
	{
		if(EV_VALUE2>EV_VALUE1)
		{
			if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE2*chuhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE2*chuhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty<duojiZhongZhi+EV_VALUE1*chuhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi+EV_VALUE1*chuhuanP[huancount-1]/100;
		}
	}
  else if(youhuan==2)
	{
		if(EV_VALUE3>EV_VALUE4)
		{
			if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE3*ruhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE3*ruhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE4*ruhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE4*ruhuanP[huancount-1]/100;
		}
	}
	else if(youhuan==3)
	{
				if(EV_VALUE3>EV_VALUE4)
		{
			if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE3*inhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE3*inhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE4*inhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE4*inhuanP[huancount-1]/100;
		}
	}
  else if(youhuan==4||youhuan==6)
	{
		if(EV_VALUE3>EV_VALUE4)
		{
			if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE3*chuhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE3*chuhuanP[huancount-1]/100;
		}
		else 
		{
	    if(duoji_InitalDuty>duojiZhongZhi-EV_VALUE4*chuhuanP[huancount-1]/100)
	    duoji_InitalDuty=duojiZhongZhi-EV_VALUE4*chuhuanP[huancount-1]/100;
		}
	} 
		
  }
	servo_Flush();
}
void motor_pwm_set()  //起始限幅改变试试
{
	/*                          差速                           */
	{
				if(duoji_InitalDuty<duojiZhongZhi)
				{
					if(duoji_InitalDuty-duoji_InitalDuty_Min>0)
				aimyou=(double)((duoji_InitalDuty-duoji_InitalDuty_Min)*straightSpeed*chasup)/(duojiZhongZhi-duoji_InitalDuty_Min)/20.0+(1-(double)chasup/20.0)*straightSpeed;
					else 
				aimyou=(1-(double)chasup/20.0)*straightSpeed;
				aimzuo=straightSpeed;
				}
				else if(duoji_InitalDuty>duojiZhongZhi)
				{
					if(duoji_InitalDuty_Max-duoji_InitalDuty>0)
				aimzuo=(double)((duoji_InitalDuty_Max-duoji_InitalDuty)*straightSpeed*chasup)/(duoji_InitalDuty_Max-duojiZhongZhi)/20.0+(1-(double)chasup/20.0)*straightSpeed;
					else 
				aimzuo=(1-(double)chasup/20.0)*straightSpeed;
				aimyou=straightSpeed;
				}
				else 
				{
					aimyou=straightSpeed;
					aimzuo=straightSpeed;
				}				
	}
  /*                          由赛道类型处理                 */
	{
		if(longroad==1)//不给差速
		{
		aimyou=straightSpeed+longadd;
		aimzuo=straightSpeed+longadd;
		}
		else if(dizen==1)
		{
		  dizen_count++;
			if(long_stop_speed-dizen_count*0.5>0)
			{
				aimyou-=	long_stop_speed-dizen_count*0.5;
				aimzuo-=	long_stop_speed-dizen_count*0.5;
			}
			else 
			{
			dizen=0;
			dizen_count=0;
			}	
		}
		else if(shortroad==1&&cut_filed==0)
		{
		aimyou=straightSpeed+shortadd;
		aimzuo=straightSpeed+shortadd;	
		}
		else if((zuohuan>2||youhuan>2)&&out_huan_count<80)
		{
		  if(ruhuanP[huancount-1]<=5)
			{		
				aimyou+=big_island_add;
				aimzuo+=big_island_add;						
			}		
		}
		if(zuohuan==1||youhuan==1)
		{
		aimyou=straightSpeed-islandcut;
		aimzuo=straightSpeed-islandcut;			
		}
		else if(podao_flag>0)
		{
		aimyou=straightSpeed-podaocut;
		aimzuo=straightSpeed-podaocut;
		}
		else if(cut_filed==1)
		{	
		aimyou-=long_stop_speed;
		aimzuo-=long_stop_speed;			
		}
		if(longroad==1||shortroad==1)
		{
		gpio_init(A19,GPO,1);
		}
		else 
		gpio_init(A19,GPO,0);
	}
  /*********                  双车速度策略            **********/ 
  if(dangche==0)
	{			
		/*             起始发车不给差速              */		
		if(CNT<200)				
	{
		aimyou=straightSpeed;
		aimzuo=straightSpeed;
	}	
	if(xuxianjilu==1)//中间会车操作
	{
	  if(CarFlag==0)//若为前车
		{
			if(already_huiche==0)//前车靠边停 
			{
				double cutP=(double)(straightSpeed)/Car0_MiddleStop_Count;//根据设定减场次确定p 设定减场稍微给大点以使前车能够稍微远
			  huichecount++;
		    if(huichecount<Car0_MiddleStop_Count)
				{
					aimyou=straightSpeed-cutP*huichecount;
					aimzuo=straightSpeed-cutP*huichecount;				
				}
				else
				{
				aimyou=0;
				aimzuo=0;
				}
		  }		
     else if(	already_huiche==2)//如果前车重新发车 暂时不作处理（如果在弯道会车应该要处理）
		 {
		 
		 
		 }
		}
		else if(CarFlag==1)//后车全过程只作不给差速处理
		{
			aimyou=straightSpeed;
			aimzuo=straightSpeed;									
		}	
	}
	if(xuxianjilu==2)//最后会车操作
	{
	  if(CarFlag==0)//若为前车
		{
			if(Stop_Line_Huiche==0)//前车靠边停 
			{
				double cutP=(double)(straightSpeed)/Car0_EndStop_Count;//根据设定减场次Car0_EndStop_Count确定p 设定减场稍微小 以使前车停在起跑线前面
			  tingchecount++;
		    if(tingchecount<Car0_EndStop_Count)
				{
					aimyou=straightSpeed-cutP*tingchecount;
					aimzuo=straightSpeed-cutP*tingchecount;				
				}
				else
				{
				aimyou=0;
				aimzuo=0;
				}
		  }		
      else if(Stop_Line_Huiche==2)//开始停车
		 {
		    stop_count++;
			 if(stop_count>stopline_count_one)
			 {
				 aimzuo=0;
				 aimyou=0;
			 }
		 }
		}
		else if(CarFlag==1)//后车全过程只作不给差速处理
		{
			if(Stop_Line_Huiche==1)//后车不给差速
			{
			aimyou=straightSpeed;
			aimzuo=straightSpeed;	
			}		
      else if(Stop_Line_Huiche==2)//开始停车
			{
		    stop_count++;
			 if(stop_count>stopline_count_two)
			 {
				 aimzuo=0;
				 aimyou=0;
			 }	
			}			
		}
	}
  }	
	
	
	/*            单车起跑线停车处理（双车不判起跑线）        */
	if(stop_side==1&&dangche!=0)
	{
	   stop_count++;
			if(stop_count>stop_Menu_count)
			{	
		aimzuo=0;
		aimyou=0;
	    }			
	}
	/*                         各种保护                         */
	if(motor_stop==1&&protect_flag==1)
	{		
		aimyou=0;
		aimzuo=0;	
    if(dangche==0)
		{
			NRF2401_SetMode(TX);
			Wireless(TX,4);			
		}			
	}	
	if(motor_stop==2&&protect_flag==1)
	{		
		aimyou=0;
		aimzuo=0;	
	}	
	/*                         模糊控制电机输出                   */
	Encodercontrol_motor();
	/*                          停车时防止倒退                    */
	if(xuxianjilu>0&&LPulse<3&&Dulse<3&&LPulse>-3&&Dulse>-3)
	{	
	if(motor_left<0)
		motor_left=0;
		if(motor_right<0)
		motor_right=0;
	}
	/*                          更新电机占空比                     */
	motor_flush();
}

void OLED_show()
{
			OLED_Write_Num4(0, 0,CarFlag);
		OLED_Write_Num4(0, 2,already_huiche);
		OLED_Write_Num4(0, 4,aimyou);
		OLED_Write_Num4(0, 6,huichejuli);
	
		OLED_Write_Num4(0, 0,EV_VALUE1);
		OLED_Write_Num4(0, 2,EV_VALUE2);
		OLED_Write_Num4(0, 4,EV_VALUE3);
		OLED_Write_Num4(0, 6,EV_VALUE4);
		OLED_Write_Num4(6, 0,EV_VALUE6);
		OLED_Write_Num4(6, 2,EV_VALUE7);	
	  OLED_Write_Num4(6, 4,EV_VALUE5);
//	OLED_Write_Num4(12,4,duojiZhongZhi+nowError*kp*RP/10000.0+kd*DEVIATE*D/100.0);
//	OLED_Write_Char(14,0,zuohuan);
//	OLED_Write_Char(14,2,youhuan);
	OLED_Write_Num4(12,0,uwb_Distance);
}
void mode0()//纯电磁
{
  OLED_Clear();
	ADC_read();
	pid_data();
	enable_irq(PIT1_IRQn);//打开pit0的中断开关	
	while(1)
	{
			if(oled_flag==1)
			OLED_show();
	}
}
void mode1()//显示图像 sd卡
{ 
	ADC_read();//读取归一化值
	pid_data();//编码器pi	
	camera_init();
	set_irq_priority(PIT1_IRQn,0);
	enable_irq(PIT1_IRQn);//打开pit1的中断开关
	if(sd_Flag==1)
	{
    My_SdCard_Init(sd_save);
	}
	OLED_Clear();
	while(1)
	{
	  if(mt9v032_finish_flag)
		{ 
			Pixels_get();//获取二值化数组
			if(camera_Flag==1)
			{
			DisplayImage_WithOLED();
			}
			if(sd_Flag==1)//在环的最后一圈存SD卡&&(huanallcount>huannum*(quanNum-1)||quanNum==1)
			{
				My_SdCard_Save();
			}
			 SignalProcess();
			 mt9v032_finish_flag = 0;
		}
	if(oled_flag==1)
			OLED_show();
  }
}
void mode2()//实时传输
{
	pid_data();
  My_Tranfer_Init();
	camera_init();
	while(1)
	{
		if(mt9v032_finish_flag)
		{
			//oled显示	
			DisplayImage_WithOLED();
			My_LiveTransfer();
			mt9v032_finish_flag = 0;
		}
	}
}
void mode3()//读sd卡
{
	pid_data();
	uart_rx_irq_dis(uart0);
	My_Tranfer_Init();
	My_SdCard_Init(sd_read);
	while(1)
	{
		My_SdCard_Read();
		DisplayImage_WithOLED();
	  systick_delay_ms(50);
	}
}
void mode4()//防止刷flash
{
	LP=38;
	D=34;
	shuruP=28;
	RP=38;
	protect_flag=1;
	duojiZhongZhi=697;
	sdMode=1;
  straightSpeed=76;
	chuhuanleftP=0;	
  falsh_buffer[9]=chasup;
  chuhuanrightP=0;	
	
	camera_Flag=0;	
  oled_flag=0;
  secP=40;
	shiziH=0;
	shiziL=0;	
	image_chuhuanwide=0;
	change_point=0;
	sd_Flag=1;
	imageP=0;
	imageD=0;
	
	quanNum=1;
	huannum=3;
	quanDirection=0;
	yuzhi=95;
	encoder_LP=40;
	encoder_RP=40;
  encoder_LI=30;
  encoder_RI=30;
	
	long_stop_filed=0;
	long_stop_speed=0;
	out_island_line=10;
	Car0_second_error=900;
	Car1_second_error=600;
	dangche=0;
	binaryzation=1;
	Car0_MiddleStop_Count=120;
	Car0_EndStop_Count=110;
	Car0_third_error=1000;
	Car1_third_error=600;
	huiche_Menu_uwb=1200;
	aheaderror=0;
	encoderI=30;
	encoderP=40;
	out_island=0;
	Car=0;
	
	
	Motorshuru=85;
	MotorshuchuP=10;
	MotorshuchuD=25;
	longadd=0;
  shortadd=0;
	islandcut=0;
	podaocut=0;
	
	ruhuanP[0]=6;
	ruhuanP[1]=8;
	ruhuanP[2]=8;
	ruhuanP[3]=8;
	ruhuanP[4]=8;
	
	inhuanP[0]=6;
	inhuanP[1]=9;
	inhuanP[2]=9;
	inhuanP[3]=9;
	inhuanP[4]=9;

	chuhuanP[0]=7;
	chuhuanP[1]=10;
	chuhuanP[2]=10;
	chuhuanP[3]=10;
	chuhuanP[4]=10;
	
	huanNum[0]=52;
	huanNum[1]=50;
	huanNum[2]=50;
	huanNum[3]=50;
	huanNum[4]=50;
	
	stopline_count_one=110;
	stopline_count_two=50;
	
	falsh_buffer[0]=LP;
	falsh_buffer[1]=D;
	falsh_buffer[2]=shuruP;
	falsh_buffer[3]=RP;
	falsh_buffer[4]=protect_flag;
	falsh_buffer[5]=duojiZhongZhi;
	falsh_buffer[6]=sdMode;
  falsh_buffer[7]=straightSpeed;
	falsh_buffer[8]=chuhuanleftP;	
  falsh_buffer[9]=chasup;
  falsh_buffer[10]=chuhuanrightP;		
	
  falsh_buffer[11]=camera_Flag;	
  falsh_buffer[12]=oled_flag;
  falsh_buffer[13]=secP;
	falsh_buffer[14]=shiziH;
	falsh_buffer[15]=shiziL;	
	falsh_buffer[16]=image_chuhuanwide;
	falsh_buffer[17]=change_point;
	falsh_buffer[19]=sd_Flag;
	falsh_buffer[20]=imageP;
	falsh_buffer[21]=imageD;
  falsh_buffer[22]=quanNum;
	falsh_buffer[23]=huannum;
	falsh_buffer[24]=quanDirection;
	falsh_buffer[25]=yuzhi;
	falsh_buffer[26]=encoder_LP;
	falsh_buffer[27]=encoder_RP;
  falsh_buffer[28]=encoder_LI;
  falsh_buffer[29]=encoder_RI;
	falsh_buffer[30]=huanNum[0];
	falsh_buffer[31]=huanNum[1];
	falsh_buffer[32]=long_stop_filed;
	falsh_buffer[33]=long_stop_speed;
	falsh_buffer[34]=out_island_line;
	falsh_buffer[18]=Car0_second_error;
	falsh_buffer[35]=Car1_second_error;
	falsh_buffer[36]=dangche;
	falsh_buffer[37]=binaryzation;
	falsh_buffer[38]=Car0_MiddleStop_Count;
	falsh_buffer[39]=Car0_EndStop_Count;
	falsh_buffer[40]=Car0_third_error;
  falsh_buffer[41]=Car1_third_error;
	falsh_buffer[42]=huiche_Menu_uwb;
	falsh_buffer[43]=aheaderror;
	falsh_buffer[44]=encoderI;
	falsh_buffer[45]=encoderP;
	falsh_buffer[46]=out_island;
	falsh_buffer[47]=Car;
	falsh_buffer[48]=Motorshuru;
	falsh_buffer[49]=MotorshuchuP;
	falsh_buffer[50]=MotorshuchuD;
	falsh_buffer[51]=longadd;
  falsh_buffer[52]=shortadd;
  falsh_buffer[53]=islandcut;
	falsh_buffer[54]=podaocut;
  falsh_buffer[55]=ruhuanP[0];
	falsh_buffer[56]=ruhuanP[1];
	falsh_buffer[57]=ruhuanP[2];
	falsh_buffer[58]=ruhuanP[3];
	falsh_buffer[59]=ruhuanP[4];
	
	falsh_buffer[60]=inhuanP[0];
	falsh_buffer[61]=inhuanP[1];
	falsh_buffer[62]=inhuanP[2];
	falsh_buffer[63]=inhuanP[3];
	falsh_buffer[64]=inhuanP[4];
	
	falsh_buffer[65]=chuhuanP[0];
	falsh_buffer[66]=chuhuanP[1];
	falsh_buffer[67]=chuhuanP[2];
	falsh_buffer[68]=chuhuanP[3];
	falsh_buffer[69]=chuhuanP[4];	
	falsh_buffer[70]=huanNum[2];
	falsh_buffer[71]=huanNum[3];
	falsh_buffer[72]=huanNum[4];
	falsh_buffer[73]=stopline_count_one;
  falsh_buffer[74]=stopline_count_two;
	FLASH_EraseSector(223);
	FLASH_WriteSector(223, (const uint8 *) falsh_buffer,160,0);			
	 while(1);
}
void mode5()//调编码器
{
  OLED_Clear();
	pid_data();
	while(1)
	{
			Encoder_valueGet();
			if(LPulse<0)
		{
			OLED_Write_Char(0, 0,'-');
			OLED_Write_Num4(2, 0,-LPulse);
		}
		else 
		{
			OLED_Write_Char(0,0,36);
			OLED_Write_Num4(2,0,LPulse);
		}
				if(Dulse<0)
		{
			OLED_Write_Char(0, 2,'-');
			OLED_Write_Num4(2, 2,-Dulse);
		}
		else 
		{
			OLED_Write_Char(0,2,36);
			OLED_Write_Num4(2,2,Dulse);
		}
	}
}
void mode6()//煲舵机
{
	while(1)
	{
		ftm_pwm_duty(servo_Port,servo_Ch,670);
		systick_delay_ms(100);
	}
}	
void mode7()//归一化
{
	OLED_Clear();
	pit_init_ms(pit0,5);//5ms
	enable_irq(PIT0_IRQn);
	static int write_flag=0; 
  while(1)
	{
		  if (Key_Add == 0) //加
    {
      systick_delay_ms(80);
      if (Key_Add == 0)
        swm++;
    }
		  if(write_flag == 0 && swm==1)  //水平电感,斜电感
    {       
      f_k[0] = 400 * precision / EV_VALUE1;
			f_k[1] = 400 * precision / EV_VALUE2;
			f_k[2] = 400 * precision / EV_VALUE3;
			f_k[3] = 400 * precision / EV_VALUE4;
			f_k[5] = 250 * precision / EV_VALUE6;
			f_k[6] = 250 * precision / EV_VALUE7;
			ADC_write(); //写入flash
			write_flag = 1;
			swm=0;     
    }
    if(write_flag == 1 && swm==2)//竖直电感
    {
        f_k[4] = 1023 * precision / EV_VALUE5;            
        ADC_write(); //写入flash
        write_flag = 2;
        swm=0;
    }
	OLED_Write_Num2(13,0,swm);
	OLED_Write_Num4(0, 0,EV_VALUE1);
	OLED_Write_Num4(0, 2,EV_VALUE2);
	OLED_Write_Num4(0, 4,EV_VALUE3);
	OLED_Write_Num4(0, 6,EV_VALUE4);
	OLED_Write_Num4(6, 0,EV_VALUE6);
	OLED_Write_Num4(6, 2,EV_VALUE7);	
	OLED_Write_Num4(6, 4,EV_VALUE5);
	OLED_Write_Num4(6, 6,EV_VALUE8);
	OLED_Write_Num4(12, 2,f_k[0]);
	OLED_Write_Num4(12, 4,f_k[1]);
	OLED_Write_Num4(12, 6,f_k[2]);	 	
	}
}
void mode8()//纯摄像头
{

	camera_init();
		OLED_Clear();

	while(1)
	{
	  if(mt9v032_finish_flag)
		{ 
      Pixels_get();
			DisplayImage_WithOLED();
		
			mt9v032_finish_flag = 0;
		}
	}
}
void mode9()
{
	OLED_Clear();
	while(1)
	{
	aimzuo=change_point;
	aimyou=change_point;
	Encoder_valueGet();
	Encodercontrol_motor();
	motor_flush();
		OLED_Write_Num3(0,0,LPulse);
		OLED_Write_Num3(0,2,Dulse);
		
	}
	//初始化sd卡
	//MySD_Init();
//	Data_Uart_Init_zhufei();
//	camera_init();
//	Send_Image_zhufei_init();
//	OLED_Write_String(0,0,"init ok");
//	printf("camera init ok\n");
//	while(1)
//	{
//		if(mt9v032_finish_flag)
//		{
//			//oled显示
//			DisplayImage_WithOLED();
//			//SD_Write();
//			Send_Image_zhufei();
//			//Live_Transmission();
//			//Send_Image();//上位机发送
//			mt9v032_finish_flag = 0;
//		}
//	}
}

void menu()
{
	menu_read();//读取菜单参数
	servo_Init();//舵机初始化
	EncoderInit();//编码器初始化	
	while(1)
	{			
		 menu_Init();
		 systick_delay_ms(100);
    while(Menu_Mode == 1)
		{	
      menu_write();//写入菜单参数				
			if(quanDirection==1&&huannum>0)//至少有一个环 防止数组溢出
	{
		for(uint8_t i=0;i<huannum/2;i++)
		{
			midruhuanP[i]=ruhuanP[i];
			midinhuanP[i]=inhuanP[i];
			midchuhuanP[i]=chuhuanP[i];
			midhuanNum[i]=huanNum[i];
			
			ruhuanP[i]=ruhuanP[huannum-1-i];
			inhuanP[i]=inhuanP[huannum-1-i];
			chuhuanP[i]=chuhuanP[huannum-1-i];
			huanNum[i]=huanNum[huannum-1-i];
			
			ruhuanP[huannum-1-i]= midruhuanP[i];
			inhuanP[huannum-1-i]=midinhuanP[i];
			chuhuanP[huannum-1-i]=midchuhuanP[i];
			huanNum[huannum-1-i]=midhuanNum[i];
		}
	}	

			switch(sdMode)//增加while(1) 防止无限刷flash
			{			
				case 0:mode0();
				break;	
				case 1:mode1();
				break;
				case 2:mode2();
				break;			
				case 3:mode3();
				break;	
			  case 4:mode4();//防止刷flash
				break;
				case 255:mode4();//防止刷flash
				break;
				case 5:mode5();
				break;
				case 6:mode6();
				break;		
				case 7:mode7();
				break;	
				case 8:mode8();
				break;	
				case 9:mode9();
				break;				
				default : while(1);
			}
			while(1);
		}
		while(flash_flag == 1)
		{		
      menu_write();
			while(1);
		}	
	}
}

int main(void)
{
	all_Init();//初始化所有模块
  menu();
}

