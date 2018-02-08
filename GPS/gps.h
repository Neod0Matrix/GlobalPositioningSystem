#pragma once
#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//GPS底层驱动库 S1216F8-BD

#define __GPS_Model__			"S1216F8-BD"	//GPS型号
#define Support_Satellite_Num	12u				//定义支持的卫星数量

//GPS NMEA-0183协议重要参数结构体定义
//卫星信息
typedef __packed struct
{
    u8 	num;					//卫星编号
    u8 	eledeg;					//卫星仰角
    u16 azideg;					//卫星方位角
    u8 	sn;						//信噪比
} nmea_slmsg;

//北斗 NMEA-0183协议重要参数结构体定义
//卫星信息
typedef __packed struct
{
    u8 	beidou_num;				//卫星编号
    u8 	beidou_eledeg;			//卫星仰角
    u16 beidou_azideg;			//卫星方位角
    u8 	beidou_sn;				//信噪比
} beidou_nmea_slmsg;

//UTC时间信息
typedef __packed struct
{
    u16 year;					//年份
    u8 	month;					//月份
    u8	date;					//日期
    u8 	hour; 					//小时
    u8 	min; 					//分钟
    u8 	sec; 					//秒钟
} nmea_utc_time;

//NMEA 0183 协议解析后数据存放结构体
typedef __packed struct
{
    u8 					svnum;	//可见GPS卫星数
    u8 					beidou_svnum;//可见GPS卫星数
    nmea_slmsg 			slmsg[Support_Satellite_Num];//最多12颗GPS卫星
    beidou_nmea_slmsg 	beidou_slmsg[Support_Satellite_Num];
    nmea_utc_time 		utc;	//UTC时间
    u32 				latitude;//纬度 分扩大100000倍,实际要除以100000
    u8 					nshemi;	//北纬/南纬,N:北纬;S:南纬
    u32 				longitude;//经度 分扩大100000倍,实际要除以100000
    u8 					ewhemi;	//东经/西经,E:东经;W:西经
    u8 					gpssta;	//GPS状态:0,未定位;1,非差分定位;2,差分定位;6,正在估算.
    u8 					posslnum;//用于定位的GPS卫星数,0~12.
    u8 					possl[Support_Satellite_Num];//用于定位的卫星编号
    u8 					fixmode;//定位类型:1,没有定位;2,2D定位;3,3D定位
    u16 				pdop;	//位置精度因子 0~500,对应实际值0~50.0
    u16 				hdop;	//水平精度因子 0~500,对应实际值0~50.0
    u16 				vdop;	//垂直精度因子 0~500,对应实际值0~50.0
    int 				altitude;//海拔高度,放大了10倍,实际除以10.单位:0.1m
    u16 				speed;	//地面速率,放大了1000倍,实际除以10.单位:0.001公里/小时
} nmea_msg;

//SkyTra S1216F8 配置波特率结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0004；
    u8 	id;             		//ID，固定为0X05
    u8 	com_port;       		//COM口，固定为0X00，即COM1
    u8 	Baud_id;      		 	//波特率（0~8,4800,9600,19200,38400,57600,115200,230400,460800,921600）
    u8 	Attributes;     		//配置数据保存位置 ,0保存到SRAM，1保存到SRAM&FLASH，2临时保存
    u8 	CS;             		//校验值
    u16 end;            		//结束符:0X0D0A
} SkyTra_baudrate;

//SkyTra S1216F8 配置输出信息结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0009；
    u8 	id;             		//ID，固定为0X08
    u8 	GGA;            		//1~255（s）,0:disable
    u8 	GSA;            		//1~255（s）,0:disable
    u8 	GSV;            		//1~255（s）,0:disable
    u8 	GLL;            		//1~255（s）,0:disable
    u8 	RMC;            		//1~255（s）,0:disable
    u8 	VTG;            		//1~255（s）,0:disable
    u8 	ZDA;            		//1~255（s）,0:disable
    u8 	Attributes;     		//配置数据保存位置 ,0保存到SRAM，1保存到SRAM&FLASH，2临时保存
    u8 	CS;             		//校验值
    u16 end;            		//结束符:0X0D0A
} SkyTra_outmsg;

//SkyTra S1216F8 配置位置更新率结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0003；
    u8 	id;             		//ID，固定为0X0E
    u8 	rate;           		//取值范围:1, 2, 4, 5, 8, 10, 20, 25, 40, 50
    u8 	Attributes;     		//配置数据保存位置 ,0保存到SRAM，1保存到SRAM&FLASH，2临时保存
    u8 	CS;             		//校验值
    u16 end;            		//结束符:0X0D0A
} SkyTra_PosRate;

//SkyTra S1216F8 配置输出脉冲(PPS)宽度结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0007；
    u8 	id;             		//ID，固定为0X65
    u8 	Sub_ID;         		//0X01
    u32 width;        			//1~100000(us)
    u8 	Attributes;    	 		//配置数据保存位置 ,0保存到SRAM，1保存到SRAM&FLASH，2临时保存
    u8 	CS;             		//校验值
    u16 end;            		//结束符:0X0D0A
} SkyTra_pps_width;

//SkyTra S1216F8 ACK结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0002；
    u8 	id;             		//ID，固定为0X83
    u8 	ACK_ID;         		//ACK ID may further consist of message ID and message sub-ID which will become 3 bytes of ACK message
    u8 	CS;             		//校验值
    u16 end;           		 	//结束符
} SkyTra_ACK;

//SkyTra S1216F8 NACK结构体
typedef __packed struct
{
    u16 sos;            		//启动序列，固定为0XA0A1
    u16 PL;             		//有效数据长度0X0002；
    u8 	id;             		//ID，固定为0X84
    u8 	NACK_ID;         		//ACK ID may further consist of message ID and message sub-ID which will become 3 bytes of ACK message
    u8 	CS;             		//校验值
    u16 end;            		//结束符
} SkyTra_NACK;

//本地调用链接结构体
typedef __packed struct
{
	float 			Longitude;	//经度
	u8 				EWsymbol;	//东西经标识
	float 			Latitude;	//纬度
	u8				NSsymbol;	//南北纬
	Bool_ClassType 	CapSignal;	//捕获信号状态
	float			Altitude;	//高度
	float			Speed;		//速度
	u8				FixMode;	//修正模式
	u8 				PosslNum;	//用于定位卫星数
	u8 				SvNum;		//可视化卫星数
	u8				BeidouSvNum;//北斗可视化卫星数
	nmea_utc_time 	GPS_UTC;	//UTC日期及时间
} Local_GPSTotalData;

extern nmea_msg ggps;
extern Local_GPSTotalData lgps;

//返回u32的pow函数
#ifndef uint32_pow
#define uint32_pow				(uint32_t)pow
#endif

static u8 NMEA_Comma_Pos (u8 *buf, u8 cx);
static int NMEA_Str2num (u8 *buf, u8*dx);
//局部协议分析
static void NMEA_GPGSV_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_BDGSV_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_GNGGA_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_GNGSA_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_GNGSA_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_GNRMC_Analysis (nmea_msg *gpsx, u8 *buf);
static void NMEA_GNVTG_Analysis (nmea_msg *gpsx, u8 *buf);
static u8 SkyTra_Cfg_Cfg_Save (void);
static u8 SkyTra_Cfg_Msg (u8 msgid, u8 uart1set);
static u8 SkyTra_Cfg_Prt (u32 baud_id);
static u8 SkyTra_Cfg_Tp (u32 width);
static u8 SkyTra_Cfg_Rate (u8 Frep);
static void SkyTra_Send_Date (u8* dbuf, u16 len);
static void GPS_SkytraProtocolAnalysis (nmea_msg *gpsx, u8 *buf);			//整体调用分析
//GPS应用
void GPS_TotalConfigInit (void);											//GPS初始化
static void GPS_TotalData_Storage (nmea_msg *gpsx, Local_GPSTotalData *l);	//存储GPS数据
static void GPS_TotalData_Display (Local_GPSTotalData *l);					//GPS数据显示
void GPS_DataGatherTaskHandler (void);	


//====================================================================================================
//code by </MATRIX>@Neod Anderjon
