#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
//GPS库

u8 u2TempBuffer[USART2_MAX_RECV_LEN]; 		//USART1依据USART2应用内建的发送缓存区
nmea_msg gpsx; 								//GPS信息
__align(4) u8 dtbuf[50];   					//打印缓存器，unsigned char 4字对齐
const u8 *fixmode_tbl[4] = {(u8 *)"Fail", (u8 *)"Fail", (u8 *)"2D", (u8 *)"3D"};//fix mode，定位模式字符串，也可以用enum型数字代替
const u32 BAUD_id[9] = {4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};//模块支持波特率数组
float gpsWholeData[gpsDataSize] = {0};		//内建数据存储数组

/*
	从buf里面得到第cx个逗号所在的位置
	返回值:0~0XFE,代表逗号所在位置的偏移
			0XFF,代表不存在第cx个逗号
*/
u8 NMEA_Comma_Pos (u8 *buf, u8 cx)
{
    u8 *p = buf;
	
    while (cx)
    {
		//遇到'*'或者非法字符，则不存在第cx个逗号
        if (*buf == '*' || *buf < ' ' || *buf > 'z') 
			return 0xff;					
        if (*buf == ',') 
			cx--;
        buf++;
    }
	
    return (buf - p);
}

/*
	str转换为数字,以','或者'*'结束
	buf:数字存储区
	dx:小数点位数,返回给调用函数
	返回值:转换后的数值
*/
int NMEA_Str2num (u8 *buf, u8 *dx)
{
    u32 ires = 0, fres = 0;
    u8 i, ilen = 0, flen = 0, mask = 0, *p = buf;
    int res;
	
    while (True) 								
    {
        if (*p == '-')
        {
            mask |= 0X02;    				//是负数
            p++;
        }
        if (*p == ',' || *p == '*')
			break;							//遇到结束
        if (*p == '.')
        {
            mask |= 0X01;    				//遇到小数点
            p++;
        }
        else if (*p > '9' || *p < '0')		//有非法字符
        {
            ilen = 0;
            flen = 0;
            break;
        }
        if (mask & 0X01) 
			flen++;
        else 
			ilen++;
        p++;
    }
    if (mask & 0X02) 
		buf++;								//去掉负号
	//得到整数部分数据
    for (i = 0; i < ilen; i++)				
        ires += uint32_pow(10, ilen - 1 -  i) * (buf[i] - '0');
	//最多取5位小数
    if (flen > 5) 
		flen = 5;					
    *dx = flen;	 							//小数点位数
	//得到小数部分数据
    for (i = 0; i < flen; i++)				
        fres += uint32_pow(10, flen - 1 - i) * (buf[ilen + 1 + i] - '0');
    res = ires * uint32_pow(10, flen) + fres;
    if (mask & 0X02) 
		res = -res;
	
    return res;
}

/*
	分析GPGSV信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_GPGSV_Analysis (nmea_msg *gpsx, u8 *buf)
{
    u8 *p, *p1, dx, posx, i, j, slx = 0;
	
    p = buf;
    p1 = (u8 *)strstr((const char *)p, "$GPGSV");
    posx = NMEA_Comma_Pos(p1, 3); 			//得到可见卫星总数
	
    if (posx != 0XFF) 
		gpsx -> svnum = NMEA_Str2num(p1 + posx, &dx);
	//得到GPGSV的条数
    for (i = 0; i < (p1[7] - '0'); i++)		
    {
        p1 = (u8 *)strstr((const char *)p, "$GPGSV");
        for (j = 0; j < 4; j++, slx++)
        {
			//得到卫星编号
            posx = NMEA_Comma_Pos(p1, 4 + j * 4);
            if (posx != 0XFF) gpsx -> slmsg[slx].num = NMEA_Str2num(p1 + posx, &dx);
            else break;
			//得到卫星仰角
            posx = NMEA_Comma_Pos(p1, 5 + j * 4);
            if (posx != 0XFF) gpsx -> slmsg[slx].eledeg = NMEA_Str2num(p1 + posx, &dx);
            else break;
			//得到卫星方位角
            posx = NMEA_Comma_Pos(p1, 6 + j * 4);
            if (posx != 0XFF) gpsx -> slmsg[slx].azideg = NMEA_Str2num(p1 + posx, &dx);
            else break;
			//得到卫星信噪比
            posx = NMEA_Comma_Pos(p1, 7 + j * 4);
            if (posx != 0XFF) gpsx -> slmsg[slx].sn = NMEA_Str2num(p1 + posx, &dx);
            else break;
			//slx++;
        }
        p = p1 + 1;							//切换到下一个GPGSV信息
    }
}

/*
	分析BDGSV信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_BDGSV_Analysis (nmea_msg *gpsx, u8 *buf)
{
    u8 *p, *p1, dx, i, j, slx = 0, posx;
	
    p = buf;
    p1 = (u8 *)strstr((const char *)p, "$BDGSV");
    posx = NMEA_Comma_Pos(p1, 3); 			//得到可见北斗卫星总数
	
    if (posx != 0XFF) 
		gpsx -> beidou_svnum = NMEA_Str2num(p1 + posx, &dx);
	//得到BDGSV的条数
    for (i = 0; i < (p1[7] - '0'); i++, slx++)		
    {
        p1 = (u8 *)strstr((const char *)p, "$BDGSV");
        for (j = 0; j < 4; j++)
        {
            posx = NMEA_Comma_Pos(p1, 4 + j * 4);
            if (posx != 0XFF) gpsx -> beidou_slmsg[slx].beidou_num = NMEA_Str2num(p1 + posx, &dx);//得到卫星编号
            else break;
            posx = NMEA_Comma_Pos(p1, 5 + j * 4);
            if (posx != 0XFF) gpsx -> beidou_slmsg[slx].beidou_eledeg = NMEA_Str2num(p1 + posx, &dx);//得到卫星仰角
            else break;
            posx = NMEA_Comma_Pos(p1, 6 + j * 4);
            if (posx != 0XFF) gpsx -> beidou_slmsg[slx].beidou_azideg = NMEA_Str2num(p1 + posx, &dx);//得到卫星方位角
            else break;
            posx = NMEA_Comma_Pos(p1, 7 + j * 4);
            if (posx != 0XFF) gpsx -> beidou_slmsg[slx].beidou_sn = NMEA_Str2num(p1 + posx, &dx);//得到卫星信噪比
            else break;
            //slx++;
        }
        p = p1 + 1;							//切换到下一个BDGSV信息
    }
}

/*
	分析GNGGA信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_GNGGA_Analysis (nmea_msg *gpsx, u8 *buf)
{
    u8 *p1, dx, posx;
	
    p1 = (u8 *)strstr((const char *)buf, "$GNGGA");
	//得到GPS状态
    posx = NMEA_Comma_Pos(p1, 6);			
    if (posx != 0XFF) 
		gpsx -> gpssta = NMEA_Str2num(p1 + posx, &dx);
	//得到用于定位的卫星数
    posx = NMEA_Comma_Pos(p1, 7);			
    if (posx != 0XFF) 
		gpsx -> posslnum = NMEA_Str2num(p1 + posx, &dx);
	//得到海拔高度
    posx = NMEA_Comma_Pos(p1, 9);			
    if (posx != 0XFF) 
		gpsx -> altitude = NMEA_Str2num(p1 + posx, &dx);
}

/*
	分析GNGSA信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_GNGSA_Analysis (nmea_msg *gpsx, u8 *buf)
{
    u8 *p1, dx, posx, i;
	
    p1 = (u8 *)strstr((const char *)buf, "$GNGSA");
    posx = NMEA_Comma_Pos(p1, 2);			//得到定位类型
	
    if (posx != 0XFF) 
		gpsx -> fixmode = NMEA_Str2num(p1 + posx, &dx);
	//得到定位卫星编号
    for (i = 0; i < Support_Satellite_Num; i++)
    {
        posx = NMEA_Comma_Pos(p1, 3 + i);
        if (posx != 0XFF) gpsx -> possl[i] = NMEA_Str2num(p1 + posx, &dx);
        else break;
    }
	
	//得到PDOP位置精度因子
    posx = NMEA_Comma_Pos(p1, 15);			
    if (posx != 0XFF) gpsx -> pdop = NMEA_Str2num(p1 + posx, &dx);
	//得到HDOP位置精度因子
    posx = NMEA_Comma_Pos(p1, 16);			
    if (posx != 0XFF) gpsx -> hdop = NMEA_Str2num(p1 + posx, &dx);
	//得到VDOP位置精度因子
    posx = NMEA_Comma_Pos(p1, 17);			
    if (posx != 0XFF) gpsx -> vdop = NMEA_Str2num(p1 + posx, &dx);
}

/*
	分析GNRMC信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_GNRMC_Analysis (nmea_msg *gpsx, u8 *buf)
{
    u8 *p1, dx, posx;
    u32 temp;
    float rs;
	
    p1 = (u8 *)strstr((const char *)buf, "$GNRMC");	//"$GNRMC",经常有&和GNRMC分开的情况,故只判断GPRMC.
    posx = NMEA_Comma_Pos(p1, 1);					//得到UTC时间
    if (posx != 0XFF)
    {
        temp = NMEA_Str2num(p1 + posx, &dx) / uint32_pow(10, dx);//得到UTC时间,去掉ms
        gpsx ->  utc.hour = temp / 10000;
        gpsx -> utc.min = (temp / 100) % 100;
        gpsx -> utc.sec = temp %  100;
    }
    posx = NMEA_Comma_Pos(p1, 3);					//得到纬度
    if (posx != 0XFF)
    {
        temp = NMEA_Str2num(p1 + posx, &dx);
        gpsx -> latitude = temp / uint32_pow(10, dx + 2);//得到°
        rs = temp % uint32_pow(10, dx + 2);			//得到'
        gpsx -> latitude = gpsx -> latitude * uint32_pow(10, 5) + (rs * uint32_pow(10, 5 - dx)) / 60;//转换为°
    }
    posx = NMEA_Comma_Pos(p1, 4);					//南纬还是北纬
    if (posx != 0XFF) gpsx -> nshemi = *(p1 + posx);
    posx = NMEA_Comma_Pos(p1, 5);					//得到经度
    if (posx != 0XFF)
    {
        temp = NMEA_Str2num(p1 + posx, &dx);
        gpsx -> longitude = temp / uint32_pow(10, dx + 2);//得到°
        rs = temp % uint32_pow(10, dx + 2);			//得到'
        gpsx -> longitude = gpsx -> longitude * uint32_pow(10, 5) + (rs * uint32_pow(10, 5 - dx)) / 60;//转换为°
    }
    posx = NMEA_Comma_Pos(p1, 6);					//东经还是西经
    if (posx != 0XFF) gpsx -> ewhemi = *(p1 + posx);
    posx = NMEA_Comma_Pos(p1, 9);					//得到UTC日期
    if (posx != 0XFF)
    {
        temp = NMEA_Str2num(p1 + posx, &dx);		//得到UTC日期
        gpsx -> utc.date = temp / 10000;
        gpsx -> utc.month = (temp / 100) % 100;
        gpsx -> utc.year = 2000 + temp % 100;
    }
}

/*
	分析GNVTG信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void NMEA_GNVTG_Analysis (nmea_msg *gpsx,  u8 *buf)
{
    u8 *p1, dx, posx;
	
    p1 = (u8 *)strstr((const char *)buf, "$GNVTG");
    posx = NMEA_Comma_Pos(p1, 7);					//得到地面速率
    if (posx != 0XFF)
    {
        gpsx -> speed = NMEA_Str2num(p1 + posx, &dx);
        if (dx < 3) gpsx -> speed *= uint32_pow(10, 3 - dx);//确保扩大1000倍
    }
}

/*
	提取NMEA-0183信息
	gpsx:nmea信息结构体
	buf:接收到的GPS数据缓冲区首地址
*/
void GPS_Analysis (nmea_msg *gpsx, u8 *buf)
{
    NMEA_GPGSV_Analysis(gpsx, buf);			//GPGSV解析
    NMEA_BDGSV_Analysis(gpsx, buf);			//BDGSV解析
    NMEA_GNGGA_Analysis(gpsx, buf);			//GNGGA解析
    NMEA_GNGSA_Analysis(gpsx, buf);			//GPNSA解析
    NMEA_GNRMC_Analysis(gpsx, buf);			//GPNMC解析
    NMEA_GNVTG_Analysis(gpsx, buf);			//GPNTG解析
}

/*
	UBLOX 配置代码
	检查CFG配置执行情况
	返回值:	0,ACK成功
			1,接收超时错误
			2,没有找到同步字符
			3,接收到NACK应答
*/
u8 SkyTra_Cfg_Ack_Check (void)
{
    u16 len = 0, i;
    u8 rval = 0;
	
	//等待接收到应答
    while ((USART2_RX_STA & 0X8000) == 0 && len++ < 100)
        delay_ms(5);
    
    if (len < 100)   						//超时错误
    {
        len = USART2_RX_STA & 0X7FFF;		//此次接收到的数据长度
        for (i = 0; i < len; i++)
        {
            if (USART2_RX_BUF[i] == 0X83) break;
            else if (USART2_RX_BUF[i] == 0X84)
            {
                rval = 3;
                break;
            }
        }
		//没有找到同步字符
        if (i == len) 
			rval = 2;				
    }
	//接收超时错误
    else 
		rval = 1;							
    USART2_RX_STA = 0;						
	
    return rval;
}

/*
	配置SkyTra_GPS/北斗模块波特率
	baud_id:0~8，对应波特率,4800/9600/19200/38400/57600/115200/230400/460800/921600
	返回值:0,执行成功;其他,执行失败(这里不会返回0了)
*/
u8 SkyTra_Cfg_Prt (u32 baud_id)
{
    SkyTra_baudrate *cfg_prt = (SkyTra_baudrate *)USART2_TX_BUF;
    cfg_prt -> sos = 0XA1A0;				//引导序列(小端模式)
    cfg_prt -> PL = 0X0400;					//有效数据长度(小端模式)
    cfg_prt -> id = 0X05;		   	 		//配置波特率的ID
    cfg_prt -> com_port = 0X00;				//操作串口1
    cfg_prt -> Baud_id = baud_id;	 		//波特率对应编号
    cfg_prt -> Attributes = 1; 				//保存到SRAM&FLASH
    cfg_prt -> CS = cfg_prt -> id ^ cfg_prt -> com_port ^ cfg_prt -> Baud_id ^ cfg_prt -> Attributes;//异或校验
    cfg_prt -> end = 0X0A0D;        		//发送结束符(小端模式)
    SkyTra_Send_Date((u8 *)cfg_prt, sizeof(SkyTra_baudrate));//发送数据给SkyTra
    delay_ms(200);							//等待发送完成
    USART2_Init(BAUD_id[baud_id]);			//重新初始化串口2
	
    return SkyTra_Cfg_Ack_Check();			//这里不会反回0,因为UBLOX发回来的应答在串口重新初始化的时候已经被丢弃了
}

/*
	配置SkyTra_GPS模块的时钟脉冲宽度
	width:脉冲宽度1~100000(us)
	返回值:0,发送成功;其他,发送失败
*/
u8 SkyTra_Cfg_Tp (u32 width)
{
    u32 temp = width;
	
    SkyTra_pps_width *cfg_tp = (SkyTra_pps_width *)USART2_TX_BUF;
    temp = (width >> 24) | ((width >> 8) & 0X0000FF00) | ((width << 8) & 0X00FF0000) | ((width << 24) & 0XFF000000);//小端模式
    cfg_tp -> sos = 0XA1A0;		    		//cfg header(小端模式)
    cfg_tp -> PL = 0X0700;       		 	//有效数据长度(小端模式)
    cfg_tp -> id = 0X65;			   	 	//cfg tp id
    cfg_tp -> Sub_ID = 0X01;				//数据区长度为20个字节
    cfg_tp -> width = temp;		  			//脉冲宽度,us
    cfg_tp -> Attributes = 0X01;  			//保存到SRAM&FLASH
	
    //异或计算校验 &优先级高于^
    cfg_tp -> CS = cfg_tp -> id ^ cfg_tp -> Sub_ID ^ (cfg_tp -> width >> 24)
                   ^ ((cfg_tp -> width >> 16) & 0XFF) ^ ((cfg_tp -> width >> 8) & 0XFF) ^ (cfg_tp -> width & 0XFF) ^ cfg_tp -> Attributes;
    cfg_tp -> end = 0X0A0D;       			//发送结束符(小端模式)
    SkyTra_Send_Date((u8 *)cfg_tp, sizeof(SkyTra_pps_width));//发送数据给NEO-6M
	
    return SkyTra_Cfg_Ack_Check();
}

/*
	配置SkyTraF8-BD的更新速率
	Frep:（取值范围:1,2,4,5,8,10,20,25,40,50）测量时间间隔，单位为Hz，最大不能大于50Hz
	返回值:0,发送成功;其他,发送失败
*/
u8 SkyTra_Cfg_Rate (u8 Frep)
{
    SkyTra_PosRate *cfg_rate = (SkyTra_PosRate *)USART2_TX_BUF;
    cfg_rate -> sos = 0XA1A0;	    		//cfg header(小端模式)
    cfg_rate -> PL = 0X0300;				//有效数据长度(小端模式)
    cfg_rate -> id = 0X0E;	      			//cfg rate id
    cfg_rate -> rate = Frep;	 	  		//更新速率
    cfg_rate -> Attributes = 0X01;	   		//保存到SRAM&FLASH
    cfg_rate -> CS = cfg_rate -> id ^ cfg_rate -> rate ^ cfg_rate -> Attributes;//脉冲间隔，us
    cfg_rate -> end = 0X0A0D;       		//发送结束符(小端模式)
    SkyTra_Send_Date((u8 *)cfg_rate, sizeof(SkyTra_PosRate));//发送数据给NEO-6M
	
    return SkyTra_Cfg_Ack_Check();
}

/*
	发送一批数据给SkyTraF8-BD，这里通过串口2发送
	实际就是直接USART2调用发送
	dbuf：数据缓存首地址
	len：要发送的字节数
*/
void SkyTra_Send_Date (u8* dbuf, u16 len)	//USART2字符发送
{
    u16 j;
	
    for (j = 0; j < len; j++) 				//循环发送数据
    {
        USART2 -> DR = dbuf[j];				
		usart2WaitForDataTransfer();		
    }
}

//GPS初始化
void GPS_TotalConfigInit (void)
{
    u8 key = 0xFF;
	
    if (SkyTra_Cfg_Rate(5) != 0)			//设置定位信息更新速度
    {
		__ShellHeadSymbol__; U1SD("GPS Module S1216F8-BD Setting\r\n");
        do
        {
            USART2_Init(9600);				//初始化串口2波特率为9600
            SkyTra_Cfg_Prt(3);				
            USART2_Init(38400);				//初始化串口2波特率为38400
            key = SkyTra_Cfg_Tp(100000);	//脉冲宽度为100ms
        }
		//配置SkyTraqF8-BD的更新速率
        while (SkyTra_Cfg_Rate(5) != 0 && key != 0);
		__ShellHeadSymbol__; U1SD("GPS Module S1216F8-BD Set Done\r\n");
        delay_ms(400);						//等待芯片初始化完成
    }
}

//GPS数据处理存储
void GPS_TotalData_Storage (void)
{
	float tp = 0.f;
	static Bool_ClassType memCreatedflag = False;//内存创建标志
	static Bool_ClassType capSignal = False;//捕获到信号标志		
	
	tp = gpsx.longitude;	
	gpsWholeData[0] = (tp /= 100000);		//经度
	gpsWholeData[1] = gpsx.ewhemi;			//经度标识
	
	tp = gpsx.latitude;
	gpsWholeData[2] = (tp /= 100000);		//纬度
	gpsWholeData[3] = gpsx.nshemi;			//纬度标识

	//捕获到信号标志(仅判断经纬度状态)
	if (gpsWholeData[0] != 0 && gpsWholeData[2] != 0 && capSignal == False)
	{
		Beep_Once; 
		capSignal = True;					//置位
		Beep_Once;
	}
	gpsWholeData[16] = capSignal;			//信号标识
	
	tp = gpsx.altitude;						//高度
	gpsWholeData[4] = (tp /= 10);
	
	tp = gpsx.speed;						//速度
	gpsWholeData[5] = (tp /= 1000);		
	
	if (gpsx.fixmode <= 3)					//定位状态
		gpsWholeData[6] = gpsx.fixmode;			
	
	gpsWholeData[7] = gpsx.posslnum;		//用于定位的GPS卫星数
	
	gpsWholeData[8] = (gpsx.svnum % 100);	//可见GPS卫星数
	
	gpsWholeData[9] = (gpsx.beidou_svnum % 100);//可见北斗卫星数
	
	//显示UTC日期
	gpsWholeData[10] = gpsx.utc.year;
	gpsWholeData[11] = gpsx.utc.month;
	gpsWholeData[12] = gpsx.utc.date;
	
	//显示UTC时间
	gpsWholeData[13] = gpsx.utc.hour;
	gpsWholeData[14] = gpsx.utc.min;
	gpsWholeData[15] = gpsx.utc.sec;
	
	//GPS全局数据存储封装完毕
	if (memCreatedflag == False)
	{
		memCreatedflag = True;
		stackOverFlow(gpsWholeData);			
	}
}

//显示GPS定位信息
void GPS_TotalData_Display (void)
{	
	if (PD_Switch == PD_Enable && No_Data_Receive)
	{	
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nLongitude: 			 %4.5f %1c\r\n", gpsWholeData[0], (u8)gpsWholeData[1]);//得到经度字符串
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nLatitude: 			 %4.5f %1c\r\n",gpsWholeData[2], (u8)gpsWholeData[3]);//得到纬度字符串
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();	
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nAltitude: 			 %4.2fm\r\n", gpsWholeData[4]);//得到高度字符串
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nSpeed: 			 	 %4.3fkm/h\r\n", gpsWholeData[5]);//得到速度字符串
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nFix Mode: 			 %s\r\n", fixmode_tbl[(int)gpsWholeData[6]]);
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		

		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nGPS+BD Valid satellite: 	 %02.f\r\n", gpsWholeData[7]);//用于定位的GPS卫星数
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nGPS Visible satellite:  	 %02.f\r\n", gpsWholeData[8]);//可见GPS卫星数
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nBD Visible satellite: 		 %02.f\r\n", gpsWholeData[9]);//可见北斗卫星数
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nUTC Date: 			 %04d/%02d/%02d\r\n", (int)gpsWholeData[10], (int)gpsWholeData[11], (int)gpsWholeData[12]);//显示UTC日期
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();		
		
		snprintf((char *)dtbuf, snprintfStackSpace, "\r\nUTC Time: 			 %02d:%02d:%02d\r\n", (int)gpsWholeData[13], (int)gpsWholeData[14], (int)gpsWholeData[15]);//显示UTC时间
		printf("%s", dtbuf);
		usart1WaitForDataTransfer();
		
		//打印信号状态
		if (gpsWholeData[16] == True)
			printf("\r\nGPS Real-Time Signal Status: 	 Capture OK\r\n");
		else
			printf("\r\nGPS Real-Time Signal Status: 	 Capture Fail\r\n");
		usart1WaitForDataTransfer();
		
		RTC_ReqOrderHandler();					//可选，打印时间
	}
}

//GPS应用
void GPS_DataGatherTaskHandler (void)
{
    u16 i;
    u8 upload = 0u;
	
    delay_ms(1);
	//USART2接收GPS传回数据，用USART1打印出来
	//也可以放到暂存数组
	if (USART2RecDataOver)						//接收到一次数据
	{
		//数据缓存区转移
		for (i = 0; i < (USART2DataLength); i++) 
			u2TempBuffer[i] = USART2_RX_BUF[i];
		USART2_RX_STA = 0;		   				//启动下一次接收
		u2TempBuffer[i] = 0;					//自动添加结束符
		
		GPS_Analysis(&gpsx, (u8 *)u2TempBuffer);//分析字符串
		GPS_TotalData_Storage();				//GPS数据存储处理
	}
	
	//KEY0 KEY1双键长按转换状态
	if (KEY0_LTrigger && KEY1_LTrigger)					
	{
		upload = !upload;
		__ShellHeadSymbol__; U1SD((upload)? "NMEA Data Upload: ON\r\n" : "NMEA Data Upload: OFF\r\n");
	}
	GPS_TotalData_Display();					//打印调试	
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
