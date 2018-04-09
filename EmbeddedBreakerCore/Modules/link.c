#include "stdafx.h"
//code by </MATRIX>@Neod Anderjon
//author: Neod Anderjon
//====================================================================================================
/*
	模块对框架EmbeddBreakerCore的链接
	该文件写入对框架的函数调用支持
*/

//选项设置，链接到Universal_Resource_Config函数的模块库
void Modules_UniResConfig (void)
{
	//该函数设置内容可以更新Universal_Resource_Config函数原设置
}

//模块选项映射表，链接到urcMapTable_Print函数
void Modules_URCMap (void)
{

}

//选项处理，链接到pclURC_DebugHandler函数
void Modules_urcDebugHandler (u8 ed_status, Modules_SwitchNbr sw_type)
{
	//使用前请先更新Modules_SwitchNbr内容
}

//协议调用指令响应，链接到OrderResponse_Handler函数
void Modules_ProtocolTask (void)
{
	
}

//OLED常量显示屏，链接到OLED_DisplayInitConst和UIScreen_DisplayHandler函数
void OLED_ScreenModules_Const (void)
{
	snprintf((char*)oled_dtbuf, OneRowMaxWord, ("Global Position"));
	OLED_ShowString(strPos(0u), ROW1, (StringCache*)oled_dtbuf, Font_Size);
	snprintf((char*)oled_dtbuf, OneRowMaxWord, ("SatelliteSystem"));
	OLED_ShowString(strPos(0u), ROW2, (StringCache*)oled_dtbuf, Font_Size);
	OLED_Refresh_Gram();
}

//OLED模块调用数据显示，链接到UIScreen_DisplayHandler函数
void OLED_DisplayModules (u8 page)
{
	switch (page)
	{
	case 5:
		OLED_DisplayGPS_LonLat(&lgps);			//经纬度
		break;
	case 6:
		OLED_DisplayGPS_AltSpd(&lgps);			//高度速度
		break;
	}
}

//硬件底层初始化任务，链接到bspPeriSysCalls函数
void Modules_HardwareInit (void)
{
	GPS_TotalConfigInit();
}

//硬件底层外部中断初始化，链接到EXTI_Config_Init函数
void Modules_ExternInterruptInit (void)
{
	
}

//外部中断任务，无需声明，使用时修改函数名
void EXTIx_IRQHandler (void)
{
#if SYSTEM_SUPPORT_OS 												
	OSIntEnter();    
#endif
	
	
	
#if SYSTEM_SUPPORT_OS 												
	OSIntExit();  											 
#endif
}

//模块非中断任务，链接到local_taskmgr.c，默认添加到第二任务
void Modules_NonInterruptTask (void)
{
	GPS_DataGatherTaskHandler();
}

//模块中断任务，链接到time_base.c TIM2_IRQHandler函数中
void Modules_InterruptTask (void)
{
	
}

//基于RTC时间的任务计划，链接到local_taskmgr.c，默认添加到第四任务
void Modules_RTC_TaskScheduler (void)
{
	/*
		RTC API:
			*(rtcTotalData + 0): 年份
			*(rtcTotalData + 1): 月份
			*(rtcTotalData + 2): 日
			*(rtcTotalData + 3): 星期
			*(rtcTotalData + 4): 时
			*(rtcTotalData + 5): 分
			*(rtcTotalData + 6): 秒
	*/
	//example: 设置含有灯光效果的外设休眠
	if ((*(rtcTotalData + 4) >= 1 
		&& *(rtcTotalData + 4) <= 6) || *(rtcTotalData + 4) == 0)
	{
		OLED_Clear();
		OLED_Switch = OLED_Disable;
		Light_Switch = Light_Disable;
	}
	else
	{
		OLED_Switch = OLED_Enable;
		Light_Switch = Light_Enable;
	}
}

//模块状态内容打印请求，链接到sta_req.c displaySystemInfo函数中
void Modules_StatusReqHandler (void)
{
	//此项设计可以减少模块指令的多余添加
	GPS_TotalData_Display(&lgps);
}

//====================================================================================================
//code by </MATRIX>@Neod Anderjon
