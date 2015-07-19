#include "mcu_key.h"
#include "log_time.h"
#include "common.h"
#include "log_power.h"
#include "mcu_systick.h"

#define Read_Key    GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_0)

/*
 * 函数名：key_scan
 * 描述  ：处理按键动作
 * 输入  ：无
 * 输出  ：无
 * 返回  ：0	无效按键动作
 *         1	单击
 *         2	双击
 *         3	三击
 *         4	长按
 * 调用  ：
 */
 /********************************************************************************************************************
*
*
*
*
*********************************************************************************************************************/
u8 key_scan(void)
{	 
	volatile u32 		Key_Time=0;
	
 	Key_Time = systick;

	// 开始扫描时若检测到按键按下，则进入判断流程
	if(Read_Key==SET)
	{
		Key_Time = systick;
		
		//=========================长按=========================//
		// 软件去抖延时30ms，即延时30ms后再检测按键是否仍按下，以过滤按键按下初期的接触抖动
		delay_100ms(1);                      	 	

		// 30ms后若按键仍按下，说明按键确实是被人为按下的。
		if(Read_Key==SET)							// 说明是真正有按下动作
		{	 		
			while(Read_Key==SET)
			{
				if((systick - Key_Time) > 20)		// 按下约2秒
				 {
					// 这里不能添加打印语句，因为按键中断优先级设置比串口1发送中断高，因此会造成死锁
					
					return 4;
				}
			}

			// 如果不到2秒就松开，则会进入下面的判断流程。
			
			//=========================单击=========================//			
			if(systick-Key_Time > 10)				// 按键按下1-2秒，视为无效按键动作
				return 0;							// 返回零，无效按键动作

ER_return_1:
			while(Read_Key==RESET)					// 等待按下
			{
				if(systick-Key_Time > 5)			// 0.5秒内没有再次按下
			 	{
					return 1;						// 单击
				}				
			}

			// 软件去抖动延时30ms
			delay_100ms(1);                      	

			if(Read_Key==RESET) 
				goto ER_return_1;					// 因抖动，要重新判断

			//=========================双击=========================//
			Key_Time=systick;			

			while(Read_Key==SET)					// 0.5秒内又有真正的按下，则会进入这个环节，等待弹起
			{
				if(systick-Key_Time > 5)			// 按下0.5秒没有弹起
			 	{
					return 0;						// Error，超时退出
				}
			}
			
			Key_Time=systick;
		
ER_return_2:
			while(Read_Key==RESET)					// 有弹起，等待按下
			{										// 再等0.5秒，没有按下就认为是双击
				if(systick-Key_Time > 5)			// 0.5秒内没有再次按下
			 	{
					return 2;						// 双击
				}				
			}
			
			delay_100ms(1);                      	// 软件去抖动延时20-40ms

			if(Read_Key==RESET) 
				goto ER_return_2;					// 因抖动，要重新判断;如果还是弹起状态，则重新执行

			//=========================三击=========================//
			Key_Time=systick;

			while(Read_Key==SET)					// 0.5秒内又有真正的按下，则会进入这个环节，等待弹起
			{
				if(systick-Key_Time > 5)			// 按下0.5秒没有弹起
				{	
					return 3;						// 超时退出，认为只有三击有效
				}
			}
		
			return 3;								// 忽略后面的多击，最多只判断三击作
		}
	}

	return 0;
}
