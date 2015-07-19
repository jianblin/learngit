#ifndef __LOG_POWER__
#define __LOG_POWER__

#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "common.h"

#define MIN_BATVOL_FOR_WORKING			370		// 根据锂电池的放电曲线设置
#define NML_BATVOL_FOR_WORKING			390		// 设备正常工作所需的最低电压

#define MAX_TIMES_DETECT_LOWBATVOL		3
#define MAX_TIMES_DETECT_NMLBATVOL		3

#define MAX_TIEMS_ASSERT_LOWBATVOL		3		// 低电检测事件触发的次数上限

#define DEF_BATVOL_TO_CHARGE				410		// 锂电池充电与否的临界电压(4.10V)

#define MAX_ADCVOL_FOR_BAT				400		// 电池电压测量时区分电池是否存在的最大测量值的(若测量值小于400则电池必存在)

#define THD_BATVOL_FULL_CHARGED			400		// 电池充满电时的最低临界电压

extern BOOL			is_mcu_in_stopmode;		

extern BOOL			is_sys_in_darkmode;		

// battery related
#ifdef USING_PWR_BAT
extern BOOL			is_bat_in_charging;		

extern unsigned int	de_batvol;				
#endif

#define BIT_STS_POWER_BAT		7			// 电池通断状态位序号
#define BIT_STS_POWER_EXT		6			// 主电源通断状态位序号
#define BIT_STS_POWER_ACC		5			// ACC电源通断状态位序号
#define BIT_STS_POWER_RLP		4			// (继电器控制的)电路通断状态位序号
#define BIT_STS_POWER_RLO		3			// (继电器控制的)油路通断状态位序号

#define GET_STS_POWER(bit)	(sts_power&(1<<bit)?ON:OFF)

#define SET_STS_POWER(bit)	(sts_power |= (1<<bit))
#define CLR_STS_POWER(bit)	(sts_power &= ~(1<<bit))

extern unsigned char	sts_power;		

void power_init(void);
void check_power_status(void);

__asm void mcu_reset(void);
void mcu_shutdown(void);
void mcu_standby(void);
void mcu_stop_enter(void);

#endif
