#ifndef __LOG_POWER__
#define __LOG_POWER__

#include "stm32f10x_gpio.h"
#include "stm32f10x.h"
#include "common.h"

#define MIN_BATVOL_FOR_WORKING			370		// ����﮵�صķŵ���������
#define NML_BATVOL_FOR_WORKING			390		// �豸���������������͵�ѹ

#define MAX_TIMES_DETECT_LOWBATVOL		3
#define MAX_TIMES_DETECT_NMLBATVOL		3

#define MAX_TIEMS_ASSERT_LOWBATVOL		3		// �͵����¼������Ĵ�������

#define DEF_BATVOL_TO_CHARGE				410		// ﮵�س�������ٽ��ѹ(4.10V)

#define MAX_ADCVOL_FOR_BAT				400		// ��ص�ѹ����ʱ���ֵ���Ƿ���ڵ�������ֵ��(������ֵС��400���رش���)

#define THD_BATVOL_FULL_CHARGED			400		// ��س�����ʱ������ٽ��ѹ

extern BOOL			is_mcu_in_stopmode;		

extern BOOL			is_sys_in_darkmode;		

// battery related
#ifdef USING_PWR_BAT
extern BOOL			is_bat_in_charging;		

extern unsigned int	de_batvol;				
#endif

#define BIT_STS_POWER_BAT		7			// ���ͨ��״̬λ���
#define BIT_STS_POWER_EXT		6			// ����Դͨ��״̬λ���
#define BIT_STS_POWER_ACC		5			// ACC��Դͨ��״̬λ���
#define BIT_STS_POWER_RLP		4			// (�̵������Ƶ�)��·ͨ��״̬λ���
#define BIT_STS_POWER_RLO		3			// (�̵������Ƶ�)��·ͨ��״̬λ���

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
