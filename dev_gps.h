#ifndef __DEV_GPS
#define __DEV_GPS

#include "common.h"
#include "stm32f10x.h"

extern STATUS	sts_gps_power;

extern BOOL		is_gps_fixed;

extern BOOL		is_rmc_received;
extern BOOL		is_gsa_received;

#define 	MAX_LEN_GPS_RMC		76
#define 	MAX_LEN_GPS_GSA			70

extern char		gps_msg_rmc[MAX_LEN_GPS_RMC+1];	
extern char		gps_msg_gsa[MAX_LEN_GPS_GSA+1];	

void 	gps_init(void);
void 	gps_power_up(void);
void 	gps_power_down(void);

void 	gps_coldstart(void);

int 		gps_ana_rmc(void);
int 		gps_ana_gsa(void);

#endif
