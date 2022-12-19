#ifndef __SOLAR_MPPT_H__
#define __SOLAR_MPPT_H__

#include "stdbool.h"

typedef enum
{
	MPPT_STATE_SCAN = 0,
	MPPT_STATE_HOLD,
} mppt_state_t;

double mppt_output_p_get(void);
void mppt_process_po(void);
void mppt_process_nml(void);
void mppt_test(void);
bool mppt_is_running(void);
void mppt_power_check_for_low_power_en(void);
void mppt_process_nml_deinit(void);
#endif	// #ifndef __SOLAR_MPPT_H__
