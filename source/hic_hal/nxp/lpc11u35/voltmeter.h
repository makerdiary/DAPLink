
#ifndef __VOLTMETER_H__
#define __VOLTMETER_H__

#include <stdint.h>

extern void voltmeter_start(uint32_t options);
extern void voltmeter_stop(void);
extern void voltmeter_set_amp(uint8_t amp);
// extern void voltmeter_set_period(uint32_t us);

#endif // __VOLTMETER_H__
