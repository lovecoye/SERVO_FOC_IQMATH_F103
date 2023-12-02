#ifndef __GETMICROS_H__
#define __GETMICROS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

uint32_t micros(void);
void delay(uint32_t _ms);

#ifdef __cplusplus
}
#endif
#endif /*__GETMICROS_H__ */
