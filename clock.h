/*
 * This include file describes the functions exported by clock.c
 */
#ifndef __CLOCK_H
#define __CLOCK_H

/*
 * Definitions for functions being abstracted out
 */
void micro_second_sleep(uint32_t);

uint32_t micro_second(void);

void clock_setup(void);

#endif /* generic header protector */
