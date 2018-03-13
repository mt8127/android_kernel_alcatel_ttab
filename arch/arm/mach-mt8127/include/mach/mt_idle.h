#ifndef _MT_IDLE_H
#define _MT_IDLE_H

extern void enable_dpidle_by_bit(int id);
extern void disable_dpidle_by_bit(int id);


#ifdef __MT_IDLE_C__

/* idle task driver internal use only */

#if EN_PTP_OD
extern u32 ptp_data[3];
#endif

#ifdef SPM_SODI_ENABLED
extern u32 gSPM_SODI_EN;
extern bool gSpm_IsLcmVideoMode;
#endif

extern unsigned long localtimer_get_counter(void);
extern int localtimer_set_next_event(unsigned long evt);

#endif /* __MT_IDLE_C__ */


#endif
