#include <linux/kernel.h>


#define OFF (0)
#define ON  (1)

#define TRIP_TEMP   (60) // 70°C
#define NORMAL_TEMP   (45)
#define CRITIC_TEMP (100)

#define MOS_OFF  (0xff)
#define MOS_ON   (0x80)
#define MOS_KEEP (0)
#define TEMP_SAMPLE_TIME  (3)
#define TEMP_SAMPLE_DELAY (5000)//us
#define POLL_PEROID  (2000) //ms
#define TRIP_NUM  	 (1)

extern unsigned char log_enable;

#if 0
#define INFOR(fmt,args...)
#else
#define INFOR(fmt,args...)  if(log_enable) printk(KERN_EMERG "USB-TZ: %s " fmt,__func__,##args);
#endif
