#include <linux/gfp.h>
#include <linux/types.h>
#include <linux/string.h> // for test cases
#include <asm/cacheflush.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <mach/memory.h>
#include "ddp_wdma.h"
#include <asm/uaccess.h>
#include <mach/mt_irq.h>
#include <mach/mt_clkmgr.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_spm_idle.h>
#include <mach/m4u.h>
#include <linux/met_drv.h>

#include "cmdq_def.h"
#include "ddp_drv.h"
#include "ddp_od.h"
#include "ddp_reg.h"
#include "ddp_debug.h"
#include "ddp_dither.h"
#include "ddp_misc.h"
#include "ddp_log.h"
#include "lcm_drv.h"
#include "primary_display.h"


#define OD_ALLOW_DEFAULT_TABLE
//#define OD_LINEAR_TABLE_IF_NONE


#define OD_USE_CMDQ     1

#define OD_TBL_S_DIM    17
#define OD_TBL_M_DIM    33

/* compression ratio */
#define OD_MANUAL_CPR   58 // 38
#define OD_HSYNC_WIDTH  110

// Additional space after the given memory to avoid unexpected memory corruption
// In the experiments, OD hardware overflew 240 bytes
#define OD_ADDITIONAL_BUFFER 256

#define OD_GUARD_PATTERN 0x16881688
#define OD_GUARD_PATTERN_SIZE 4

#define OD_TBL_S_SIZE   (OD_TBL_S_DIM * OD_TBL_S_DIM)
#define OD_TBL_M_SIZE   (OD_TBL_M_DIM * OD_TBL_M_DIM)
#define FB_TBL_SIZE     (FB_TBL_DIM * FB_TBL_DIM)


#define OD_DBG_ALWAYS   0
#define OD_DBG_VERBOSE  1
#define OD_DBG_DEBUG    2
//#define OD_MET_DEBUG    0 // compile option for debug with mey tool

#define OD_REG_SET_FIELD(cmdq, reg, val, field) DISP_REG_SET_FIELD(cmdq, field, reg, val)


#define ABS(a) ((a > 0) ? a : -a)


/* debug macro */
#define ODDBG(level, fmt, arg...) \
    do { \
        if (od_debug_level >= (level)) \
            printk(KERN_DEBUG "[DISP_OD] " fmt "\n", ##arg); \
    } while (0)

// ioctl
typedef enum{	 
	OD_CTL_READ_REG,
	OD_CTL_WRITE_REG,	 
	OD_CTL_ENABLE_DEMO_MODE,
	OD_CTL_RUN_TEST,	
	OD_CTL_WRITE_TABLE,
	OD_CTL_CMD_NUM,
	OD_CTL_ENABLE
} DISP_OD_CMD_TYPE;
typedef enum{
	OD_CTL_ENABLE_OFF,
	OD_CTL_ENABLE_ON
} DISP_OD_ENABLE_STAGE;

enum
{
    OD_RED,
    OD_GREEN,
    OD_BLUE,
    OD_ALL
};

enum
{
    OD_TABLE_17,
    OD_TABLE_33
};

typedef enum{	 
	OD_STATUS_IDLE,
	OD_STATUS_ONOFF,
	OD_STATUS_ENABLE,
	OD_STATUS_DISABLE,
	OD_STATUS_BYPASS,	
	OD_STATUS_RESET
} OD_STATUS;


static void od_dbg_dump(void);

typedef struct REG_INIT_TABLE_STRUCT
{
   unsigned long reg_addr;
   unsigned int reg_value;
   unsigned int reg_mask;

} REG_INIT_TABLE_STRUCT;


// dummy 17x17
static u8 OD_Table_dummy_17x17[OD_TBL_S_SIZE] =
{
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
  0,  16,  32,  48,  64,  80,  96, 112, 128, 144, 160, 176, 192, 208, 224, 240, 255,
};


#if 0

// 60 hz
static u8 OD_Table_17x17[OD_TBL_S_SIZE] =
{
0,18,36,56,76,95,114,132,148,163,177,192,208,223,236,248,255,
0,16,34,53,73,92,112,129,146,161,176,191,207,220,235,248,255,
0,15,32,50,69,91,109,127,143,160,174,189,205,221,234,247,255,
0,14,29,48,67,87,106,123,140,156,172,187,203,220,234,247,255,
0,13,27,45,64,84,103,120,138,155,170,185,202,219,233,246,255,
0,10,25,43,61,80,99,117,135,152,169,184,200,217,232,246,255,
0,8,24,40,58,77,96,115,133,150,167,183,199,216,231,245,255,
0,6,23,39,56,74,93,112,130,148,165,182,198,214,230,245,255,
0,5,22,38,54,71,89,108,128,146,164,180,197,213,229,244,255,
0,4,20,35,51,68,86,105,125,144,162,179,196,212,228,243,255,
0,3,18,33,48,64,83,102,122,141,160,177,194,211,227,243,255,
0,1,16,31,46,63,80,99,119,138,157,176,193,210,227,242,255,
0,0,13,28,43,59,77,96,116,136,155,173,192,209,226,242,255,
0,0,11,26,41,56,74,93,113,133,152,171,189,208,225,241,255,
0,0,9,24,39,54,71,90,110,130,150,169,187,205,224,240,255,
0,0,7,21,36,50,68,86,106,126,147,166,184,202,222,240,255,
0,0,4,18,32,47,62,82,103,123,143,163,181,199,219,238,255,
};

#else

// 120 hz
static u8 OD_Table_17x17[OD_TBL_S_SIZE] =
{
0,21,44,71,99,122,141,160,175,190,203,216,228,239,251,255,255,
0,16,38,65,90,115,136,153,171,186,200,213,226,237,249,255,255,
0,10,32,56,83,107,130,148,166,182,196,210,223,236,248,255,255,
0,4,25,48,73,98,120,142,159,176,191,206,219,234,246,255,255,
0,0,19,41,64,88,113,135,154,171,188,203,217,231,245,255,255,
0,0,14,35,56,80,104,126,148,166,184,200,214,229,243,255,255,
0,0,9,28,49,71,96,119,141,161,179,196,212,227,241,255,255,
0,0,4,23,42,64,87,112,135,155,175,192,210,226,240,254,255,
0,0,0,18,37,55,78,103,128,150,171,190,208,226,239,253,255,
0,0,0,12,31,49,71,95,119,144,166,186,205,224,238,252,255,
0,0,0,8,24,42,62,86,111,136,160,181,202,222,237,251,255,
0,0,0,3,19,36,54,77,102,127,152,176,197,216,235,250,255,
0,0,0,0,13,28,47,68,94,120,145,169,192,213,232,248,255,
0,0,0,0,8,22,40,59,83,110,136,161,183,208,228,245,255,
0,0,0,0,2,16,30,50,72,99,127,154,175,196,224,242,255,
0,0,0,0,0,8,22,39,60,84,112,142,165,192,218,240,255,
0,0,0,0,0,1,15,28,48,70,99,130,158,188,213,235,255,
};

#endif


#if 0
// dummy 33x33
static u8 OD_Table_33x33[OD_TBL_M_SIZE] =
{
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255,
  0,   8,  16,  24,  32,  40,  48,  56,  64,  72,  80,  88,  96, 104, 112, 120, 128, 136, 144, 152, 160, 168, 176, 184, 192, 200, 208, 216, 224, 232, 240, 248, 255
};

#endif

#if 0
// 60 hz
static u8 OD_Table_33x33[OD_TBL_M_SIZE] =
{
0,9,18,26,36,46,56,66,76,86,95,105,114,123,132,140,148,155,163,170,177,185,192,200,208,216,223,230,236,242,248,254,255,
0,8,17,26,35,45,55,65,75,84,94,104,113,122,131,139,147,155,162,169,177,184,192,199,207,215,222,229,236,242,248,254,255,
0,8,16,25,34,44,53,63,73,83,92,103,112,121,129,137,146,154,161,169,176,183,191,198,207,215,220,229,235,241,248,254,255,
0,7,16,24,33,42,52,61,71,82,91,101,110,119,128,136,144,153,161,168,175,182,190,198,206,214,220,229,235,241,247,254,255,
0,7,15,24,32,41,50,59,69,80,91,100,109,118,127,135,143,152,160,167,174,182,189,197,205,213,221,228,234,241,247,253,255,
0,7,15,23,31,40,49,58,68,78,89,98,107,116,125,134,142,150,158,166,173,181,188,196,204,212,221,228,234,240,247,253,255,
0,6,14,22,29,39,48,57,67,77,87,96,106,115,123,132,140,149,156,165,172,180,187,195,203,212,220,227,234,240,247,253,255,
0,6,13,21,28,37,47,56,65,75,85,95,104,113,122,131,139,147,155,164,171,179,186,194,202,211,219,227,233,240,246,253,255,
0,5,13,20,27,36,45,55,64,74,84,94,103,111,120,129,138,146,155,163,170,178,185,193,202,210,219,226,233,239,246,253,255,
0,4,11,19,26,35,44,53,63,72,82,92,101,110,119,127,136,145,154,162,169,177,185,193,201,209,218,226,232,239,246,252,255,
0,3,10,18,25,33,43,52,61,71,80,90,99,108,117,126,135,144,152,161,169,176,184,192,200,209,217,225,232,239,246,252,255,
0,1,9,17,24,32,42,51,60,69,79,88,98,107,116,125,134,143,151,160,168,176,184,191,200,208,216,224,231,238,245,252,255,
0,0,8,16,24,32,40,49,58,68,77,86,96,105,115,124,133,142,150,159,167,175,183,191,199,207,216,224,231,238,245,252,255,
0,0,7,15,23,32,40,49,57,67,76,85,94,104,113,123,132,140,149,157,166,174,182,190,199,207,215,222,230,238,245,252,255,
0,0,6,15,23,31,39,48,56,66,74,83,93,102,112,121,130,139,148,156,165,174,182,190,198,206,214,222,230,237,245,252,255,
0,0,6,14,22,31,39,47,55,64,73,82,91,100,110,120,129,138,147,155,164,173,181,189,198,206,214,221,229,237,244,252,255,
0,0,5,13,22,30,38,46,54,62,71,80,89,99,108,118,128,137,146,155,164,172,180,189,197,205,213,221,229,236,244,251,255,
0,0,4,13,21,29,37,45,53,61,69,79,88,97,107,117,126,136,145,154,163,171,180,188,196,204,213,221,229,236,244,251,255,
0,0,4,12,20,27,35,43,51,59,68,77,86,96,105,115,125,135,144,153,162,170,179,187,196,204,212,220,228,236,243,251,255,
0,0,3,11,19,26,34,42,50,58,66,75,85,94,104,114,123,133,143,152,161,169,178,186,195,203,211,220,228,235,243,250,255,
0,0,3,10,18,25,33,41,48,56,64,73,83,92,102,112,122,132,141,151,160,169,177,186,194,202,211,219,227,235,243,250,255,
0,0,2,9,17,24,32,39,47,55,64,72,81,91,101,111,120,130,140,149,159,168,177,185,194,202,210,219,227,235,242,250,255,
0,0,1,8,16,23,31,38,46,53,63,70,80,89,99,109,119,129,138,148,157,167,176,185,193,202,210,218,227,234,242,250,255,
0,0,0,7,15,22,30,37,45,52,61,69,78,88,98,108,118,127,137,147,156,165,175,184,193,201,210,218,226,234,242,250,255,
0,0,0,6,13,21,28,36,43,51,59,67,77,86,96,106,116,126,136,145,155,164,173,183,192,201,209,218,226,234,242,250,255,
0,0,0,5,12,20,27,35,42,50,57,67,75,85,95,105,115,125,134,144,154,163,172,181,191,200,209,217,225,233,241,249,255,
0,0,0,4,11,19,26,34,41,49,56,66,74,83,93,103,113,123,133,143,152,162,171,180,189,199,208,217,225,233,241,249,255,
0,0,0,3,10,18,25,33,40,48,55,64,72,82,92,101,111,121,131,141,151,161,170,179,188,197,207,216,224,232,240,248,255,
0,0,0,2,9,17,24,32,39,47,54,61,71,80,90,100,110,120,130,140,150,160,169,178,187,196,205,215,224,232,240,248,255,
0,0,0,0,8,15,23,30,37,45,52,60,70,78,88,98,108,118,128,138,148,158,167,176,185,195,204,213,223,232,240,248,255,
0,0,0,0,7,14,21,29,36,43,50,58,68,76,86,96,106,116,126,137,147,156,166,175,184,193,202,212,222,231,240,248,255,
0,0,0,0,5,12,20,27,34,41,49,56,65,74,84,94,104,115,125,135,145,154,164,173,182,191,201,211,220,230,239,248,255,
0,0,0,0,4,11,18,25,32,40,47,54,62,71,82,92,103,113,123,133,143,153,163,172,181,189,199,209,219,229,238,247,255,
};

#else
static u8 OD_Table_33x33_buf[OD_TBL_M_SIZE] =
{
	0,8,17,33,46,56,69,80,93,102,110,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,17,32,43,54,66,78,90,102,111,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,17,31,40,51,63,74,86,98,110,119,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,14,24,36,48,59,71,83,95,107,117,127,136,144,152,160,168,176,184,192,200,208,216,224,232,240,247,252,254,255,255,255,
	0,7,11,20,32,44,56,67,80,90,102,112,125,135,144,152,160,168,176,184,192,200,208,216,224,232,240,247,252,254,255,255,255,
	0,6,9,17,28,40,52,64,76,87,98,110,120,133,143,151,160,168,176,184,192,200,208,216,224,232,240,246,251,254,255,255,255,
	0,5,9,14,24,36,48,60,72,83,94,106,118,128,140,150,159,168,176,184,192,200,208,216,224,232,239,246,251,254,255,255,255,
	0,4,6,11,22,32,44,56,68,80,90,102,114,126,136,148,158,167,175,184,192,200,208,216,224,232,239,245,251,254,255,255,255,
	0,2,6,10,18,29,40,52,64,76,88,99,111,122,134,144,156,166,174,183,191,200,208,216,223,231,238,245,250,254,255,255,255,
	0,2,4,9,16,25,37,48,60,72,84,96,107,119,130,142,152,164,173,182,190,199,208,215,223,230,237,244,250,253,255,255,255,
	0,0,1,8,14,22,33,45,56,68,80,92,104,115,127,138,150,160,172,181,189,198,206,215,222,230,237,244,250,253,255,255,255,
	0,0,1,7,12,20,30,41,53,64,76,88,100,112,123,135,146,158,168,178,188,197,205,214,221,229,236,243,250,253,255,255,255,
	0,0,1,6,10,18,26,38,49,60,72,84,96,108,120,131,142,154,166,175,186,196,205,213,220,228,235,243,249,253,255,255,255,
	0,0,0,4,10,17,23,34,46,56,68,80,92,104,116,128,139,150,162,172,182,194,203,211,219,227,235,242,247,253,255,255,255,
	0,0,0,3,9,14,21,30,42,52,64,77,88,100,112,124,136,147,158,168,180,191,203,210,218,226,234,241,247,253,255,255,255,
	0,0,0,2,6,11,19,29,37,49,60,72,85,96,108,120,132,143,155,167,177,190,203,210,218,226,234,241,247,253,255,255,255,
	0,0,0,0,4,9,17,27,33,45,56,69,81,93,104,116,128,137,152,165,181,191,201,212,222,230,234,241,247,253,255,255,255,
	0,0,0,0,3,10,16,25,32,41,52,64,77,89,101,112,124,135,148,162,180,190,199,210,219,229,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,40,50,61,72,86,96,109,119,130,144,157,171,186,198,207,217,226,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,43,52,62,72,82,93,104,114,125,138,152,166,179,191,204,215,225,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,66,77,88,101,110,120,132,147,160,173,184,200,211,222,232,241,247,252,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,66,75,85,95,104,116,128,141,155,168,180,192,205,218,227,239,247,252,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,64,72,83,92,100,111,122,136,150,163,175,188,200,213,226,237,246,251,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,64,73,81,89,97,105,116,130,142,153,170,184,196,208,222,236,245,250,255,255,255,
	0,0,0,0,1,8,18,28,36,43,51,59,68,77,85,93,101,110,118,128,139,149,162,179,192,204,216,231,244,249,255,255,255,
	0,0,0,0,1,8,19,29,37,45,53,61,69,77,85,93,101,110,118,128,139,149,162,174,188,200,212,226,241,248,255,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,147,159,170,183,196,208,226,240,247,255,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,166,178,192,204,216,233,246,253,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,166,177,188,200,213,225,242,252,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,165,175,186,197,211,223,233,246,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,156,166,176,186,198,211,221,232,241,252,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,157,169,179,190,201,211,221,231,240,248,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,157,169,179,190,201,211,221,231,240,248,255
};

// 120 hz
static u8 OD_Table_33x33[OD_TBL_M_SIZE] =
{
	0,8,17,33,46,56,69,80,93,102,110,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,17,32,43,54,66,78,90,102,111,120,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,17,31,40,51,63,74,86,98,110,119,128,136,144,152,160,168,176,184,192,200,208,216,224,232,240,248,253,255,255,255,255,
	0,8,14,24,36,48,59,71,83,95,107,117,127,136,144,152,160,168,176,184,192,200,208,216,224,232,240,247,252,254,255,255,255,
	0,7,11,20,32,44,56,67,80,90,102,112,125,135,144,152,160,168,176,184,192,200,208,216,224,232,240,247,252,254,255,255,255,
	0,6,9,17,28,40,52,64,76,87,98,110,120,133,143,151,160,168,176,184,192,200,208,216,224,232,240,246,251,254,255,255,255,
	0,5,9,14,24,36,48,60,72,83,94,106,118,128,140,150,159,168,176,184,192,200,208,216,224,232,239,246,251,254,255,255,255,
	0,4,6,11,22,32,44,56,68,80,90,102,114,126,136,148,158,167,175,184,192,200,208,216,224,232,239,245,251,254,255,255,255,
	0,2,6,10,18,29,40,52,64,76,88,99,111,122,134,144,156,166,174,183,191,200,208,216,223,231,238,245,250,254,255,255,255,
	0,2,4,9,16,25,37,48,60,72,84,96,107,119,130,142,152,164,173,182,190,199,208,215,223,230,237,244,250,253,255,255,255,
	0,0,1,8,14,22,33,45,56,68,80,92,104,115,127,138,150,160,172,181,189,198,206,215,222,230,237,244,250,253,255,255,255,
	0,0,1,7,12,20,30,41,53,64,76,88,100,112,123,135,146,158,168,178,188,197,205,214,221,229,236,243,250,253,255,255,255,
	0,0,1,6,10,18,26,38,49,60,72,84,96,108,120,131,142,154,166,175,186,196,205,213,220,228,235,243,249,253,255,255,255,
	0,0,0,4,10,17,23,34,46,56,68,80,92,104,116,128,139,150,162,172,182,194,203,211,219,227,235,242,247,253,255,255,255,
	0,0,0,3,9,14,21,30,42,52,64,77,88,100,112,124,136,147,158,168,180,191,203,210,218,226,234,241,247,253,255,255,255,
	0,0,0,2,6,11,19,29,37,49,60,72,85,96,108,120,132,143,155,167,177,190,203,210,218,226,234,241,247,253,255,255,255,
	0,0,0,0,4,9,17,27,33,45,56,69,81,93,104,116,128,137,152,165,181,191,201,212,222,230,234,241,247,253,255,255,255,
	0,0,0,0,3,10,16,25,32,41,52,64,77,89,101,112,124,135,148,162,180,190,199,210,219,229,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,40,50,61,72,86,96,109,119,130,144,157,171,186,198,207,217,226,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,43,52,62,72,82,93,104,114,125,138,152,166,179,191,204,215,225,233,241,247,253,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,66,77,88,101,110,120,132,147,160,173,184,200,211,222,232,241,247,252,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,66,75,85,95,104,116,128,141,155,168,180,192,205,218,227,239,247,252,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,64,72,83,92,100,111,122,136,150,163,175,188,200,213,226,237,246,251,255,255,255,
	0,0,0,0,1,8,16,24,32,40,48,56,64,73,81,89,97,105,116,130,142,153,170,184,196,208,222,236,245,250,255,255,255,
	0,0,0,0,1,8,18,28,36,43,51,59,68,77,85,93,101,110,118,128,139,149,162,179,192,204,216,231,244,249,255,255,255,
	0,0,0,0,1,8,19,29,37,45,53,61,69,77,85,93,101,110,118,128,139,149,162,174,188,200,212,226,241,248,255,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,147,159,170,183,196,208,226,240,247,255,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,166,178,192,204,216,233,246,253,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,166,177,188,200,213,225,242,252,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,155,165,175,186,197,211,223,233,246,255,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,156,166,176,186,198,211,221,232,241,252,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,157,169,179,190,201,211,221,231,240,248,255,
	0,0,0,0,1,10,21,29,37,45,53,61,69,77,85,93,101,110,118,127,136,146,157,169,179,190,201,211,221,231,240,248,255
};

#endif

/// init table
static REG_INIT_TABLE_STRUCT OD_INIT_TABLE[]=
{
    {0, 0x82040201, 0xffffffff},
    // {OD_REG01, 0x00071401, 0x000000ff},
    {0, 0x00070E01, 0x000000ff},
    {0, 0x00000400, 0xffffffff},
    {0, 0x00E0FF04, 0xffffffff},

    {0, 0x00010026, 0xffffffff}, //set cr rate ==0x42
    //{OD_REG32, 0x04055600, 0xffffffff},
    {0, 0x00021000, 0x003fffff},
    {0, 0x0674A0E6, 0xffffffff},
    {0, 0x000622E0, 0xffffffff}, //0x00051170
    {0, 0xE0849C40, 0xffffffff}, //DET8B_POS=4
    {0, 0x05CFDE08, 0xffffffff}, //0x04E6D998
    {0, 0x011F1013, 0xffffffff}, //line size=0xf8 | (0x37 << 13)
    {0, 0x00200000, 0x00ff0000}, //dram_crc_cnt=0x20
    {0, 0x20000610, 0xffffffff}, //enable GM
    {0, 0x001E02D0, 0xffffffff},
    {0, 0x00327C7C, 0xffffffff},
    {0, 0x00180000, 0xffffffff}, //disable film mode detection
    {0, 0x006400C8, 0xffffffff},
    {0, 0x00210032, 0xfffffffc},//pcid_alig_sel=1
    {0, 0x4E00023F, 0xffffffff},
    {0, 0xC306B16A, 0xffffffff}, //pre_bw=3
    {0, 0x10240408, 0xffffffff},  //???00210408???
    {0, 0xC5C00000, 0xC7C00000},
    {0, 0x21A1B800, 0xffffffff},//dump burst
    {0, 0x60000044, 0xe00000ff},//DUMP_WFF_FULL_CONF=3
    {0, 0x2FFF3E00, 0xffffffff},
    {0, 0x80039000, 0xffffffff}, //new 3x3
    {0, 0x352835CA, 0xffffffff}, //skip color thr
    {0, 0x00014438, 0xffffffff}, //pattern gen
    {0, 0x27800898, 0xffffffff},
    {0, 0x00438465, 0xffffffff},
    {0, 0x01180001, 0xffffffff},
    {0, 0x0002D000, 0xffffffff},
    {0, 0x3FFFFFFF, 0xffffffff},
    {0, 0x00000000, 0xffffffff},
    {0, 0x000200C0, 0xffffffff},
    //{OD_REG78, 0x04000000, 0x0C000000}, //force_1st_frame_end
} ;

static void od_init_table_init(void)
{
    OD_INIT_TABLE[0    ].reg_addr = OD_REG00;
    OD_INIT_TABLE[1    ].reg_addr = OD_REG01;
    OD_INIT_TABLE[2    ].reg_addr = OD_REG02;
    OD_INIT_TABLE[3    ].reg_addr = OD_REG03;
    OD_INIT_TABLE[4    ].reg_addr = OD_REG30;
    OD_INIT_TABLE[5    ].reg_addr = OD_REG33;
    OD_INIT_TABLE[6    ].reg_addr = OD_REG34;
    OD_INIT_TABLE[7    ].reg_addr = OD_REG35;
    OD_INIT_TABLE[8    ].reg_addr = OD_REG36;
    OD_INIT_TABLE[9    ].reg_addr = OD_REG37;
    OD_INIT_TABLE[10   ].reg_addr = OD_REG38;
    OD_INIT_TABLE[11   ].reg_addr = OD_REG39;
    OD_INIT_TABLE[12   ].reg_addr = OD_REG40;
    OD_INIT_TABLE[13   ].reg_addr = OD_REG41;
    OD_INIT_TABLE[14   ].reg_addr = OD_REG42;
    OD_INIT_TABLE[15   ].reg_addr = OD_REG43;
    OD_INIT_TABLE[16   ].reg_addr = OD_REG44;
    OD_INIT_TABLE[17   ].reg_addr = OD_REG45;
    OD_INIT_TABLE[18   ].reg_addr = OD_REG46;
    OD_INIT_TABLE[19   ].reg_addr = OD_REG47;
    OD_INIT_TABLE[20   ].reg_addr = OD_REG48;
    OD_INIT_TABLE[21   ].reg_addr = OD_REG49;
    OD_INIT_TABLE[22   ].reg_addr = OD_REG51;
    OD_INIT_TABLE[23   ].reg_addr = OD_REG52;
    OD_INIT_TABLE[24   ].reg_addr = OD_REG53;
    OD_INIT_TABLE[25   ].reg_addr = OD_REG54;
    OD_INIT_TABLE[26   ].reg_addr = OD_REG57;
    OD_INIT_TABLE[27   ].reg_addr = OD_REG62;
    OD_INIT_TABLE[28   ].reg_addr = OD_REG63;
    OD_INIT_TABLE[29   ].reg_addr = OD_REG64;
    OD_INIT_TABLE[30   ].reg_addr = OD_REG65;
    OD_INIT_TABLE[31   ].reg_addr = OD_REG66;
    OD_INIT_TABLE[32   ].reg_addr = OD_REG67;
    OD_INIT_TABLE[33   ].reg_addr = OD_REG68;
    OD_INIT_TABLE[34   ].reg_addr = OD_REG69;
}


static u32 g_od_buf_size = 0;
static unsigned long g_od_buf_pa = NULL;
static void *g_od_buf_va = NULL;

static int od_debug_level = 0;
static int g_od_is_demo_mode = 0;
static int g_od_is_enabled = 0; /* OD is disabled by default */
static int g_od_is_enabled_from_hwc=1;
static int g_od_is_enabled_from_mhl=1;
static int g_od_is_enabled_force=2;
static int g_od_function_availible=1;
static int g_od_counter=0;
static int g_od_trigger=0;
static int g_od_hang_detect_label=0;

unsigned int g_od_err_cnt=0;
unsigned int g_rdma_error_cnt=0;
unsigned int g_rdma_normal_cnt=0;

static OD_STATUS g_od_status;
static OD_STATUS g_od_status_pre;

static ddp_module_notify g_od_ddp_notify = NULL;
struct task_struct *disp_od_thread = NULL;

static void od_refresh_screen(void);
/*
OD reset function for recover from OD hold condition
*/
void disp_od_core_reset(DISP_MODULE_ENUM module, void *cmdq)
{
	unsigned int value=0;
	DISP_REG_MASK(cmdq, DISP_OD_CFG, 1, 0x1);// relay enable

	DISP_REG_MASK(cmdq, DISP_OD_RESET, 0x1, 0x1);// SW reset = 1

	//DDP_REG_POLLING_NEG(OD_STA03, 0x300000);
	value = DISP_REG_GET(OD_STA03);
	ODDBG(OD_DBG_ALWAYS, "DRAM RW reg polling done value=0x%x\n", value);

	DISP_REG_MASK(cmdq, DISP_OD_RESET, 0x0, 0x1);// SW reset = 0

	//rg_od_rst toggle
	OD_REG_SET_FIELD(cmdq, OD_REG01, 1, OD_RST);
	//DDP_REG_POLLING(OD_REG01, 0x1000000);// polling positive
	value = DISP_REG_GET(OD_REG01);
	ODDBG(OD_DBG_ALWAYS, "OD_RST reg polling done value=0x%x\n", value);

	OD_REG_SET_FIELD(cmdq, OD_REG01, 0, OD_RST);
	//DDP_REG_POLLING_NEG(OD_REG01, 0x1000000);// polling negitive
	value = DISP_REG_GET(OD_REG01);
	ODDBG(OD_DBG_ALWAYS, "OD_RST reg polling done value=0x%x\n", value);

	// dram and bypass setting
	OD_REG_SET_FIELD(cmdq, OD_REG37, 0xf, ODT_MAX_RATIO);//ODT ratio=0
	OD_REG_SET_FIELD(cmdq, OD_REG02, 0, ODT_BYPASS);// disable od table bypass
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_HOLD_EN);// enable wdram_hold
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_HOLD_EN);// enable rdram_hold
	OD_REG_SET_FIELD(cmdq, OD_REG39, 0, WDRAM_DIS);// off wdram_disable
   	OD_REG_SET_FIELD(cmdq, OD_REG39, 0, RDRAM_DIS);// off rdram_disable

	//rg_od_start=1
	OD_REG_SET_FIELD(cmdq, OD_REG12, 1, RG_OD_START);
	//rg_od_en=1
	OD_REG_SET_FIELD(cmdq, OD_REG00, 1, OD_ENABLE);


}

/*
OD enable/disable function
*/
void disp_od_core_enabled(void *cmdq, int enabled)
{
#if defined(CONFIG_MTK_OD_SUPPORT)
	unsigned int value=0;
	DISP_REG_MASK(cmdq, DISP_OD_CFG, 1, 0x1);// relay enable
	DISP_REG_MASK(cmdq, DISP_OD_CFG, 0<<1, 0x2);// core disable
	
	DISP_REG_MASK(cmdq, DISP_OD_RESET, 0x1, 0x1);// SW reset = 1
#if 0
	DDP_REG_POLLING_NEG(OD_STA03, 0x300000);
	value = DISP_REG_GET(OD_STA03);
	ODDBG(OD_DBG_ALWAYS, "OD reg polling done value=0x%x\n", value);
#endif
	DISP_REG_MASK(cmdq, DISP_OD_RESET, 0x0, 0x1);// SW reset = 0

	OD_REG_SET_FIELD(cmdq, OD_REG01, 1, OD_RST);//rg_od_rst
	OD_REG_SET_FIELD(cmdq, OD_REG01, 0, OD_RST);

	// dram and bypass setting
	OD_REG_SET_FIELD(cmdq, OD_REG37, 0xf, ODT_MAX_RATIO);//ODT ratio=0
	OD_REG_SET_FIELD(cmdq, OD_REG02, 0, ODT_BYPASS);// disable od table bypass
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_HOLD_EN);// enable wdram_hold
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_HOLD_EN);// enable rdram_hold
	OD_REG_SET_FIELD(cmdq, OD_REG39, 0, WDRAM_DIS);// off wdram_disable
   	OD_REG_SET_FIELD(cmdq, OD_REG39, 0, RDRAM_DIS);// off rdram_disable

	if (enabled==1) {
		DISP_REG_MASK(cmdq, OD_REG00, 0, (1 << 31)); /* bypass_all = 0 */
		OD_REG_SET_FIELD(cmdq, OD_REG12, 1, RG_OD_START);//rg_od_start=1
		DISP_REG_MASK(cmdq, OD_REG00, 1, 1); /* EN = 1 */
		DISP_REG_MASK(cmdq, DISP_OD_CFG, 1<<1, 0x2);// core en
		//DISP_REG_MASK(cmdq, DISP_OD_CFG,    0, 0x1);// relay disable

	} else {
		DISP_REG_MASK(cmdq, OD_REG00, 1, (1 << 31)); /* bypass_all = 1 */
		OD_REG_SET_FIELD(cmdq, OD_REG12, 0, RG_OD_START);//rg_od_start=0
		DISP_REG_MASK(cmdq, OD_REG00, 0, 1); /* EN = 1 */
		DISP_REG_MASK(cmdq, DISP_OD_CFG, 0<<1, 0x2);// core disable
		//DISP_REG_MASK(cmdq, DISP_OD_CFG,    1, 0x1);// relay enable
	}
	ODDBG(OD_DBG_ALWAYS, "disp_od_core_enabled value=%d\n", enabled);
#ifdef OD_MET_DEBUG	
	met_tag_oneshot(40002, "od_switch", (unsigned int)enabled);
#endif
#else
	ODDBG(OD_DBG_ALWAYS, "disp_od_core_enabled: CONFIG_MTK_OD_SUPPORT is not set");
#endif
}

/*
OD irq handler for trigger OD fsm update function
*/
void disp_od_irq_handler() 
{
#if defined(CONFIG_MTK_OD_SUPPORT)
	unsigned int od_istra=DISP_REG_GET(DISP_REG_OD_INTSTA);
	if ((od_istra & 0x1) != 0)
		g_od_trigger = 1;
	else if ((od_istra & 0x40) != 0)
		g_od_err_cnt += 1;
	else;
#endif	
	DISP_REG_SET(NULL, DISP_REG_OD_INTSTA, 0);
}

/*
rdma irq handler for detect rdma underflow
*/
int disp_rdma_notify(unsigned int reg) {
#if defined(CONFIG_MTK_OD_SUPPORT)
	if ((reg &(1 << 4)) != 0) {//
		g_rdma_error_cnt += 1;
#if 1
		OD_REG_SET_FIELD(NULL, OD_REG02, 1, ODT_BYPASS);// bypass OD
		OD_REG_SET_FIELD(NULL, OD_REG39, 1, WDRAM_DIS);// disable OD WDRAM
    	OD_REG_SET_FIELD(NULL, OD_REG39, 1, RDRAM_DIS);// disable OD RDRAM
#else
		OD_REG_SET_FIELD(NULL, OD_REG37, 0, ODT_MAX_RATIO);//ODT ratio=0
		OD_REG_SET_FIELD(NULL, OD_REG71, 0, RG_WDRAM_HOLD_EN);
    	OD_REG_SET_FIELD(NULL, OD_REG72, 0, RG_RDRAM_HOLD_EN);
#endif		
    	DISP_REG_MASK(NULL, DISP_OD_CFG, 1, 0x1);// relay enable
		met_tag_oneshot(40003, "rdma_underflow", 1);

	} else if ((reg & (1 << 1)) != 0) {
		g_rdma_normal_cnt += 1;
	}
#endif	
	return 0;
}

int disp_od_check_status() {
	DISP_REG_MASK(NULL, DISP_OD_CFG, 1, 0x1);// relay enable
	OD_REG_SET_FIELD(NULL, OD_REG71, 0, RG_WDRAM_HOLD_EN);
   	OD_REG_SET_FIELD(NULL, OD_REG72, 0, RG_RDRAM_HOLD_EN);
	g_od_status = OD_STATUS_RESET;
	met_tag_oneshot(40001, "od_hang", 1);
	return 0;
}

/*
OD state update function
*/
int disp_od_update_status(void *cmdq)
{
#if defined(CONFIG_MTK_OD_SUPPORT)
	unsigned int od_status_sta  =DISP_REG_GET(DISP_REG_OD_STATUS);// input/output valid ready flag
	unsigned int od_status_sta_2=DISP_REG_GET(OD_STA02);// 
	unsigned int od_status_sta_3=DISP_REG_GET(OD_STA03);// WDRAM_HOLD AND RDRAM_HOLD flag
	unsigned int od_control_2a  =DISP_REG_GET(OD_REG71);// WDRAM_HOLD_EN
	unsigned int od_control_2b  =DISP_REG_GET(OD_REG72);// RDRAM_HOLD_EN
	//ODDBG(OD_DBG_ALWAYS, "disp_od_update_status=%d normal:%d error:%d", g_od_status, g_rdma_normal_cnt, g_rdma_error_cnt);
	if (g_rdma_error_cnt>0) {// detect rdma underflow
		if (g_od_status != OD_STATUS_RESET) {
			ODDBG(OD_DBG_ALWAYS,"disp_od_update_status detect_rdma_underflow");
			OD_REG_SET_FIELD(cmdq, OD_REG71, 0, RG_WDRAM_HOLD_EN);// disable WDRAM_HOLD
    		OD_REG_SET_FIELD(cmdq, OD_REG72, 0, RG_RDRAM_HOLD_EN);// disable RDRAM_HOLD
			DISP_REG_MASK(cmdq, DISP_OD_CFG, 1, 0x1);// relay enable
			g_od_counter = 0;
			g_rdma_error_cnt = 0;
			g_rdma_normal_cnt = 0;
			g_od_function_availible = 0;
			g_od_hang_detect_label = 1;
			g_od_status = OD_STATUS_RESET;
			od_refresh_screen ();
		}

	//} else if( ((od_status_sta_3 & 0xc0000) != 0) || ((od_control_2a & 0x20000000) == 0) || ((od_control_2b & 0x20000000) == 0) ) {
	} else if( ((od_status_sta_3 & 0xc0000) != 0) ) {// detect OD dram hold condition
		if (g_od_status != OD_STATUS_RESET) {
			ODDBG(OD_DBG_ALWAYS,"disp_od_update_status detect_hold");
			//OD_REG_SET_FIELD(NULL, OD_REG02, 1, ODT_BYPASS);
			//OD_REG_SET_FIELD(NULL, OD_REG39, 1, WDRAM_DIS);
    		//OD_REG_SET_FIELD(NULL, OD_REG39, 1, RDRAM_DIS);
			OD_REG_SET_FIELD(NULL, OD_REG71, 0, RG_WDRAM_HOLD_EN);// disable WDRAM_HOLD
    		OD_REG_SET_FIELD(NULL, OD_REG72, 0, RG_RDRAM_HOLD_EN);// disable RDRAM_HOLD
			DISP_REG_MASK(NULL, DISP_OD_CFG, 1, 0x1);// relay enable
			g_od_counter = 0;
			g_rdma_error_cnt = 0;
			g_rdma_normal_cnt = 0;
			g_od_function_availible = 0;
			g_od_hang_detect_label = 1;
			g_od_status = OD_STATUS_RESET;
			od_refresh_screen ();
		}

	} else if(g_od_is_enabled != g_od_is_enabled_from_mhl) {// detect mhl or wfd conflict with corrent OD on/off setting
		if (g_od_is_enabled_from_mhl == 0) {
			disp_od_set_enabled(cmdq, 0);// disable OD when wfd or mhl enable
		}

	} else {
		g_od_hang_detect_label = 0;
		g_od_function_availible = 1;

	}

	if (g_od_trigger == 1) {
		ODDBG(OD_DBG_ALWAYS, "disp_od_update_status=%d(available=%d,cnt=%d,tri=%d,err_cnt=%d) sta1=0x%x sta2=0x%x", 
			g_od_status, g_od_function_availible, g_od_counter, g_od_trigger, g_od_err_cnt,
			od_status_sta_2, od_status_sta_3);
	}

	// OD state case
	switch(g_od_status) {
		case OD_STATUS_IDLE:// idle state : no action
			if (g_od_trigger == 1) {
				g_od_counter = 0;
				g_od_trigger = 0;
			}
		
#ifdef OD_MET_DEBUG				
			met_tag_oneshot(40001, "od_hang", 0);
#endif
			break;

		case OD_STATUS_ONOFF:// enable/disable command
			if (g_od_trigger == 1) {
				g_od_status = OD_STATUS_ENABLE;
				g_od_counter = 0;
				g_od_trigger = 0;
				disp_od_core_enabled(cmdq, g_od_is_enabled);
			}
			break;

		case OD_STATUS_ENABLE:
			if (g_od_trigger == 1) {
				g_od_counter += 1;
				if (g_od_counter >= 8) {
					DISP_REG_MASK(cmdq, DISP_OD_CFG, 0, 0x1);// relay disable
					g_od_status = OD_STATUS_IDLE;
				} else;
				g_od_trigger = 0;
			}
#ifdef OD_MET_DEBUG				
			met_tag_oneshot(40001, "od_hang", 0);
#endif
			break;
			
		case OD_STATUS_BYPASS:
			g_od_counter+=1;
			break;
			
		case OD_STATUS_RESET:// reset state for run reset flow
			if (g_od_trigger == 1) {
				if (g_od_function_availible == 1) {
					disp_od_core_reset(DISP_MODULE_OD, cmdq);
					g_od_counter = 0;
					g_od_status = OD_STATUS_IDLE;
					
				} else {
					g_od_counter+=1;
					g_od_counter&=0x7;
					if (g_od_counter == 0x0) {
						OD_REG_SET_FIELD(cmdq, OD_REG71, 0, RG_WDRAM_HOLD_EN);// disable OD WDRAM HOLD
    					OD_REG_SET_FIELD(cmdq, OD_REG72, 0, RG_RDRAM_HOLD_EN);// disable OD RDRAM HOLD
					} else if (g_od_counter == 0x4)
						disp_od_core_reset(DISP_MODULE_OD, cmdq);// reset OD engine
					else if (g_od_counter == 0x1)
						disp_od_core_enabled(cmdq, 0);// disable OD engine
					g_od_status = OD_STATUS_RESET;
				}
				g_od_trigger = 0;
			}
#ifdef OD_MET_DEBUG	
			met_tag_oneshot(40001, "od_hang", 1);
#endif
			break;

		default:
			break;
	}
	g_od_status_pre = g_od_status;
	
#endif
	return 0;
}

static void _od_reg_init(void *cmdq, const REG_INIT_TABLE_STRUCT *init_table, int tbl_size)
{
   int index = 0;

   for (index = 0; index < tbl_size; index++, init_table++) {
       DISP_REG_SET(cmdq, init_table->reg_addr, (init_table->reg_value & init_table->reg_mask));
   }
}


static void od_refresh_screen(void)
{
    if (g_od_ddp_notify != NULL)
        g_od_ddp_notify(DISP_MODULE_OD, DISP_PATH_EVENT_TRIGGER);
}

static void _od_set_dram_buffer_addr(void *cmdq, int manual_comp, int image_width, int image_height)
{
    u32 u4ODDramSize;
    u32 u4Linesize;
    u32 od_buf_pa_32;

    static int is_inited = 0;

    // set line size
    u4Linesize = ((image_width * manual_comp) >> 9) + 2;  // linesize = ( h active/4* manual CR )/128  ==>linesize = (h active * manual CR)/512

    u4ODDramSize = u4Linesize * (image_height / 2) * 16;

    if (!is_inited)
    {
        void *va;
        dma_addr_t dma_addr;

        va = dma_alloc_coherent(disp_get_device(), u4ODDramSize + OD_ADDITIONAL_BUFFER + OD_GUARD_PATTERN_SIZE, &dma_addr, GFP_KERNEL);

        if (va == NULL) {
            ODDBG(OD_DBG_ALWAYS, "OD: MEM NOT ENOUGH %d", u4Linesize);
            BUG();
        }

        ODDBG(OD_DBG_ALWAYS, "OD: pa %08lx size %d order %d va %lx\n",
            (unsigned long)(dma_addr), u4ODDramSize, get_order(u4ODDramSize), (unsigned long)va);

        is_inited = 1;

        g_od_buf_size = u4ODDramSize;
        g_od_buf_pa = (unsigned long)dma_addr;
        g_od_buf_va = va;

        /* set guard pattern */
        *((u32*)((unsigned long)va + u4ODDramSize)) = OD_GUARD_PATTERN;
        *((u32*)((unsigned long)va + u4ODDramSize + OD_ADDITIONAL_BUFFER)) = OD_GUARD_PATTERN;

    }

    od_buf_pa_32 = (u32)g_od_buf_pa;

    OD_REG_SET_FIELD(cmdq, OD_REG06, (od_buf_pa_32 >> 4), RG_BASE_ADR);
    OD_REG_SET_FIELD(cmdq, OD_REG56, ((od_buf_pa_32 + u4ODDramSize) >> 4), DRAM_UPBOUND);
    OD_REG_SET_FIELD(cmdq, OD_REG56, 1, DRAM_PROT);
}


static void _od_set_frame_protect_init(void *cmdq, int image_width, int image_height)
{
    OD_REG_SET_FIELD(cmdq, OD_REG08, image_width, OD_H_ACTIVE );
    OD_REG_SET_FIELD(cmdq, OD_REG32, image_width, OD_DE_WIDTH);

    OD_REG_SET_FIELD(cmdq, OD_REG08, image_height, OD_V_ACTIVE);

    OD_REG_SET_FIELD(cmdq, OD_REG53, 0x0BFB, FRAME_ERR_CON);   /// don't care v blank
    OD_REG_SET_FIELD(cmdq, OD_REG09, 0x01E, RG_H_BLANK);       /// h_blank  = htotal - h_active
    OD_REG_SET_FIELD(cmdq, OD_REG09, 0x0A, RG_H_OFFSET);       /// tolerrance

    OD_REG_SET_FIELD(cmdq, OD_REG10, 0xFFF, RG_H_BLANK_MAX);
    OD_REG_SET_FIELD(cmdq, OD_REG10, 0x3FFFF, RG_V_BLANK_MAX); /// pixel-based counter

    OD_REG_SET_FIELD(cmdq, OD_REG11, 0xB000, RG_V_BLANK);      /// v_blank  = vtotal - v_active
    OD_REG_SET_FIELD(cmdq, OD_REG11, 2, RG_FRAME_SET);
}


static void _od_set_low_power(void *cmdq)
{
	/*
	low power setting for experiment
	
	0x700, 29b = 1, rg_od_clken
	rg_odt_ink_en = 0 ( OK)
	OD_EXT_FP_EN = 0 ( OK)
	rg_gm_en = 0(option, must try)
	rg_od_ecp_en = 0, rg_od_ecp_all_on =0 ( OK)
	rg_2ch_ptgen_2ch_en = 0 ( OK)
	rg_pkt_crc_en = 0 (option, try)
	LOCA_TABLE_EN = 0 ( OK)
	*/
	OD_REG_SET_FIELD(cmdq, OD_REG03, 0, ODT_INK_EN);
	
	OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_EXT_FP_EN);

	OD_REG_SET_FIELD(cmdq, OD_REG40, 0, GM_EN);

	OD_REG_SET_FIELD(cmdq, OD_REG56, 0, OD_ECP_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG56, 0, OD_ECP_ALL_ON);

	OD_REG_SET_FIELD(cmdq, OD_REG63, 0, RG_2CH_PTGEN_2CH_EN);
	
	OD_REG_SET_FIELD(cmdq, OD_REG_CRC32_2, 0, PKT_CRC_EN);

	OD_REG_SET_FIELD(cmdq, OD_REG_LT_00, 0, LOCA_TABLE_EN);
}

static void _od_set_param(void *cmdq, int manual_comp, int image_width, int image_height)
{
    u32 u4GMV_width;
    u32 u4Linesize;

    //set gmv detection width
    u4GMV_width = image_width / 6;

    OD_REG_SET_FIELD(cmdq, OD_REG40, (u4GMV_width*1)>>4, GM_R0_CENTER);
    OD_REG_SET_FIELD(cmdq, OD_REG40, (u4GMV_width*2)>>4, GM_R1_CENTER);
    OD_REG_SET_FIELD(cmdq, OD_REG41, (u4GMV_width*3)>>4, GM_R2_CENTER);
    OD_REG_SET_FIELD(cmdq, OD_REG41, (u4GMV_width*4)>>4, GM_R3_CENTER);
    OD_REG_SET_FIELD(cmdq, OD_REG42, (u4GMV_width*5)>>4, GM_R4_CENTER);

    OD_REG_SET_FIELD(cmdq, OD_REG43, 12 >> 2                 , GM_V_ST );
    OD_REG_SET_FIELD(cmdq, OD_REG43, (image_height-12)>>2    , GM_V_END);
    OD_REG_SET_FIELD(cmdq, OD_REG42, (100*image_height)/1080 , GM_LGMIN_DIFF);
    OD_REG_SET_FIELD(cmdq, OD_REG44, (400*image_height)/1080 , GM_LMIN_THR);
    OD_REG_SET_FIELD(cmdq, OD_REG44, (200*image_height)/1080 , GM_GMIN_THR);

    //set compression ratio
    OD_REG_SET_FIELD(cmdq, OD_REG30, manual_comp, MANU_CPR);

	OD_REG_SET_FIELD(cmdq, OD_REG00, 0xf, DC_CNT);
	OD_REG_SET_FIELD(cmdq, OD_REG00, 0x1e, BTC_ERR_CNT);
	OD_REG_SET_FIELD(cmdq, OD_REG00, 0x1, C_GAT_CNT);

    // set line size
    u4Linesize = ((image_width * manual_comp) >> 9) + 2;  // linesize = ( h active/4* manual CR )/128  ==>linesize = (h active * manual CR)/512

    OD_REG_SET_FIELD(cmdq, OD_REG47, 0, PRE_BW);  //vIO32WriteFldAlign(OD_REG47, 3, PRE_BW);
#if 1
	OD_REG_SET_FIELD(cmdq, OD_REG34, 0x18, ODT_SB_TH0);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x19, ODT_SB_TH1);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x1A, ODT_SB_TH2);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x1B, ODT_SB_TH3);

    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x1C, ODT_SB_TH4);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x1D, ODT_SB_TH5);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x1E, ODT_SB_TH6);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x1F, ODT_SB_TH7);
#else
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0xF,  ODT_SB_TH0);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x10, ODT_SB_TH1);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x11, ODT_SB_TH2);
    OD_REG_SET_FIELD(cmdq, OD_REG34, 0x12, ODT_SB_TH3);

    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x13, ODT_SB_TH4);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x14, ODT_SB_TH5);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x15, ODT_SB_TH6);
    OD_REG_SET_FIELD(cmdq, OD_REG47, 0x16, ODT_SB_TH7);
#endif
    OD_REG_SET_FIELD(cmdq, OD_REG38, u4Linesize, LINE_SIZE);

    // use 64 burst length
    OD_REG_SET_FIELD(cmdq, OD_REG38, 3, WR_BURST_LEN);
    OD_REG_SET_FIELD(cmdq, OD_REG38, 3, RD_BURST_LEN);

    //set auto 8bit parameters
    if (image_width > 1900)
    {
        OD_REG_SET_FIELD(cmdq, OD_REG35, (140000 << 0), DET8B_DC_NUM );
        OD_REG_SET_FIELD(cmdq, OD_REG36, (40000 << 18), DET8B_BTC_NUM);
        OD_REG_SET_FIELD(cmdq, OD_REG37, (1900000>>4) , DET8B_BIT_MGN);
    }
    else
    {
        OD_REG_SET_FIELD(cmdq, OD_REG35, 70000, DET8B_DC_NUM);
        OD_REG_SET_FIELD(cmdq, OD_REG36, 20000, DET8B_BTC_NUM);
        OD_REG_SET_FIELD(cmdq, OD_REG37, (950000>>4) ,DET8B_BIT_MGN);
    }

    //set auto Y5 mode thr
    OD_REG_SET_FIELD(cmdq, OD_REG46,  0x4E00, AUTO_Y5_NUM);
    OD_REG_SET_FIELD(cmdq, OD_REG53,  0x4E00, AUTO_Y5_NUM_1);

    //set OD threshold
    OD_REG_SET_FIELD(cmdq, OD_REG01, 10, MOTION_THR);
    OD_REG_SET_FIELD(cmdq, OD_REG48, 8, ODT_INDIFF_TH);
    OD_REG_SET_FIELD(cmdq, OD_REG02, 1, FBT_BYPASS);

    //set dump param
    OD_REG_SET_FIELD(cmdq, OD_REG51, 0, DUMP_STLINE);
    OD_REG_SET_FIELD(cmdq, OD_REG51, (image_height-1), DUMP_ENDLINE);

    //set compression param
    OD_REG_SET_FIELD(cmdq, OD_REG77, 0xfc, RC_U_RATIO);
    OD_REG_SET_FIELD(cmdq, OD_REG78, 0xfc, RC_U_RATIO_FIRST2);
    OD_REG_SET_FIELD(cmdq, OD_REG77, 0x68, RC_L_RATIO);

    OD_REG_SET_FIELD(cmdq, OD_REG78, 0x68, RC_L_RATIO_FIRST2);
    OD_REG_SET_FIELD(cmdq, OD_REG76, 0x3, CHG_Q_FREQ);

    OD_REG_SET_FIELD(cmdq, OD_REG76, 0x2, CURR_Q_UV);
    OD_REG_SET_FIELD(cmdq, OD_REG76, 0x2, CURR_Q_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG77, 0x8, IP_SAD_TH);

	// 6795 preutila
#if 0
	OD_REG_SET_FIELD(cmdq, OD_REG71, 60, RG_WGULTRA_L2_THR);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WGULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 60, RG_WGPREULTRA_L2_THR);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WGPREULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_HOLD_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_LEN_X8);
	
	OD_REG_SET_FIELD(cmdq, OD_REG71, 60, RG_RGULTRA_L2_THR);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_RGULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 60, RG_RGPREULTRA_L2_THR);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RGPREULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_HOLD_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_LEN_X8);
#else
	OD_REG_SET_FIELD(cmdq, OD_REG71, 6, RG_WR_HIGH);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 2, RG_WR_PRE_HIGH);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WRULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 2, RG_WR_LOW);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 6, RG_WR_PRELOW);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WGPREULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_HOLD_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG71, 1, RG_WDRAM_LEN_X8);
	
	OD_REG_SET_FIELD(cmdq, OD_REG72, 6, RG_RD_HIGH);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 2, RG_RD_PRE_HIGH);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 2, RG_RD_LOW);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 6, RG_RD_PRELOW);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RGPREULTRA_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_HOLD_EN);
	OD_REG_SET_FIELD(cmdq, OD_REG72, 1, RG_RDRAM_LEN_X8);
#endif
	// increase burst length for enhance memory efficiency
	OD_REG_SET_FIELD(cmdq, OD_REG38, 3, WR_BURST_LEN);
	OD_REG_SET_FIELD(cmdq, OD_REG38, 3, RD_BURST_LEN);
}


static void _od_write_table(void *cmdq, u8 TableSel, u8 ColorSel, u8* pTable, int table_inverse)
{
    u32 i, u4TblSize;
    u32 u1ODBypass   = DISP_REG_GET(OD_REG02) & (1 << 9)  ? 1 : 0;
    u32 u1FBBypass   = DISP_REG_GET(OD_REG02) & (1 << 10) ? 1 : 0;
    u32 u1PCIDBypass = DISP_REG_GET(OD_REG02) & (1 << 21) ? 1 : 0;

    if (ColorSel > 3)
    {
        return;
    }

    // disable OD_START
    DISP_REG_SET(cmdq, OD_REG12, 0);

    OD_REG_SET_FIELD(cmdq, OD_REG02, 1, ODT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG02, 1, FBT_BYPASS);

    OD_REG_SET_FIELD(cmdq, OD_REG45, 0, OD_PCID_EN);
    OD_REG_SET_FIELD(cmdq, OD_REG45, 1, OD_PCID_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG04, 1, TABLE_ONLY_W_ADR_INC);
    OD_REG_SET_FIELD(cmdq, OD_REG04, 0, ADDR_YX);
    OD_REG_SET_FIELD(cmdq, OD_REG04, 0x3, TABLE_RW_SEL_OD_BGR);

    if (ColorSel == 3)
    {
        OD_REG_SET_FIELD(cmdq, OD_REG04, 7, TABLE_RW_SEL_OD_BGR);
    }
    else
    {
        OD_REG_SET_FIELD(cmdq, OD_REG04, (1 << ColorSel), TABLE_RW_SEL_OD_BGR);
    }

    switch (TableSel)
    {
        case OD_TABLE_33:
            u4TblSize = OD_TBL_M_SIZE;
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_41);
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_17);
            break;

        case OD_TABLE_17:
            u4TblSize = OD_TBL_S_SIZE;
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_41);
            OD_REG_SET_FIELD(cmdq, OD_REG32, 1, OD_IDX_17);
            break;

        default:
            return;
    }

    for (i = 0; i < u4TblSize; i++)
    {
       if (table_inverse)
       {
          u8 value = ABS(255 - *(pTable+i));
          DISP_REG_SET(cmdq, OD_REG05, value);
       }
       else
       {
          DISP_REG_SET(cmdq, OD_REG05, *(pTable+i));
       }
    }

    DISP_REG_SET(cmdq, OD_REG04, 0);
    OD_REG_SET_FIELD(cmdq, OD_REG02,  u1ODBypass, ODT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG02,  u1FBBypass, FBT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG45,  (!u1PCIDBypass), OD_PCID_EN);
    OD_REG_SET_FIELD(cmdq, OD_REG45,  (u1PCIDBypass), OD_PCID_BYPASS);
}


static u8 _od_read_table(void *cmdq, u8 TableSel, u8 ColorSel, u8* pTable, int table_inverse)
{
    u32 i, u4TblVal, u4TblSize, u4ErrCnt = 0;
    u32 mask;
    u32 u1ODBypass    = DISP_REG_GET(OD_REG02) & (1 << 9)  ? 1 : 0;
    u32 u1FBBypass    = DISP_REG_GET(OD_REG02) & (1 << 10) ? 1 : 0;
    u32 u1PCIDBypass  = DISP_REG_GET(OD_REG02) & (1 << 21) ? 1 : 0;

    if (ColorSel > 2)
    {
        return 1;
    }

    OD_REG_SET_FIELD(cmdq, OD_REG02, u1ODBypass, ODT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG02, u1FBBypass, FBT_BYPASS);

    OD_REG_SET_FIELD(cmdq, OD_REG04, 0, TABLE_ONLY_W_ADR_INC);
    OD_REG_SET_FIELD(cmdq, OD_REG04, 0, ADDR_YX);

    mask = ~(0x7 << 19);
    OD_REG_SET_FIELD(cmdq, OD_REG04, 7, TABLE_RW_SEL_OD_BGR);

    switch (TableSel)
    {
        case OD_TABLE_33:
            u4TblSize = OD_TBL_M_SIZE;
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_41);
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_17);
            break;

        case OD_TABLE_17:
            u4TblSize = OD_TBL_S_SIZE;
            OD_REG_SET_FIELD(cmdq, OD_REG32, 0, OD_IDX_41);
            OD_REG_SET_FIELD(cmdq, OD_REG32, 1, OD_IDX_17);
            break;

        default:
            return 0;
    }

    for (i = 0; i < u4TblSize; i++)
    {
        u4TblVal = DISP_REG_GET(OD_REG05);

        if (table_inverse)
        {
            u8 value = ABS(255 - *(pTable+i));
            if (value != u4TblVal)
            {
               u4ErrCnt++;
            }
        }
        else
        {

            if (*(pTable+i) != u4TblVal)
            {
               ODDBG(OD_DBG_ALWAYS, "OD %d TBL %d %d != %d\n", ColorSel, i, *(pTable+i), u4TblVal);
               u4ErrCnt++;
            }
        }
    }

    DISP_REG_SET(cmdq, OD_REG04, 0);
    OD_REG_SET_FIELD(cmdq, OD_REG02, u1ODBypass, ODT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG02, u1FBBypass, FBT_BYPASS);
    OD_REG_SET_FIELD(cmdq, OD_REG45, !u1PCIDBypass, OD_PCID_EN);
    OD_REG_SET_FIELD(cmdq, OD_REG45, u1PCIDBypass, OD_PCID_BYPASS);

    return u4ErrCnt;
}


static void _od_set_table(void *cmdq, int tableSelect, u8 *od_table, int table_inverse)
{
    /// Write OD table
    if (OD_TABLE_17 == tableSelect)
    {
       _od_write_table(cmdq, OD_TABLE_17, OD_ALL, od_table, table_inverse);

       /// Check OD table
       /// #ifndef DEF_CMDQ
       /// _od_read_table(OD_TABLE_17, OD_RED, OD_Table_17x17, table_inverse);
       /// _od_read_table(OD_TABLE_17, OD_GREEN, OD_Table_17x17, table_inverse);
       /// _od_read_table(OD_TABLE_17, OD_BLUE, OD_Table_17x17, table_inverse);
       /// #endif

       DISP_REG_SET(cmdq, OD_REG02, (1 << 10));
       OD_REG_SET_FIELD(cmdq, OD_REG45, 1, OD_PCID_BYPASS);
       OD_REG_SET_FIELD(cmdq, OD_REG45, 0, OD_PCID_EN);
       DISP_REG_SET(cmdq, OD_REG12, 1 << 0);
    }
    else if (OD_TABLE_33 == tableSelect)
    {
      _od_write_table(cmdq, OD_TABLE_33, OD_ALL, od_table, table_inverse);

        /// #ifndef DEF_CMDQ
        /// /// Check OD table
        /// _od_read_table(OD_TABLE_33, OD_RED, OD_Table_33x33, table_inverse);
        /// _od_read_table(OD_TABLE_33, OD_GREEN, OD_Table_33x33, table_inverse);
        /// _od_read_table(OD_TABLE_33, OD_BLUE, OD_Table_33x33, table_inverse);
        /// #endif

       DISP_REG_SET(cmdq, OD_REG02, (1 << 10));
       OD_REG_SET_FIELD(cmdq, OD_REG45, 1, OD_PCID_BYPASS);
       OD_REG_SET_FIELD(cmdq, OD_REG45, 0, OD_PCID_EN);
       DISP_REG_SET(cmdq, OD_REG12, 1 << 0);
    }
    else
    {
       ODDBG(OD_DBG_ALWAYS, "Error OD table\n");
       BUG();
    }
}

static void od_dbg_dump(void)
{
    ODDBG(OD_DBG_ALWAYS, "OD EN %d INPUT %d %d\n", DISP_REG_GET(DISP_OD_EN), DISP_REG_GET(DISP_OD_INPUT_COUNT) >> 16, DISP_REG_GET(DISP_OD_INPUT_COUNT) & 0xFFFF);
    ODDBG(OD_DBG_ALWAYS, "STA 0x%08x\n", DISP_REG_GET(OD_STA00));
    ODDBG(OD_DBG_ALWAYS, "REG49 0x%08x\n", DISP_REG_GET(OD_REG49));
}

static void od_test_stress_table(void *cmdq)
{
    int i;

    ODDBG(OD_DBG_ALWAYS, "OD TEST -- STRESS TABLE START\n");

    DISP_REG_SET(cmdq, DISP_OD_MISC, 1); /// [1]:can access OD table; [0]:can't access OD table

    /* read/write table for 100 times, 17x17 and 33x33 50 times each */
    for (i = 0; i < 50; i++)
    {
        /* test 17 table */
        _od_set_table(cmdq, OD_TABLE_17, OD_Table_17x17, 0);
        _od_read_table(cmdq, OD_TABLE_17, 0, OD_Table_17x17, 0);
        _od_read_table(cmdq, OD_TABLE_17, 1, OD_Table_17x17, 0);
        _od_read_table(cmdq, OD_TABLE_17, 2, OD_Table_17x17, 0);

        /* test 33 table */
        _od_set_table(cmdq, OD_TABLE_33, OD_Table_33x33, 0); /// default use 17x17 table
        _od_read_table(cmdq, OD_TABLE_33, 0, OD_Table_33x33, 0);
        _od_read_table(cmdq, OD_TABLE_33, 1, OD_Table_33x33, 0);
        _od_read_table(cmdq, OD_TABLE_33, 2, OD_Table_33x33, 0);
    }

    DISP_REG_SET(cmdq, DISP_OD_MISC, 0); /// [1]:can access OD table; [0]:can't access OD table

    ODDBG(OD_DBG_ALWAYS, "OD TEST -- STRESS TABLE END\n");
}


void disp_config_od(unsigned int width, unsigned int height, void *cmdq, unsigned int od_table_size, void *od_table)
{
    int manual_cpr = OD_MANUAL_CPR;

    int od_table_select = 0;
#ifdef OD_MET_DEBUG		
	met_tag_init();
#endif
    ODDBG(OD_DBG_ALWAYS, "OD conf start %lx %x %lx\n", (unsigned long)cmdq, od_table_size, (unsigned long)od_table);

    switch (od_table_size)
    {
    case 17*17:
        od_table_select = OD_TABLE_17;
        break;

    case 33*33:
        od_table_select = OD_TABLE_33;
        break;

    // default linear table
    default:
        od_table_select = OD_TABLE_17;
        od_table = OD_Table_dummy_17x17;
        break;
    }

    if (od_table == NULL)
    {
        ODDBG(OD_DBG_ALWAYS, "LCM NULL OD table\n");
        BUG();
    }

    DISP_REG_SET(cmdq, DISP_OD_EN, 1);
#if 0
	// tuning timing for 120hz
    DISP_REG_SET(cmdq, DISP_OD_SIZE, (width << 16) | height);
    DISP_REG_SET(cmdq, DISP_OD_HSYNC_WIDTH, OD_HSYNC_WIDTH);
    DISP_REG_SET(cmdq, DISP_OD_VSYNC_WIDTH, (OD_HSYNC_WIDTH << 16)| (width * 2 / 2));
#else
	DISP_REG_SET(cmdq, DISP_OD_SIZE, (width << 16) | height);
    DISP_REG_SET(cmdq, DISP_OD_HSYNC_WIDTH, OD_HSYNC_WIDTH);
    DISP_REG_SET(cmdq, DISP_OD_VSYNC_WIDTH, (OD_HSYNC_WIDTH << 16)| (width * 3 / 2));
#endif
    /// OD register init
    od_init_table_init();

    _od_reg_init(cmdq, OD_INIT_TABLE, sizeof(OD_INIT_TABLE) / sizeof(OD_INIT_TABLE[0]));
    _od_set_dram_buffer_addr(cmdq, manual_cpr, width, height);


    _od_set_frame_protect_init(cmdq, width, height);

    /// OD on/off align to vsync
    DISP_REG_SET(cmdq, OD_REG53, DISP_REG_GET(OD_REG53) | (1 << 30));

    // _od_set_param(38, width, height);
    _od_set_param(cmdq, manual_cpr, width, height);

	//_od_set_low_power(cmdq);

    DISP_REG_SET(cmdq, OD_REG53, 0x6BFB7E00);

    DISP_REG_SET(cmdq, DISP_OD_MISC, 1); /// [1]:can access OD table; [0]:can't access OD table

    // _od_set_table(OD_TABLE_17, 0); /// default use 17x17 table
    _od_set_table(cmdq, od_table_select, od_table, 0);

    DISP_REG_SET(cmdq, DISP_OD_MISC, 0); /// [1]:can access OD table; [0]:can't access OD table

    /// modified ALBUF2_DLY OD_REG01

    // OD_REG_SET_FIELD(cmdq, OD_REG01, (0xD), ALBUF2_DLY); // 1080w
    OD_REG_SET_FIELD(cmdq, OD_REG01, (0xE), ALBUF2_DLY); // 720w

    /// enable INK
    // OD_REG_SET_FIELD(cmdq, OD_REG03, 1, ODT_INK_EN);

    // disable hold for debug
    //OD_REG_SET_FIELD(cmdq, OD_REG71, 0, RG_WDRAM_HOLD_EN);
    //OD_REG_SET_FIELD(cmdq, OD_REG72, 0, RG_RDRAM_HOLD_EN);

    // enable debug OSD for status reg
    // OD_REG_SET_FIELD(cmdq, OD_REG46, 1, OD_OSD_SEL);

    // lower hold threshold
    // OD_REG_SET_FIELD(cmdq, OD_REG73, 0, RG_WDRAM_HOLD_THR);
    // OD_REG_SET_FIELD(cmdq, OD_REG73, 0, RG_RDRAM_HOLD_THR);

    /* restore demo mode for suspend / resume */
    if (g_od_is_demo_mode)
        OD_REG_SET_FIELD(cmdq, OD_REG02, 1, DEMO_MODE);

    OD_REG_SET_FIELD(cmdq, OD_REG00, 0, BYPASS_ALL);

    // GO OD. relay = 0, od_core_en = 1, DITHER_EN = 1
    // Dynamic turn on from API
#if 0
    if (g_od_is_enabled)
        spm_enable_sodi(0); /* To make sure SODI is disabled */
#endif
        
    //DISP_REG_MASK(cmdq, DISP_OD_CFG, (g_od_is_enabled << 1), 0x3); /* Also disable relay mode */
    DISP_REG_MASK(cmdq, DISP_OD_CFG, 0<<1, 0x2);// core en
	DISP_REG_MASK(cmdq, DISP_OD_CFG,    1, 0x1);// relay enable


    // clear crc error first
    OD_REG_SET_FIELD(cmdq, OD_REG38, 1, DRAM_CRC_CLR);


    if (od_debug_level >= OD_DBG_DEBUG)
        od_dbg_dump();

    ODDBG(OD_DBG_ALWAYS, "OD inited W %d H %d \n", width, height);

	g_od_function_availible = 1;
	g_od_counter = 0;
	g_od_status = OD_STATUS_IDLE;
	disp_od_core_enabled(cmdq, 0);//default off OD
	DISP_REG_MASK(cmdq, DISP_REG_OD_INTEN, 0x43, 0xff);

}

int disp_od_get_enabled(void)
{
	return g_od_is_enabled;
}

/*
api for mhl and wfd to on/off OD
*/
void disp_od_mhl_force(int mhl)
{
	g_od_is_enabled_from_mhl = mhl;
}

void disp_od_set_enabled(void *cmdq, int enabled)
{
#if defined(CONFIG_MTK_OD_SUPPORT)
	if (g_od_function_availible == 0) {
		printk(KERN_DEBUG "[OD] disp_od_set_enabled un-availible");
		return;
	}

	if (g_od_is_enabled_force == 2)// auto seitch mode for normal run
		//g_od_is_enabled = (enabled & 0x1 & g_od_is_enabled_from_hwc);
		g_od_is_enabled = (enabled & 0x1 & g_od_is_enabled_from_mhl);// force od disble if mhl flag off (mhl or wfd is running)
	else if (g_od_is_enabled_force == 1)// force od enable for debug
		g_od_is_enabled = 1;
	else // force od disable for debug
		g_od_is_enabled = 0;
	
    printk(KERN_DEBUG "[OD] disp_od_set_enabled=%d (in:%d)(hwc:%d) (mhl:%d)\n", \
		g_od_is_enabled, enabled, g_od_is_enabled_from_hwc, g_od_is_enabled_from_mhl);
#if 0    
		if (g_od_is_enabled)
			spm_enable_sodi(0); /* To make sure SODI is disabled */
#endif
	if (g_od_function_availible == 1)
		g_od_status = OD_STATUS_ONOFF;
	/*
	if (g_od_function_availible == 1)
		disp_od_core_enabled(cmdq, g_od_is_enabled);
	od_refresh_screen();
	//*/
#endif
}


/*
 * Must be called after the 3rd frame after disp_od_set_enabled(1) 
 * to enable OD function
 */
void disp_od_start_read(void *cmdq)
{
#if defined(CONFIG_MTK_OD_SUPPORT)
#if 0
    printk(KERN_DEBUG "[OD] disp_od_start_read()\n");

    if (g_od_is_enabled) {
        DISP_REG_MASK(cmdq, OD_REG00, 0, (1 << 31)); /* bypass_all = 0 */
        DISP_REG_MASK(cmdq, OD_REG39, 0, (1 << 29)); /* RDRAM_DIS = 0 */
    }
#endif	
#endif
}

static void disp_od_set_enabled_user(unsigned long user_arg, void *cmdq)
{
    int enabled = 0;

    if (copy_from_user((void*)&enabled, (void*)user_arg, sizeof(int))) {
        return -EFAULT;
    }

    disp_od_set_enabled(cmdq, enabled);
}


static int disp_od_ioctl_ctlcmd(DISP_MODULE_ENUM module, int msg, unsigned long arg, void *cmdq)
{
    DISP_OD_CMD cmd;

    if (copy_from_user((void*)&cmd, (void*)arg, sizeof(DISP_OD_CMD)))
    {
        return -EFAULT;
    }

    ODDBG(OD_DBG_ALWAYS, "OD ioctl cmdq %lx\n", (unsigned long)cmdq);

    switch (cmd.type) {
	/* on/off OD */
	case OD_CTL_ENABLE:
		{
			if (cmd.param0 == OD_CTL_ENABLE_OFF) {
				g_od_is_enabled_from_hwc=0;
				disp_od_set_enabled(cmdq, 0);
				ODDBG(OD_DBG_ALWAYS, "disp_od_set_disabled\n");
			} else if (cmd.param0 == OD_CTL_ENABLE_ON) {
				g_od_is_enabled_from_hwc=1;
				disp_od_set_enabled(cmdq, 1);
				ODDBG(OD_DBG_ALWAYS, "disp_od_set_enable\n");
			} else {
				ODDBG(OD_DBG_ALWAYS, "unknown enable type command\n");
			}
		}
	break;
    /* read reg */
    case OD_CTL_READ_REG:
        if (cmd.param0 < 0x1000) { // deny OOB access
            cmd.ret = DISP_REG_GET(cmd.param0 + OD_BASE);
        } else {
            cmd.ret = 0;
        }
    break;

    /* write reg */
    case OD_CTL_WRITE_REG:
        if (cmd.param0 < 0x1000) { // deny OOB access
            DISP_REG_SET(cmdq, cmd.param0 + OD_BASE, cmd.param1);
            cmd.ret = DISP_REG_GET(cmd.param0 + OD_BASE);
        } else {
            cmd.ret = 0;
        }
    break;

    /* enable split screen OD demo mode for miravision */
    case OD_CTL_ENABLE_DEMO_MODE:
    {
        switch (cmd.param0) {
        /* demo mode */
        case 0:
        case 1:
        {
            int enable = cmd.param0 ? 1 : 0;
            OD_REG_SET_FIELD(cmdq, OD_REG02, enable, DEMO_MODE);


            ODDBG(OD_DBG_ALWAYS, "OD demo %d\n", enable);
            /* save demo mode flag for suspend/resume */
            g_od_is_demo_mode = enable;
        }
        break;

        /* enable ink */
        case 2: /* off */
        case 3: /* on */
            OD_REG_SET_FIELD(cmdq, OD_REG03, (cmd.param0 - 2), ODT_INK_EN);
        break;

        /* eanble debug OSD */
        case 4: /* off */
        case 5: /* on */
            OD_REG_SET_FIELD(cmdq, OD_REG46, (cmd.param0 - 4), OD_OSD_SEL);
        break;


        default:
        break;
        }

    }
    break;

    /* write od table */
    case OD_CTL_WRITE_TABLE:
    {
#if 0
        unsigned char od_table[33 * 33] = {0};
        int od_table_select;

        if (cmd.param0 == 17 * 17) {
            od_table_select = OD_TABLE_17;
        } else if (cmd.param0 == 33 * 33) {
            od_table_select = OD_TABLE_33;
        } else { // deny OOB attack
            break;
        }

        if (copy_from_user((void*)od_table, (void*)cmd.param1, cmd.param0))
            break;

        ODDBG(OD_DBG_ALWAYS, "OD wtbl %d: %d %d %d\n", cmd.param0, od_table[0], od_table[1], od_table[2]);

        DISP_REG_SET(cmdq, DISP_OD_MISC, 1);
        _od_set_table(cmdq, od_table_select, od_table, 0);
        DISP_REG_SET(cmdq, DISP_OD_MISC, 0);
#else
        return -EFAULT;
#endif
    }
    break;

    default:
    break;
    }

    if (copy_to_user((void*)arg, (void*)&cmd, sizeof(DISP_OD_CMD)))
    {
        return -EFAULT;
    }

    return 0;
}


static int disp_od_ioctl(DISP_MODULE_ENUM module, int msg, unsigned long arg, void *cmdq)
{
    switch (msg) {
    case DISP_IOCTL_OD_SET_ENABLED:
        disp_od_set_enabled_user(arg, cmdq);
        break;
        
    case DISP_IOCTL_OD_CTL:
        return disp_od_ioctl_ctlcmd(module, msg, arg, cmdq);

    default:
        return -EFAULT;
    }

    return 0;
}

static void ddp_bypass_od(unsigned int width, unsigned int height, void * handle)
{
    printk(KERN_NOTICE "ddp_bypass_od\n");
    DISP_REG_SET(handle, DISP_REG_OD_SIZE, (width << 16) | height);
    /* do not use OD relay mode (dither will be bypassed) od_core_en = 0 */
    DISP_REG_SET(handle, DISP_REG_OD_CFG, 0);
    DISP_REG_SET(handle, DISP_REG_OD_EN, 0x1);
}

static int od_config_od(DISP_MODULE_ENUM module, disp_ddp_path_config* pConfig, void* cmdq)
{
    const LCM_PARAMS *lcm_param = &(pConfig->dispif_config);

    if (pConfig->dst_dirty) {
#if defined(CONFIG_MTK_OD_SUPPORT)
        M4U_PORT_STRUCT m4u_port;
        unsigned int od_table_size = lcm_param->od_table_size;
        void *od_table = lcm_param->od_table;

        m4u_port.ePortID = M4U_PORT_DISP_OD_R;
        m4u_port.Virtuality = 0;
        m4u_port.Security = 0;
        m4u_port.domain = 0;
        m4u_port.Distance = 1;
        m4u_port.Direction = 0;
        m4u_config_port(&m4u_port);
        m4u_port.ePortID = M4U_PORT_DISP_OD_W;
        m4u_config_port(&m4u_port);

        if (od_table != NULL) {
            printk(KERN_NOTICE "od_config_od: LCD OD table\n");
        }

    #if defined(OD_ALLOW_DEFAULT_TABLE)
        if (od_table == NULL) {
            od_table_size = 33 * 33;
            od_table = OD_Table_33x33;
            printk(KERN_NOTICE "od_config_od: Use default table\n");
        }
    #endif
    #if defined(OD_LINEAR_TABLE_IF_NONE)
        if (od_table == NULL) {
            od_table_size = 17 * 17;
            od_table = OD_Table_dummy_17x17;
            printk(KERN_NOTICE "od_config_od: Use linear table\n");
        }
    #endif
    
        if (od_table != NULL) {
            disp_config_od(pConfig->dst_w, pConfig->dst_h, cmdq, od_table_size, od_table);
    #if 0
            // For debug
            DISP_REG_MASK(cmdq, DISP_OD_INTEN, (1 << 6) | (1 << 3) | (1 << 2), (1 << 6) | (1 << 3) | (1 << 2));
    #endif
        } else {
            ddp_bypass_od(pConfig->dst_w, pConfig->dst_h, cmdq);
        }

#else /* Not support OD */
        ddp_bypass_od(pConfig->dst_w, pConfig->dst_h, cmdq);
#endif

    }

    if (pConfig->dst_dirty) {
        /* dither0 is in the OD module, and it uses OD clk, such that the OD clock must be on when screen is on. */
        disp_dither_init(DISP_DITHER0, pConfig->lcm_bpp, cmdq);
    }

#if defined(CONFIG_MTK_OD_SUPPORT)
    if (!pConfig->dst_dirty &&
        (lcm_param->type == LCM_TYPE_DSI) && (lcm_param->dsi.mode == CMD_MODE))
    {
        if (pConfig->ovl_dirty || pConfig->rdma_dirty)
            od_refresh_screen();
    }
#endif

    return 0;
}

static int od_clock_on(DISP_MODULE_ENUM module,void * handle)
{
    enable_clock(MT_CG_DISP0_DISP_OD, "od");
    DDPMSG("od_clock on CG 0x%x \n", DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
    return 0;
}

static int od_clock_off(DISP_MODULE_ENUM module,void * handle)
{
    disable_clock(MT_CG_DISP0_DISP_OD , "od");
    DDPMSG("od_clock off CG 0x%x \n",DISP_REG_GET(DISP_REG_CONFIG_MMSYS_CG_CON0));
    return 0;
}

// for SODI to check OD is enabled or not, this will be called when screen is on and disp clock is enabled
int disp_od_is_enabled(void)
{
    return (DISP_REG_GET(DISP_OD_CFG) & (1 << 1)) ? 1 : 0;
}

static int od_set_listener(DISP_MODULE_ENUM module, ddp_module_notify notify)
{
    g_od_ddp_notify = notify;
    return 0;
}


/* OD driver module */
DDP_MODULE_DRIVER ddp_driver_od =
{
    .init            = od_clock_on,
    .deinit          = od_clock_off,
    .config          = od_config_od,
    .start           = NULL,
    .trigger         = NULL,
    .stop            = NULL,
    .reset           = disp_od_core_reset,
    .power_on        = od_clock_on,
    .power_off       = od_clock_off,
    .is_idle         = NULL,
    .is_busy         = NULL,
    .dump_info       = NULL,
    .bypass          = NULL,
    .build_cmdq      = NULL,
    .set_lcm_utils   = NULL,
    .cmd             = disp_od_ioctl,
    .set_listener    = od_set_listener
};



// ----------------------------------------------------------------------
// Test code
// Following is only for OD functional test, not normal code
// Will not be linked into user build.
// ----------------------------------------------------------------------

#define OD_TLOG(fmt, arg...) printk(KERN_NOTICE "[OD] " fmt "\n", ##arg)

static int od_parse_triple(const char *cmd, unsigned long *offset, unsigned int *value, unsigned int *mask)
{
    int count = 0;
    char *next = (char*)cmd;

    *value = 0;
    *mask = 0;
    *offset = (unsigned long)simple_strtoul(next, &next, 0);

    if (*offset > 0x1000UL || (*offset & 0x3UL) != 0)  {
        *offset = 0UL;
        return 0;
    }

    count++;

    if (*next == ',')
        next++;

    *value = (unsigned int)simple_strtoul(next, &next, 0);
    count++;

    if (*next == ',')
        next++;

    *mask = (unsigned int)simple_strtoul(next, &next, 0);
    count++;

    return count;
}


static void od_dump_reg(const char *reg_list)
{
    unsigned long offset;
    unsigned int value;
    char *next = (char*)reg_list;

    OD_TLOG("OD reg base = %lx", (unsigned long)(OD_BASE));
    while (1) {
        offset = (unsigned long)simple_strtoul(next, &next, 0);
        if (offset < 0x1000UL && (offset & 0x3UL) == 0) {
            value = DISP_REG_GET(OD_BASE + offset);
            OD_TLOG("[+0x%03lx] = 0x%08x(%d)", offset, value, value);
        }

        if (next[0] != ',')
            break;

        next++;
    }
}


static void od_test_slow_mode(void)
{
#if 0
    msleep(30);

    // SLOW
    WDMASlowMode(DISP_MODULE_WDMA0, 1, 4, 0xff, 0x7, NULL); // time period between two write request is 0xff

    ODDBG(OD_DBG_ALWAYS, "OD SLOW\n");

    msleep(2000);

    ODDBG(OD_DBG_ALWAYS, "OD OK\n");
    WDMASlowMode(DISP_MODULE_WDMA0, 0, 0, 0, 0x7, NULL);
#endif
}


static void od_verify_boundary(void)
{
    int guard1, guard2;

    guard1 = (*((u32*)((unsigned long)g_od_buf_va + g_od_buf_size)) == OD_GUARD_PATTERN);
    guard2 = (*((u32*)((unsigned long)g_od_buf_va + g_od_buf_size + OD_ADDITIONAL_BUFFER)) == OD_GUARD_PATTERN);

    OD_TLOG("od_verify_boundary(): guard 1 = %d, guard 2 = %d", guard1, guard2);
}

static int od_parse3(const char *cmd, unsigned int *offset, unsigned int *value, unsigned int *mask){    
	int count = 0;    
	char *next = (char*)cmd;    
	*value = 0;    
	*mask = 0;    
	*offset = (unsigned int)simple_strtoul(next, &next, 0);    
	count++;    
	if (*next == ',')        
		next++;    
	*value = (unsigned int)simple_strtoul(next, &next, 0);    
	count++;    
	if (*next == ',')        
		next++;    
	*mask = (unsigned int)simple_strtoul(next, &next, 0);    
	count++;    return count;
}
#if 0
static int od_parse1(const char *cmd, unsigned int *offset){    
	int count = 0;    
	char *next = (char*)cmd;    
	*offset = (unsigned int)simple_strtoul(next, &next, 0);    
	count++;    return count;
}
#endif
static void od_set_value(const char *cmd){	
	unsigned int y, x, val;	
	int cnt = od_parse3(cmd, &y, &x, &val);	
	unsigned int offset = y*33+x;	
	if (val > 256)		
		val = 255;	
	OD_Table_33x33[offset] = (u8)val;	
	ODDBG(OD_DBG_ALWAYS, "y=%3d, x=%3d, val=%3d(%3d)\n", (int)y, (int)x, (int)OD_Table_33x33[offset], (int)offset);
}

static void od_table_recover(void){	
	int i=0; 	
	for(i=0; i<OD_TBL_M_SIZE; i+=1) {		
		OD_Table_33x33[i]=OD_Table_33x33_buf[i];	
	}
}

void od_test(const char *cmd, char *debug_output)
{
    unsigned long offset;
    unsigned int value, mask;
    //void *cmdq = NULL;
    int bbp = 0;

    OD_TLOG("od_test(%s)", cmd);

    debug_output[0] = '\0';

    DISP_CMDQ_BEGIN(cmdq, CMDQ_SCENARIO_DISP_CONFIG_OD)

    if (strncmp(cmd, "set:", 4) == 0) {
        int count = od_parse_triple(cmd + 4, &offset, &value, &mask);
        if (count == 3) {
            DISP_REG_MASK(cmdq, OD_BASE + offset, value, mask);
        } else if (count == 2) {
            DISP_REG_SET(cmdq, OD_BASE + offset, value);
            mask = 0xffffffff;
        }

        if (count >= 2) {
            OD_TLOG("[+0x%03lx] = 0x%08x(%d) & 0x%08x", offset, value, value, mask);
        }
    } else if (strncmp(cmd, "dump:", 5) == 0) {
        od_dump_reg(cmd + 5);
    } else if (strncmp(cmd, "stress", 6) == 0) {
        od_test_stress_table(NULL);
    } else if (strncmp(cmd, "slow_mode", 9) == 0) {
        od_test_slow_mode();
    } else if (strncmp(cmd, "boundary", 6) == 0) {
        od_verify_boundary();
    } else if (strncmp(cmd, "sodi:", 5) == 0) {
        int enabled = (cmd[5] == '1' ? 1 : 0);
        spm_enable_sodi(enabled);
    } else if (strncmp(cmd, "en:", 3) == 0) {
        int enabled = (cmd[3] == '1' ? 1 : 0);
        disp_od_set_enabled(cmdq, enabled);
		//disp_od_core_enabled(cmdq, enabled);
		//DISP_REG_MASK(cmdq, DISP_OD_CFG, 0, 0x1);// relay disable

	} else if (strncmp(cmd, "force:", 6) == 0) {
		if (cmd[6] == '0')
			g_od_is_enabled_force = 0;
		else if (cmd[6] == '1')
			g_od_is_enabled_force = 1;
		else
			g_od_is_enabled_force = 2;
    } else if (strncmp(cmd, "ink:", 4) == 0) {
        int enabled = (cmd[4] == '1' ? 1 : 0);
        if (enabled) {
            DISP_REG_MASK(cmdq, OD_REG03, (0x33 << 24) | (0xaa << 16) | (0x55 << 8) | 1, 0xffffff01);
        } else {
            DISP_REG_MASK(cmdq, OD_REG03, 0, 0x1);
        }
    } else if (strncmp(cmd, "demo:", 5) == 0) {
    	int enabled = (cmd[5] == '1' ? 1 : 0);
		OD_REG_SET_FIELD(cmdq, OD_REG02, enabled, DEMO_MODE);
        ODDBG(OD_DBG_ALWAYS, "OD demo %d\n", enabled);
        /* save demo mode flag for suspend/resume */
        g_od_is_demo_mode = enabled;
    } else if  (strncmp(cmd, "setod:", 6) == 0) {		
    	od_set_value(cmd+6);		
		ODDBG(OD_DBG_ALWAYS, "set OD value\n");
	} else if  (strncmp(cmd, "recover", 7) == 0) {		
		od_table_recover();
		ODDBG(OD_DBG_ALWAYS, "restore default value\n");	
	} else if  (strncmp(cmd, "reload", 6) == 0) {
		disp_od_set_enabled(cmdq, 0);
		DISP_REG_SET(cmdq, DISP_OD_MISC, 1);
		_od_set_table(cmdq, OD_TABLE_33, OD_Table_33x33, 0);
		DISP_REG_SET(cmdq, DISP_OD_MISC, 0);
		disp_od_set_enabled(cmdq, 1);
		ODDBG(OD_DBG_ALWAYS, "reload od table\n");	
	} else if (strncmp(cmd, "dither:", 7) == 0) {
		if(cmd[7]=='0') {
			disp_dither_init(DISP_DITHER0, 0, NULL);
			ODDBG(OD_DBG_ALWAYS, "bbp=0");
		} else if(cmd[7]=='1') {
			disp_dither_init(DISP_DITHER0, 16, NULL);
			ODDBG(OD_DBG_ALWAYS, "bbp=16");
		} else if(cmd[7]=='2') {
			disp_dither_init(DISP_DITHER0, 18, NULL);
			ODDBG(OD_DBG_ALWAYS, "bbp=18");
		} else if(cmd[7]=='3') {
			disp_dither_init(DISP_DITHER0, 24, NULL);
			ODDBG(OD_DBG_ALWAYS, "bbp=24");
		}
	} else if (strncmp(cmd, "test:", 5) == 0) {
		int enabled = (cmd[5] == '1' ? 1 : 0);
		if(enabled==1) {
			DISP_REG_SET(cmdq, DISP_OD_EN, 1);
		} else {
			DISP_REG_SET(cmdq, DISP_OD_EN, 0);
		}
		DISP_REG_MASK(cmdq, DISP_OD_CFG, 1, 0x1);// relay enable
	}
    DISP_CMDQ_CONFIG_STREAM_DIRTY(cmdq);
    DISP_CMDQ_END(cmdq)

    od_refresh_screen();

}
