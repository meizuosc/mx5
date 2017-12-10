/*
 * RTUP
 *     An implementation of the RealTime URL Report Processor
 *
 * Changelogs:
 *     v0.1:    20170706	Bringup
 *     v1.0:    20170906	Add Poll optimization
 *
 */

#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mm.h>
#include <linux/fcntl.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <linux/netdevice.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <net/sock.h>
#include <linux/random.h>
/******************************************************************************
 *
 *  1.macro
 *
 */
// version
#define RTUP_VERSION "1.0"

// device info
#define RTUP_DEV_NAME "rtup"
#define RTUP_CLASS_NAME "rtup"
#define RTUP_DEVICE_NAME "rtup"
#define AUTHED_R_THREAD "RTUP_Listener"
#define AUTHED_R_THREAD_2 "RTUP_FD"

// buf max limit
#define RTUP_MAX_RX_COUNT 1024
#define RTUP_INTF_KEY_SIZE 33
#define RTUP_WRITE_BUF_SIZE 1024
#define RTUP_READ_BUF_SIZE 4096
#define RTUP_NODE_SIZE_MAX 4096
#define RTUP_RBUF_ATR_SIZE (PAGE_SIZE/4)
#define RTUP_MIN_FORCE_WAKE_INTERVAL_MS 1000

#define RECORD_SIZE (sizeof(rtup_record))
#define DATA_ALIGN(X)   ALIGN(X, SMP_CACHE_BYTES)
#define AUTH_THREAD_SIZE (strlen(AUTHED_R_THREAD))
#define AUTH_THREAD_SIZE_2 (strlen(AUTHED_R_THREAD_2))

// list
#define CLIENT_LIST_HEAD (&block->client_list)
#define RECORD_LIST_HEAD (&block->record_list)

// spin_lock
#define CLIENT_LOCK() spin_lock(&block->client_lock)
#define CLIENT_UNLOCK() spin_unlock(&block->client_lock)

#define DATA_READY_LOCK() spin_lock(&block->data_ready_lock)
#define DATA_READY_UNLOCK() spin_unlock(&block->data_ready_lock)

#define RECORD_LOCK() spin_lock(&block->record_lock)
#define RECORD_UNLOCK() spin_unlock(&block->record_lock)

#define FORCE_WAKE_LOCK() spin_lock(&block->force_wake_lock)
#define FORCE_WAKE_UNLOCK() spin_unlock(&block->force_wake_lock)

/* parameter prefix used for android framework */
#define RTUP_PARAMS_OP_CODE	    "opcode:"
#define RTUP_PARAMS_VALUE_START ",value {"
#define RTUP_PARAMS_VALUE_END   '}'

#define RTUP_PARAMS_MIN_WAKE_INTERVAL	"wake.interval.min="
#define RTUP_PARAMS_RX_RING_MAX_COUNT	",wake.rxq.cnt.max="

#define RTUP_PARAMS_SHOW_VERSION 						"version"
#define RTUP_PARAMS_SHOW_DISABLE 						"disabled"
#define RTUP_PARAMS_SHOW_LOG_LEVEL 						"loglevel"
#define RTUP_PARAMS_SHOW_MIN_WAKE_INTERVAL 				"wake.interval.min"
#define RTUP_PARAMS_SHOW_RX_RING_MAX_COUNT 				"wake.rxq.cnt.max"

// utils
#define RET_INFO(var) var?"success":"fail"
#define CURRENT_TS (jiffies_to_msecs(jiffies))

#define KFREE(ptr) do {if(ptr) kfree(ptr);}while(0)

#define VALID_WAKE_INTERVAL(val) (val >= 1 && val <= 100)
#define VALID_WAKE_FRCOUNT(val) (val >= 1 && val <= 100)
#define VALID_OPCODE(val) (RTUP_PARAMS_SET_LOG_LEVEL <=val  && val < RTUP_PARAMS_OPS_INVALID)

/******************************************************************************
 *
 * 2.debug
 *
 */
#define D_LOG_TAG "RTUP "
#define CH_LINE "\n"
#define TAB_BLANK "\t"
#define FLINE "%-30s[%-4d]\t"
#define FU_LINE __FUNCTION__,__LINE__
#define LOG_PRINT(L,...)	\
do{\
	 if( block->log_level <= L ){\
		printk("[%s](%4d)",D_LOG_TAG,__LINE__);\
		printk(__VA_ARGS__);\
	}\
}while(0)

#define LOG_EXCE(...) 	LOG_PRINT(D_MSG_EXCE, __VA_ARGS__)
#define LOG_DUMP(...) 	LOG_PRINT(D_MSG_DUMP, __VA_ARGS__)
#define LOG_DEBUG(...) 	LOG_PRINT(D_MSG_DEBUG, __VA_ARGS__)
#define LOG_ERROR(...) 	LOG_PRINT(D_MSG_ERROR, "E " __VA_ARGS__)
#define LOG_ERRNO() 	LOG_PRINT(D_MSG_DEBUG,"E[%d]:%s",errno,strerror(errno))

/******************************************************************************
 *
 *  3.typedef
 *
 */
typedef enum {
	CLIENT_ENQUE,/* enqueue client */
	CLIENT_DEQUE,/* dequeue client */
	RECORD_ALLOC,/* alloc rtup_record */
	RECORD_LIST_CLEAR, /* clear block->record_list */
} OPS_TYPE;

typedef enum {
	D_MSG_DUMP = 0, /* dump */
	D_MSG_DEBUG, /* debug */
	D_MSG_ERROR /* error */
} LOG_LEVEL;

typedef enum {
	RET_SUCCESS = 0,/* */
	FAIL_ALLOC_MEM,/* */
	FAIL_INVALID_PTR,/* */
	FAIL_INVALID_SIZE,/* */
	FAIL_DRIVER_INIT, /* */
} RET_VALUE;

typedef enum {
	WAKE_NO_NEED = 0,/*  */
	WAKE_BUF_FULL,/*  */
	WAKE_MAX_INTERVAL,/*  */
	WAKE_MAX_URL_CNT,/* url count of rx ring */
	WAKE_MAX_KM_CNT, /* kmalloc count */
	WAKE_FEATURE_DISABLED, /*  */
} WAKE_TYPE;

/* used for framework */
typedef enum {
	RTUP_PARAMS_SET_LOG_LEVEL = 1, /*set log level*/
	RTUP_PARAMS_SET_WAKE_FACTOR,/* 2 */
	RTUP_PARAMS_SET_DISABLE, /* 3 */
	RTUP_PARAMS_OPS_INVALID, /* */
} RTUP_PARAMS_OPS_TYPE;

typedef struct _rtup_record {
	struct list_head list;
	int len;
	void* data;
} rtup_record;

typedef struct _rtup_block {
	int rx_count;
	int rx_bytes;
	int disabled;
	int init_done;
	int need_wake;
	u32 last_read_ts;
	u32 fore_wake_up_last_time;
	atomic_t client_count;

	u32 stat_all_rx;
	u32 stat_all_tx;
	u32 stat_all_lost;

	int dev_major;
	LOG_LEVEL log_level;
	int MIN_WAKE_INTERVAL;
	int RX_RING_MAX_COUNT;

	spinlock_t record_lock;
	spinlock_t force_wake_lock;
	spinlock_t data_ready_lock;
	struct class *dev_class;
	struct device *dev_device;
	struct list_head record_list;

	wait_queue_head_t rx_wait_queue;
	char read_buf[RTUP_READ_BUF_SIZE];
	u8 entry_rkey[RTUP_INTF_KEY_SIZE];
	u8 entry_ekey[RTUP_INTF_KEY_SIZE];
} rtup_block;

static rtup_block _block, *block = &_block;

/******************************************************************************
 *
 *  Memory,Client,Record OPS
 *
 */
static inline void rtup_dump_buf(char*name, char* buf, int len) {
	int index = 0;

	if (!name || !buf || !len)
		return;

	LOG_DEBUG("Buffer %s", name);
	if (block->log_level <= D_MSG_DEBUG) {
		for (index = 0; index < len; index++) {
			if ((index % 16) == 0)
				printk("\n[%s]", D_LOG_TAG);
			printk("%2x ", buf[index]);
		}
	}
	LOG_DEBUG("\n");
}

static int rtup_record_ops(OPS_TYPE type, void** ptr, int size) {
	int list_valid = 0;
	struct list_head* head = NULL;
	rtup_record* record = NULL, *tmp = NULL;
	switch (type) {
	case RECORD_ALLOC:
		if (!ptr)
			return -FAIL_INVALID_PTR;

		if (!size || size > RTUP_NODE_SIZE_MAX)
			return -FAIL_INVALID_SIZE;

		if ((*ptr = kzalloc(size + RECORD_SIZE, GFP_KERNEL)) == NULL)
			return -FAIL_INVALID_PTR;

		((rtup_record*) (*ptr))->len = size;
		((rtup_record*) (*ptr))->data = (char*) (*ptr) + RECORD_SIZE;
		break;

	case RECORD_LIST_CLEAR:
		RECORD_LOCK();
		head = &block->record_list;
		list_valid = !list_empty(head);
		list_for_each_entry_safe(record,tmp,head, list)
		{
			list_del(&record->list);

			block->rx_count--;
			block->rx_bytes -= record->len;
			if (block->rx_bytes < 0)
				block->rx_bytes = 0;

			if (block->rx_count < 0)
				block->rx_count = 0;

			kfree(record);
		}
		RECORD_UNLOCK();

		if (list_valid) {
			LOG_DEBUG("RECORD_LIST_CLEAR\n");
		}
		break;

	default:
		break;
	}

	return RET_SUCCESS;
}

static void rtup_client_ops(OPS_TYPE type) {
	switch (type) {
	case CLIENT_ENQUE:
		atomic_inc(&block->client_count);
		LOG_DEBUG("Open device\n");
		break;

	case CLIENT_DEQUE:
		atomic_dec(&block->client_count);
		LOG_DEBUG("Close device\n");
		break;

	default:
		break;
	}
}

static inline bool local_has_sleeper(wait_queue_head_t *wait) {
	//smp_mb();
	return wait && waitqueue_active(wait);
}

static inline void local_poll_wait(struct file *filp,
		wait_queue_head_t *wait_address, poll_table *p) {
	if (!poll_does_not_wait(p) && wait_address) {
		poll_wait(filp, wait_address, p);
		//smp_mb();
	}
}

static inline void local_def_readable(wait_queue_head_t *wait) {
	wake_up_interruptible_poll(wait, POLLIN | POLLRDNORM);
}

// Ready flag producer : app thread
static inline void local_data_ready_set(int wake) {
	if (wake) {
		DATA_READY_LOCK();
		block->need_wake = wake;
		DATA_READY_UNLOCK();
	}
}

// Ready flag consumer : rtup_listener
static inline int local_data_ready_then_reset(void) {
	int need_wake = 0;
	DATA_READY_LOCK();
	need_wake = block->need_wake;
	block->need_wake = 0;
	DATA_READY_UNLOCK();
	return need_wake;
}

static inline int try_to_wake_up_listener(WAKE_TYPE need_wake) {
	wait_queue_head_t *wait = &block->rx_wait_queue;
	if (need_wake && local_has_sleeper(wait)) {
		local_data_ready_set(need_wake);
		LOG_DEBUG("NW %d Rx[%d] (%d)\n", need_wake, block->rx_count,
				block->rx_bytes);
		local_def_readable(wait);
		return 1;
	} else {
		return 0;
	}
}

static inline u32 get_force_wake_time(void) {
	u32 ret = 0;
	FORCE_WAKE_LOCK();
	ret = block->fore_wake_up_last_time;
	FORCE_WAKE_UNLOCK();
	return ret;
}

static inline void set_force_wake_time(u32 now) {
	FORCE_WAKE_LOCK();
	block->stat_all_lost++;
	block->fore_wake_up_last_time = now;
	FORCE_WAKE_UNLOCK();
}

static inline int is_reader_valid(void) {
	struct task_struct *task = NULL;
	if ((task = get_current()) == NULL
			|| strncmp(task->comm, AUTHED_R_THREAD, AUTH_THREAD_SIZE)) {
		LOG_ERROR("Invalid reader\n");
		return 0;
	}
	return 1;
}

static inline int is_writer_valid(void) {
	struct task_struct *task = NULL;
	if ((task = get_current()) == NULL
			|| !strncmp(task->comm, AUTHED_R_THREAD, AUTH_THREAD_SIZE)
			|| !strncmp(task->comm, AUTHED_R_THREAD_2,
			AUTH_THREAD_SIZE_2)) {
		LOG_DEBUG("Invalid writer\n");
		return 0;
	}
	return 1;
}

/******************************************************************************
 *
 *  Device OPS
 *
 */
static ssize_t rtup_dev_write(struct file *file, const char __user *buf,
		size_t len, loff_t *ppos) {
	u32 now = 0;
	u32 last = 0;
	int data_len = 0;
	rtup_record* record = NULL;
	int need_wake = WAKE_NO_NEED;
	char cache[RTUP_WRITE_BUF_SIZE];

	if (!block->init_done)
		return len;

	if (block->disabled) {
		rtup_record_ops(RECORD_LIST_CLEAR, NULL, 0);
		try_to_wake_up_listener(WAKE_FEATURE_DISABLED);
		return len;
	}

	if (len <= RTUP_INTF_KEY_SIZE || len >= RTUP_WRITE_BUF_SIZE)
		return len;

	if (!is_writer_valid()) {
		return len;
	}

	if (copy_from_user(cache, buf, len))
		return len;

	if (strncmp(cache, block->entry_ekey, RTUP_INTF_KEY_SIZE)) {
		LOG_ERROR("Invalid header\n");
		return len;
	}

	if (block->rx_count > RTUP_MAX_RX_COUNT) {
		now = CURRENT_TS;
		last = get_force_wake_time();

		// Avoid wake up concurrently
		// When load userdebug mode,jiffies_to_msecs(jiffies) maybe be <= last with M1792.
		if (!last || now <= last) {
			set_force_wake_time(now);
		} else if (now - last >= RTUP_MIN_FORCE_WAKE_INTERVAL_MS) {
			set_force_wake_time(now);
			try_to_wake_up_listener(WAKE_MAX_KM_CNT);
		}

		return len;
	}

	data_len = len - RTUP_INTF_KEY_SIZE;
	if (rtup_record_ops(RECORD_ALLOC, (void**) &record, data_len))
		return len;

	if (record && record->data) {
		memcpy(record->data, cache + RTUP_INTF_KEY_SIZE, data_len);

		RECORD_LOCK();
		block->rx_count++;
		block->rx_bytes += record->len;
		list_add_tail(&record->list, &block->record_list);

		if (block->rx_bytes >= RTUP_READ_BUF_SIZE) {
			need_wake = WAKE_BUF_FULL;
		} else if (block->last_read_ts > 0
				&& (now = CURRENT_TS) > block->last_read_ts) {
			if (now - block->last_read_ts >= block->MIN_WAKE_INTERVAL) {
				need_wake = WAKE_MAX_INTERVAL;
			}
		} else if (block->rx_count >= block->RX_RING_MAX_COUNT) {
			need_wake = WAKE_MAX_URL_CNT;
		}
		block->stat_all_rx++;
		RECORD_UNLOCK();
	}

	try_to_wake_up_listener(need_wake);

	return len;
}

static ssize_t rtup_dev_read(struct file *file, char __user *user, size_t size,
		loff_t *ppos) {
	int read = 0;
	int step = 0;
	char* buf = block->read_buf;
	struct list_head* head = NULL;
	rtup_record* record = NULL, *tmp = NULL;

	if (!block->init_done)
		return 0;

	if (!is_reader_valid()) {
		return 0;
	}

	memset(buf, 0, RTUP_READ_BUF_SIZE);
	RECORD_LOCK();
	head = &block->record_list;
	list_for_each_entry_safe(record,tmp,head, list)
	{
		step = record->len;
		if (!record->data) {
			continue;
		}

		if ((read + step) >= RTUP_READ_BUF_SIZE) {
			break;
		}

		memcpy(buf, record->data, step);
		read += step;
		buf += step;
		list_del(&record->list);

		block->rx_count--;
		block->rx_bytes -= record->len;
		block->stat_all_tx++;

		if (block->rx_count < 0)
			block->rx_count = 0;
		if (block->rx_bytes < 0)
			block->rx_bytes = 0;

		KFREE(record);
	}

	block->last_read_ts = CURRENT_TS;
	RECORD_UNLOCK();

	if (!copy_to_user(user, block->read_buf, read)) {
		LOG_DEBUG("Tx[%d] (%d) Read: %d\n", block->stat_all_tx, block->rx_bytes,
				read);
		return read;
	}
	LOG_ERROR("Fail to ctu");
	return 0;
}

static unsigned int rtup_dev_poll(struct file *file, poll_table *wait) {
	unsigned int mask = 0;

	if (!is_reader_valid()) {
		return POLLHUP;
	}

	local_poll_wait(file, &block->rx_wait_queue, wait);
	if (block->disabled) {
		mask |= POLLHUP;
	} else if (local_data_ready_then_reset()) {
		mask |= POLLIN | POLLRDNORM;
	}

	return mask;
}

static int rtup_dev_close(struct inode *inode, struct file *file) {
	rtup_client_ops(CLIENT_DEQUE);
	return 0;
}

static int rtup_dev_open(struct inode * inode, struct file * filp) {
	rtup_client_ops(CLIENT_ENQUE);
	return 0;
}

static const struct file_operations rtup_dev_fops = { /*  */
.owner = THIS_MODULE, /*  */
.open = rtup_dev_open, /*  */
.release = rtup_dev_close, /*  */
.poll = rtup_dev_poll, /*  */
.write = rtup_dev_write, /*  */
.read = rtup_dev_read, /*  */
};

/******************************************************************************
 *
 *  Attr OPS
 *
 */
static bool rtup_params_invalid(const char *buf, size_t count) {
	return (!buf || !count || buf[count] != '\0' || strlen(buf) != count);
}

static inline int local_atoi(const char **s) {
	int i = 0;
	int valid = 0;
	while (isdigit(**s)) {
		valid = 1;
		i = i * 10 + *((*s)++) - '0';
	}

	return valid ? i : (-1);
}

static int rtup_get_value(const char* str, const char* child) {
	int val = -1;
	const char* pos = NULL;
	if ((pos = strstr(str, child))) {
		pos += strlen(child);
		val = local_atoi(&pos);
	}
	return val;
}

/*
 * fw=/sys/class/rtup/rtup/params;
 # 1.set loglevel
 echo "opcode:1,value {2}">$fw;
 echo "opcode:1,value {1}">$fw;

 # 2.set wakeup interval and rx count ths
 echo "opcode:2,value {wake.interval=5,first.max=20}" >$fw;

 # 3.
 echo "opcode:3,value {0}" >/sys/class/rtup/rtup/params;
 echo "opcode:3,value {1}" >/sys/class/rtup/rtup/params;
 */
static ssize_t rtup_params_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int len = 0;
	int val = -1;

	const char* pos = NULL;
	const char* pos1 = NULL;
	const char* pos2 = NULL;

	int opcode = RTUP_PARAMS_OPS_INVALID;
	if (rtup_params_invalid(buf, count)) {
		LOG_ERROR("Invalid params\n");
		return count;
	}

	LOG_DEBUG("Rcv: %s\n", buf);
	opcode = rtup_get_value(buf, RTUP_PARAMS_OP_CODE);
	if (!VALID_OPCODE(opcode)) {
		LOG_ERROR("Invalid op\n");
		return count;
	}

	if ((pos = strstr(buf, RTUP_PARAMS_VALUE_START))) {
		pos += strlen(RTUP_PARAMS_VALUE_START);
		pos1 = pos;
	} else {
		LOG_ERROR("Invalid start\n");
		return count;
	}

	if ((pos = strchr(buf, RTUP_PARAMS_VALUE_END))
			&& (pos - buf + 1 == strlen(buf) || pos - buf + 2 == strlen(buf))) {
		pos2 = pos;
	} else {
		LOG_ERROR("Invalid end\n");
		return count;
	}

	if (pos2 - pos1 < 1) {
		LOG_ERROR("Invalid value\n");
		return count;
	}

	len = pos2 - pos1;
	switch (opcode) {
	case RTUP_PARAMS_SET_LOG_LEVEL:
		if ((val = local_atoi(&pos1)) > 0) {
			block->log_level = val;
		}
		break;

	case RTUP_PARAMS_SET_DISABLE:
		if ((val = local_atoi(&pos1)) >= 0 && val <= 1) {
			block->disabled = val;
		}
		break;

	case RTUP_PARAMS_SET_WAKE_FACTOR:
		val = rtup_get_value(pos1, RTUP_PARAMS_MIN_WAKE_INTERVAL);
		if (VALID_WAKE_INTERVAL(val))
			block->MIN_WAKE_INTERVAL = val * 1000;

		val = rtup_get_value(pos1, RTUP_PARAMS_RX_RING_MAX_COUNT);
		if (VALID_WAKE_FRCOUNT(val))
			block->RX_RING_MAX_COUNT = val;

		break;

	default:
		return count;
	}

	return count;
}

static ssize_t rtup_stat_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int size = 0;

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %s\n",
		RTUP_PARAMS_SHOW_VERSION, RTUP_VERSION);

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %s\n",
		RTUP_PARAMS_SHOW_DISABLE, block->disabled ? "yes" : "no");

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		RTUP_PARAMS_SHOW_LOG_LEVEL, block->log_level);

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d s\n",
		RTUP_PARAMS_SHOW_MIN_WAKE_INTERVAL, block->MIN_WAKE_INTERVAL / 1000);

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		RTUP_PARAMS_SHOW_RX_RING_MAX_COUNT, block->RX_RING_MAX_COUNT);

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "Client.Count:%d\n", atomic_read(&block->client_count));

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "S.R[%u] T[%u] L[%u]\n", block->stat_all_rx,
				block->stat_all_tx, block->stat_all_lost);

	if (size < RTUP_RBUF_ATR_SIZE)
		size += snprintf(buf + size, RTUP_RBUF_ATR_SIZE - size,
		D_LOG_TAG "S.C[%d] B[%d] \n", block->rx_count, block->rx_bytes);

	return size;
}

static ssize_t rtup_entry_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	memcpy(buf, block->entry_rkey, RTUP_INTF_KEY_SIZE - 1);
	return RTUP_INTF_KEY_SIZE - 1;
}

static ssize_t rtup_hkey_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	return snprintf(buf, RTUP_RBUF_ATR_SIZE,
			"VbuNMxNyq0IoPj+qEJD4cuVfqKkCmKg\n");
}

static struct device_attribute rtup_dev_attrs[] = { /* attribute for user */
__ATTR(stat, 0440, rtup_stat_show, NULL), /* get stat */
__ATTR(entry, 0444, rtup_entry_show, NULL), /* get key */
__ATTR(params, 0220, NULL,rtup_params_store), /* set parameters */
__ATTR(key, 0400, rtup_hkey_show, NULL), /* get */
__ATTR_NULL, /* end */
};

static int rtup_attrs_create(struct device* dev) {
	int ret = 0;
	struct device_attribute* attr = NULL;
	for (attr = rtup_dev_attrs; attr->attr.name; attr++) {
		ret = device_create_file(dev, attr);
		if (ret) {
			return ret;
		}
	}
	return ret;
}

static void rtup_attrs_remove(struct device* dev) {
	struct device_attribute* attr = NULL;
	for (attr = rtup_dev_attrs; attr->attr.name; attr++) {
		device_remove_file(dev, attr);
	}
}

/******************************************************************************
 *
 *  Block and driver OPS
 *
 */
static void build_entry_key(rtup_block *block) {
	u8 tmp = 0;
	int i = 0, k = 0;
	int S = RTUP_INTF_KEY_SIZE - 1;

	u8* buf = block->entry_rkey;
	u8* ebuf = block->entry_ekey;

	get_random_bytes(block->entry_rkey, S);
	buf[S] = ':';
	ebuf[S] = ':';
	for (i = 0; i < S; i++) {
		if (!buf[i])
			buf[i] = 'a';
	}

	for (i = 0; i < S; i++) {
		k = (i + 5) % S;
		tmp = (u8) ((buf[i] | buf[k]) / 2);
		tmp &= buf[i];
		ebuf[i] = tmp;
	}
}

static int rtup_block_init(rtup_block *block) {
	memset(block, 0, sizeof(rtup_block));
	block->log_level = D_MSG_ERROR;
	block->RX_RING_MAX_COUNT = 30;
	block->MIN_WAKE_INTERVAL = 5 * 1000;
	atomic_set(&block->client_count, 0);
	INIT_LIST_HEAD(&block->record_list);
	spin_lock_init(&block->record_lock);
	spin_lock_init(&block->force_wake_lock);
	spin_lock_init(&block->data_ready_lock);
	init_waitqueue_head(&block->rx_wait_queue);
	build_entry_key(block);
	return 0;
}

static void rtup_block_deinit(rtup_block *block) {
	if (!block->init_done)
		return;

	rtup_record_ops(RECORD_LIST_CLEAR, NULL, 0);
}

static int __init rtup_core_init(void) {
	if(rtup_block_init(block)){
		LOG_ERROR("Fail to init block\n");
		return -FAIL_DRIVER_INIT;
	}

	if ((block->dev_major= register_chrdev(0, RTUP_DEV_NAME, &rtup_dev_fops)) < 0) {
		LOG_ERROR("Fail to register chrdev");
		goto fail1;
	}

	if (!(block->dev_class= class_create(THIS_MODULE, RTUP_DEVICE_NAME))) {
		LOG_ERROR("Fail to create class");
		goto fail2;
	}

	if (!(block->dev_device= device_create(block->dev_class, NULL, MKDEV(
		block->dev_major, 0), NULL,RTUP_DEVICE_NAME))) {
		LOG_ERROR("Fail to create device");
		goto fail3;
	}

	if (rtup_attrs_create(block->dev_device)) {
		LOG_ERROR("Fail to create attr");
		goto fail4;
	}

	LOG_DEBUG("Inited successfully.\n");
	block->init_done = 1;
	return RET_SUCCESS;

fail4:
	device_unregister(block->dev_device);

fail3:
	class_destroy(block->dev_class);

fail2:
	unregister_chrdev(block->dev_major, RTUP_DEV_NAME);

fail1:
	rtup_block_deinit(block);
	LOG_DEBUG("Inited fail !!\n");
	return -FAIL_DRIVER_INIT;
}

static void __exit rtup_core_exit(void) {
	rtup_attrs_remove(block->dev_device);
	device_unregister(block->dev_device);
	class_destroy(block->dev_class);
	unregister_chrdev(block->dev_major, RTUP_DEV_NAME);
	rtup_block_deinit(block);
	LOG_DEBUG("Deinited successfully.\n");
}

module_init(rtup_core_init);
module_exit(rtup_core_exit);
MODULE_ALIAS_NETPROTO(rtup);
MODULE_DESCRIPTION("Runtime URL Record Processor");
MODULE_LICENSE("GPL");
