#include <linux/init.h>
#include <linux/types.h>
#include <linux/ctype.h>
#include <linux/idr.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/random.h>
#include <linux/major.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <net/sock.h>
#include <net/tcp.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/icmp.h>
#include <linux/percpu.h>
#include <linux/inetdevice.h>
#include <net/net_namespace.h>
#include <linux/errno.h>
#include <linux/kthread.h>

#ifdef CONFIG_MEIZU_REALTIMEDIGAN
/******************************************************************************
 *
 * 1.1.common macros definition
 *
 */
/* switch */
#define CONFIG_TCP_STREAM_MONITOR 0

/* system */
#define C2S(x) case x: return #x;
#define D_MAX_BIT_VALUE(b) ( (1<<(b)) - 1U )
#define HLIST_EMPTY(h) (h ? hlist_empty(h): 1)

#define FIRST_APP_UID (10000)
#define LAST_APP_UID  (19999)
#define RE1_APP_UID (FIRST_APP_UID + UID_HASH_APP_END - UID_HASH_APP_START)

/* diagn max counter */
#define D_INTF_NAME_SIZE 64
#define D_CPU_COUNT nr_cpu_ids
#define D_RES_UID_CNT 10
#define D_RBUF_DEV_SIZE 512
#define D_RBUF_ATR_SIZE (PAGE_SIZE/4)
#define D_PKT_WAKEUP_THRESHOLD_MIN  1
#define D_PKT_WAKEUP_THRESHOLD_MAX  15

#define D_CACHE_NPKT_CNT 32
#define D_CACHE_USLOT_CNT 1024
#define D_CACHE_USLOT_CNT_HALF 512
#define D_CACHE_TCP_SRC_BLACK_LIST 12
#define D_CACHE_DNS_CNT 4

/* diagn hash */
#define D_UID_HASH_INVALID (-1)
#define D_RE1_HASH_SIZE (UID_HASH_APP_END - UID_HASH_APP_START + 1)

#define D_USLOT_X_CNT 512
#define D_USLOT_Y_CNT 20

#define D_S_HASH_SHIFT 4
#define D_S_HASH_MAX ( 1U<<D_S_HASH_SHIFT )
#define D_S_HASH_MASK ( D_S_HASH_MAX - 1U )

/* protocol IP */
#define D_IPV4_LEN 4
#define D_IPV4_STR_LEN 15
#define AF(hdr) (hdr->version==0x4? AF_INET: AF_INET6)
#define TRANSPORT_PTR(skb,i_hdr) (skb->data + (i_hdr->ihl << 2))

/* protocol udp constant values */
#define D_UDP_PORT_DNS 0x35
#define D_DNS_REPLY_MASK (0xf)
#define D_DNS_REPLY_SUCCESS (0x0)
#define D_DNS_REPLY_REFUSED (0x5)
#define D_DNS_REPLY_ANSWER_R_TYPE_HOST_ADDR (0x1)
#define D_DNS_REPLY_ANSWER_R_TYPE_CNAME (0x5)
#define D_DNS_REPLY_ANSWER_R_TYPE_HOST_ADDR_V6 (28)

/* udp hdr ops */
#define UDP_SP(uh) (be16_to_cpu(uh->source))
#define UDP_DP(uh) (be16_to_cpu(uh->dest))
#define UDP_LEN(uh) (be16_to_cpu(uh->len))

/* realtime_network_diagnosis : short with rtnetds */
#define DIAGN_DEV_NAME "rtnetds_dev"
#define DIAGN_CLASS_NAME "rtnetds_cls"
#define DIAGN_DEVICE_NAME "netdiagn"

/* common interface name */
#define D_INTF_NAME_WLAN "wlan0"
#define D_INTF_NAME_P2P "p2p0"
#define D_INTF_NAME_LP0 "l0"
#define D_INTF_NAME_MOB "ccmni0"

/* common operations for system */
#define CURRENT_MS (jiffies_to_msecs(jiffies))
#define GET_MS(j) (jiffies_to_msecs(j))

#define DEV_IS_WLAN0( dev ) (!strcmp(dev->name,D_INTF_NAME_WLAN))
#define DEV_IS_P2P0( dev ) (!strcmp(dev->name,D_INTF_NAME_P2P))
#define DEV_IS_LP0( dev ) (!strcmp(dev->name,D_INTF_NAME_LP0))
#define DEV_IS_MOBILE( dev ) (!strcmp(dev->name,dconfig->mobile_name))

#define D_SOCK_UID(sk) (SOCK_INODE(sk->sk_socket)->i_uid )

/* raw spin lock */
#define D_NPKT_SPIN_LOCK(block,flags) spin_lock_irqsave(&block->npkt_lock,flags)
#define D_NPKT_SPIN_UNLOCK(block,flags) spin_unlock_irqrestore(&block->npkt_lock,flags)

#define D_DRES_SPIN_LOCK() spin_lock(&dconfig->dres_lock)
#define D_DRES_SPIN_UNLOCK() spin_unlock(&dconfig->dres_lock)

#define D_STAT_SPIN_LOCK(flags) spin_lock_irqsave(&dconfig->stat_lock,flags)
#define D_STAT_SPIN_UNLOCK(flags) spin_unlock_irqrestore(&dconfig->stat_lock,flags)

#define D_USER_SPIN_LOCK() spin_lock(&dconfig->user_lock)
#define D_USER_SPIN_UNLOCK() spin_unlock(&dconfig->user_lock)

/* size of xx */
#define SIZE_IP_HDR (sizeof(struct iphdr))
#define SIZE_TCP_HDR (sizeof(struct tcphdr))
#define SIZE_UDP_HDR (sizeof(struct udphdr))
#define SIZE_ICMP_HDR (sizeof(struct icmphdr))
#define SIZE_DNS_HDR (12)
#define SIZE_DNS_ANSWER_R_HDR (12)

#define SIZE_IPV4_ADDR (sizeof(struct in_addr))
#define SIZE_IPV6_ADDR (sizeof(struct in6_addr))

#define SIZE_DPKT_ERR_ARRAY ( sizeof(int) * D_NOTIFY_CODE_MAX )

#define SIZE_D_STAT_PKT (sizeof(d_stat_pkt))

#define SIZE_IPV4_BE32 (sizeof(__be32))
#define SIZE_IN_IFADDR (sizeof(struct in_ifaddr))

/* parameter prefix used for android framework */
#define D_PARAMS_OP_CODE     "opcode:"
#define D_PARAMS_VALUE_START ",value {"
#define D_PARAMS_VALUE_END   '}'

#define D_PARAMS_UPDATE_INTERVAL_UNREACH	"ui.u="
#define D_PARAMS_UPDATE_INTERVAL_DNSNACK	",ui.d="
#define D_PARAMS_NOTIFY_THRESHOLD_UNREACH	",nt.u="
#define D_PARAMS_NOTIFY_THRESHOLD_DNSNACK	",nt.d="

#define D_PARAMS_PKT_WAKEUP_THRESHOLD_UNREACH	"pw.t.u="
#define D_PARAMS_PKT_WAKEUP_THRESHOLD_DNSNACK	",pw.t.d="
#define D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPSYNC	",pw.t.syn="
#define D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPDRST	",pw.t.drst="
#define D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPSRST	",pw.t.srst="

#define D_PARAMS_NOTIFY_OPCODE	"nopCode="
#define D_PARAMS_NOTIFY_IFDEV 		",ifdev="
#define D_PARAMS_NOTIFY_UID 		",uid="
#define D_PARAMS_NOTIFY_ECODE 	",errCode="

#define D_PARAMS_LOG_LEVEL   		"LogLevel="

#define D_PARAMS_SHOW_LOG_LEVEL 							"loglevel"
#define D_PARAMS_SHOW_INTF_NAME_WLAN 					"intf.name.wlan"
#define D_PARAMS_SHOW_INTF_NAME_MOBILE 				"intf.name.mobile"
#define D_PARAMS_SHOW_NEDV_IFADDR_INITED_WLAN 		"ndev.inited.wlan"
#define D_PARAMS_SHOW_NEDV_IFADDR_INITED_MOBILE 		"ndev.inited.mobile"
#define D_PARAMS_SHOW_GATE_IP_WLAN 						"gate.ip.wlan"
#define D_PARAMS_SHOW_GATE_IP_MOBILE 					"gate.ip.mobile"
#define D_PARAMS_SHOW_FRAMEWORK_DISABLED 			"switch.framework.disable_rtnds"
#define D_PARAMS_SHOW_FRAMEWORK_HTTPDIAGN 			"switch.framework.httpdiagn"
#define D_PARAMS_SHOW_MOBILE_GLOBAL_DISABLED 		"switch.mobile.global.disabled"
#define D_PARAMS_SHOW_UNREACH_SRC_INTERNET_ENABLED	"switch.unreach.srcinternet.enabled"
#define D_PARAMS_SHOW_FRAMEWORK_ENABLE_PPPOE		"switch.framework.pppoe.enabled"
#define D_PARAMS_SHOW_UPDATE_INTERVAL_UNREACH 		"update.interval.unreach"
#define D_PARAMS_SHOW_UPDATE_INTERVAL_DNSNACK 		"update.interval.dnsnack"
#define D_PARAMS_SHOW_NOTIFY_THRESHOLD_UNREACH 		"notify.threshold.unreach"
#define D_PARAMS_SHOW_NOTIFY_THRESHOLD_DNSNACK 		"notify.threshold.dnsnack"
#define D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_UNREACH	"pktwakeup.threshold.unreach"
#define D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_DNSNACK	"pktwakeup.threshold.dnsnack"
#define D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPSYNC	"pktwakeup.threshold.tcpsync"
#define D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPDRST	"pktwakeup.threshold.tcpdrst"
#define D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPSRST	"pktwakeup.threshold.tcpsrst"
#define D_PARAMS_SHOW_TCPSRC_BLIST_WLAN					"tcpsrc.blist.wlan:"
#define D_PARAMS_SHOW_TCPSRC_BLIST_MOBILE					"tcpsrc.blist.mobile:"
#define D_PARAMS_SHOW_DNS_LIST_WLAN							"dns.list.wlan:"
#define D_PARAMS_SHOW_DNS_LIST_MOBILE						"dns.list.mobile:"

/* DEFAULT DNS */
#define D_DEFAULT_DNS_F8 "8.8.8.8"
#define D_DEFAULT_DNS_F114 "114.114.114.114"

/* file nodes */
#define D_NODE_DEV "/dev/netdiagn"
#define D_NODE_DIR "/sys/class/rtnetds_cls/netdiagn"
#define D_NODE_LOG D_NODE_DIR "/log"
/******************************************************************************
 *
 *	1.1 common enumeration used for stack\diagn_thread\framework
 *
 */
typedef enum {
	D_MSG_EXCE = 0, /* exec */
	D_MSG_DUMP, /* dump */
	D_MSG_DEBUG, /* debug */
	D_MSG_ERROR /* error */
} LOG_LEVEL;

/* ipv4 string len */
typedef enum {
	IPV4_BIT_LEN_0 = 0,
	IPV4_BIT_LEN_1,
	IPV4_BIT_LEN_2,
	IPV4_BIT_LEN_3,
	IPV4_BIT_LEN_4,
} IPV4_BIT_LEN;

/* interval distribution for all uids of android */
typedef enum {
	UID_HASH_ROOT = 0,/* 0: 0 */
	UID_HASH_KERNEL,/*  1: (0,1000) */
	UID_HASH_SYSTEM,/*  2: [1000] */
	UID_HASH_ANDROID,/* 3: (1000,10000) */
	UID_HASH_APP_START,/*4: 10000 */
	UID_HASH_APP_END = 511, /* 10507 */
} D_UID_HASH;

/* operations for memory alloced by module_init */
typedef enum {
	D_MEM_OPS_RESET = 0,/* add available node to free list */
	D_MEM_OPS_GET_NODE,/* get node from free list */
	D_MEM_OPS_GET_DNS_NAK_NODE,/* dns nak */
	D_MEM_OPS_GET_TCP_SYN_NODE, /* get tcp syn ptr */
	D_MEM_OPS_GET_TCP_RST_NODE /* get tcp rst ptr */,
} D_MEM_OPS_TYPE;

/* ops type on the special user_slot->notify_map */
typedef enum {
	D_NOTIFY_MAP_SET_ONE_BIT = 0, /* set one bit of map */
	D_NOTIFY_MAP_SET_ALL_BITS,/* set all bits of map */
	D_NOTIFY_MAP_CLR_ONE_BIT, /* clear one bit of map*/
	D_NOTIFY_MAP_CLR_ALL_BITS, /* clear all bits of map */
	D_NOTIFY_MAP_CLR_GLOBAL_BITS,/* clear bits for global exception */
	D_NOTIFY_MAP_CLR_FIREWALL_BITS,/* clear bits for firewall exception */
	D_NOTIFY_MAP_QUERY_INDIVIDUAL,/* search by ifdev\uid\code */
	D_NOTIFY_MAP_QUERY_GLOBAL_ERR,/* find global error */
	D_NOTIFY_MAP_QUERY_LOCAL_FIREWALL_ERR, /* find local firewall exception */
} D_NOTIFY_MAP_OPS_TYPE;

/* used for framework */
typedef enum {
	D_PARAMS_SET_LOG_LEVEL = 1, /*set log level*/
	D_PARAMS_SET_WLAN_NAME,/* 2 */
	D_PARAMS_SET_MOBILE_NAME,/* 3 */
	D_PARAMS_SET_WLAN_GATE_IP,/* 4 */
	D_PARAMS_SET_MOBILE_GATE_IP,/* 5 */
	D_PARAMS_SET_STAT_THRESHOLD_INTERVAL,/* 6 */
	D_PARAMS_SET_PKT_WAKEUP_THRESHOLD,/* 7 */

	D_PARAMS_SET_WLAN_DNS = 30,
	D_PARAMS_SET_MOBILE_DNS,/* 31 */
	D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN,/* 32 */
	D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE,/* 33 */

	D_PARAMS_CLEAR_NOTIFYMAP = 40,
	D_PARAMS_CLEAR_ANY_SET,

	D_PARAMS_SET_FRAMEWORK_DISABLED = 50,
	D_PARAMS_SET_FRAMEWORK_HTTPDIAGN_ONLINE,/* 51 */
	D_PARAMS_SET_MOBILE_GLOBAL_DISABLED,/* 52 */
	D_PARAMS_SET_UNREACH_SRC_INTERNET_ENABLED,/* 53 */
	D_PARAMS_SET_FRAMEWORK_ENABLE_PPPOE,/* 54 */

	D_PARAMS_OPS_INVALID,
} D_PARAMS_OPS_TYPE;

/* switch bit mask,write by diagn_thread,read by stack threads */
typedef enum {
	D_SWITCH_BIT_DRIVER_INITED = 0,
	D_SWITCH_BIT_MEM_INITED,
	D_SWITCH_BIT_FRAMEWORK_DISABLED,
	D_SWITCH_BIT_FRAMEWORK_ENABLE_PPPOE,
	D_SWITCH_BIT_FRAMEWORK_HTTP_DIAGN,
	D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN,
	D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE,
	D_SWITCH_BIT_GATE_IP_INITED_WLAN,
	D_SWITCH_BIT_GATE_IP_INITED_MOBILE,
	D_SWITCH_BIT_DIAGN_THREAD_ONLINE,
	D_SWITCH_BIT_MOBILE_GLOBAL_DISABLED,
	D_SWITCH_BIT_UNREACH_SRC_INTERNET_ENABLED,
} D_SWITCH_BIT;

/******************************************************************************
 *
 *	1.2.enums for diagn core
 *
 */

/* interface type */
typedef enum {
	PKT_IFDEV_WLAN = 0,/* wlan0 */
	PKT_IFDEV_MOBILE,/* mobile */
	PKT_IFDEV_MAX, /* we just support these two interface */
	PKT_IFDEV_P2P0,/* p2p0 */
	PKT_IFDEV_LP0,/* l0 */
	PKT_IFDEV_OTHER /* unknown ifdev */,
} PKT_IFDEV_TYPE;

/* source of packet */
typedef enum {
	PKT_SRC_KERNEL,/* icmp sent by kernel */
	PKT_SRC_GATE_WAY,/* icmp sent by gate way */
	PKT_SRC_SUBNET, /* icmp sent by internet */
	PKT_SRC_INTERNET /* icmp sent by internet */,
} PKT_SRC_TYPE;

/* destination of packet */
typedef enum {
	PKT_OUT_INTERNET = 0, /* destination is internet */
	PKT_OUT_SUBNET, /* destination is local network group */
	PKT_OUT_GATEWAY, /* destination is gateway */
} PKT_OUT_TYPE;

/* category of protocol */
typedef enum {
	D_NET_PKT_RX_ICMP_UNREACH = 1,
	D_NET_PKT_TX_DNS_QUERY,
	D_NET_PKT_RX_DNS_RESPONSE,
	D_NET_PKT_TX_TCP,
	D_NET_PKT_RX_TCP,
} D_NET_PKT_TYPE;

/* internal error code for every exceptional packet */
typedef enum {
	D_ECODE_OK = 0,

	D_ECODE_RCU_BASE = 1,/* 1 */
	D_ECODE_RCU_DNS_QUERY = D_ECODE_RCU_BASE,/* 1 */
	D_ECODE_RCU_DNS_RESPONSE, /*  2 */
	D_ECODE_RCU_DNS_RESPONSE_C5_REFUSED, /*  3 */
	D_ECODE_RCU_DNS_RESPONSE_GATEWAY_IP, /* 4 */

	D_ECODE_RCU_TCP_BASE = 20,/* 20 */
	D_ECODE_RCU_TCP_RX_SYN_ACK = D_ECODE_RCU_TCP_BASE,/* 20 */
	D_ECODE_RCU_TCP_RX_RST,/* 21 */
	D_ECODE_RCU_TCP_RX_FIN,/* 22 */
	D_ECODE_RCU_TCP_TX_RST,/* 23 */
	D_ECODE_RCU_TCP_TX_FIN,/* 24 */

	D_ECODE_TIP_BASE = 30,/* 30 */
	D_ECODE_TIP_ICMP_REDIRECT = D_ECODE_TIP_BASE,/* 31 */
	D_ECODE_TIP_ICMP_SOURCE_QUENCH,/* 32 */
	D_ECODE_TIP_ICMP_FRAG_NEEDED,/* 33 */
	D_ECODE_TIP_NR_ICMP_UNREACH,/* 34 */

	D_ECODE_ERR_BASE = 40,/* 40 */
	D_ECODE_ERR_ICMP_PARAMETERPROB = D_ECODE_ERR_BASE,/* 40 */
	D_ECODE_ERR_ICMP_TIME_EXCEEDED,/* 41 */

	D_ECODE_ERR_NETUNREACH_BASE = 50,/* 50 */
	D_ECODE_ICMP_NET_UNREACH = D_ECODE_ERR_NETUNREACH_BASE,/* 50 */
	D_ECODE_ICMP_HOST_UNREACH,/* 51 */
	D_ECODE_ICMP_PROT_UNREACH,/* 52 */
	D_ECODE_ICMP_PORT_UNREACH,/* 53 */
	D_ECODE_ICMP_SR_FAILED,/* 54 */
	D_ECODE_ICMP_NET_UNKNOWN,/* 55 */
	D_ECODE_ICMP_HOST_UNKNOWN,/* 56 */
	D_ECODE_ICMP_HOST_ISOLATED,/* 57 */
	D_ECODE_ICMP_NET_ANO,/* 58 */
	D_ECODE_ICMP_HOST_ANO,/* 59 */
	D_ECODE_ICMP_NET_UNR_TOS,/* 60 */
	D_ECODE_ICMP_HOST_UNR_TOS,/* 61 */
	D_ECODE_ICMP_PKT_FILTERED,/* 62 */
	D_ECODE_ICMP_PREC_VIOLATION,/* 63 */
	D_ECODE_ICMP_PREC_CUTOFF,/* 64 */

	D_ECODE_TCP_WRN_BASE,/* 80 */
	D_ECODE_TCP_WRN_TCPMINTTLDROP = D_ECODE_TCP_WRN_BASE,/* 80 */
	D_ECODE_TCP_WRN_FP_OUTOFWINDOWICMPS, /*FP_OUTOFWINDOWICMPS*/
	D_ECODE_MAX, /* 82 */
} D_ECODE_TYPE;

/* the key of every notification to android */
typedef enum {
	D_NOTIFY_NET_ACCESS_OK = 0, /* 0 */
	D_NOTIFY_ERR_LOCAL_FAIL, /* 1 */
	D_NOTIFY_ERR_DNS_NOACK, /* 2 */
	D_NOTIFY_ERR_DNS_REFUSED, /* 3 */
	D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP, /* 4 */
	D_NOTIFY_ERR_INTERNET_OFFLINE, /* 5 */
	D_NOTIFY_ERR_TCP_WEB_RESET, /* 6 */
	D_NOTIFY_ERR_TCP_SELF_RESET, /* 7 */
	D_NOTIFY_ERR_IPTABLE_FORBIDDEN_TX, /* 8 */
	D_NOTIFY_ERR_IPTABLE_FORBIDDEN_RX, /* 9 */
	D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PORT, /* 10 */
	D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PKT, /* 11 */
	D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_TX,/* 12 */
	D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_RX,/* 13 */
	D_NOTIFY_ERR_INTERNET_ERROR_DNS,/* 14 */
	D_NOTIFY_ERR_INTERNET_ERROR_IPTABLE_FORBIDDEN,/* 15 */
	D_NOTIFY_ERR_INTERNET_ERROR_OTHER,/* 16 */
	D_NOTIFY_CODE_MAX, /* 17 */
} D_NOTIFY_CODE;

typedef enum {
	D_SUCCESS = 0,
	D_INVALID_PARAMS = 1,
	D_INVALID_IP_HDR,
	D_INVALID_DNS_RSP,
	D_INVALID_DNS_RSP_AQ_CNT,
	D_THREAD_START_FAIL,
	D_NOT_FIND_INET,
	D_NOT_FIND_USER,
	D_NON_INITED_WLAN,
	D_NON_INITED_MOBILE,
	D_OUT_RANGE_UID,
	D_OUT_RANGE_UID_X,
	D_OUT_RANGE_UID_Y,
	D_OUT_RANGE_IFDEV,
	D_OUT_RANGE_OPS_TYPE,
	D_OUT_RANGE_PKT_ECODE,
	D_PKT_NO_NEED_TO_STAT,
	D_TCP_SRC_IN_SUBNET,
	D_TCP_SRC_IN_BLACK_LIST,
	D_TCP_DST_IN_SUBNET,
	D_UNREACH_UNSPOORT_PROTO,
	D_UNREACH_SRC_SUBNET,
	D_UNREACH_SRC_INTERNET,
	D_UNREACH_UDP_OUT_SUBNET,
	D_UNREACH_UDP_OUT_GATE_NO_DNS,
	D_UNREACH_TCP_OUT_NOT_INTERNET,
	D_MOBILE_GLOBAL_DISABLED,
} D_OPS_RET;

/******************************************************************************
 *
 *	3.1. local type definition
 *
 */
typedef atomic_t D_MAP;

typedef struct _uid_hash {
	int x;/* hlost index */
	int y;/* conflict list index */
} uid_hash;

/* dns header for request and response */
typedef struct _dns_hdr {
	__be16 tid;
	__be16 flags;
	__be16 q_cnt;
	__be16 a_cnt;
	__be16 auth_cnt;
	__be16 add_cnt;
} dns_hdr;

typedef struct _dns_aw_hdr {
	__be16 name;
	__be16 type;
	__be16 class;
	__be16 ttl_l;
	__be16 ttl_h;
	__be16 dlen;
} dns_aw_hdr;

/* tcp socket key information */
typedef struct _d_stream {
	struct hlist_node hlist;
	void* sock;
	void* user;
	u32 d_hash;
	u32 pending_cnt;
	u32 first_sent_ms;
	u32 last_sent_ms;
	D_NET_PKT_TYPE c_type;
	u32 sk_shutdown :2, sk_no_check_tx :1, sk_no_check_rx :1, sk_userlocks :4,
			sk_protocol :8, sk_type :16;
} d_stream;

/* uid information */
typedef struct _d_user {
	u8 inited:1,exped:1;
	uid_t uid;
	u32 u_last_add;
	D_MAP notify_map;
	struct hlist_node hlist;
/*
 #if 0//CONFIG_TCP_STREAM_MONITOR
 u32 stream_cnt;
 struct hlist_head stream_head[D_S_HASH_MAX];
 #endif
 */
} d_user;

/* uid hash slot */
typedef struct _d_user_slot {
	//u32 u_cnt;
	d_user duser[D_USLOT_Y_CNT];
} d_user_slot;

/* packet from net stack,just used as local variable in entry function */
typedef struct _net_pkt {
	uid_t uid;
	int hw_index;
	struct sock* sk;
	PKT_SRC_TYPE src;
	PKT_OUT_TYPE out;
	D_NOTIFY_CODE err;
	D_ECODE_TYPE icode;
	PKT_IFDEV_TYPE ifdev;
	D_NET_PKT_TYPE c_type;
	struct iphdr s_hdr;
	struct iphdr i_hdr;
	union {
		struct tcphdr t_hdr;
		struct udphdr u_hdr;
	};
} net_pkt;

/* stack get d_stat_pkt from ring buf,then save the content of net_pkt to d_stat_pkt */
typedef struct _d_stat_pkt {
	uid_t uid;
	u32 add_ts;
	int hw_index;
	D_NOTIFY_CODE err;
	D_ECODE_TYPE icode;
	PKT_IFDEV_TYPE ifdev;
	D_NET_PKT_TYPE c_type;
	struct list_head flist;
	struct list_head wlist;
} d_stat_pkt;

/* diagn record sent to android framework */
typedef struct _d_res_info {
	int err_cnt;
	int uid_cnt;
	int hw_index;
	PKT_IFDEV_TYPE ifdev;
	D_NOTIFY_CODE err_code;
	uid_t uid[D_RES_UID_CNT];
} d_res_info;

/* mem show */
typedef struct _d_mem_cost {
	u32 skuser_cnt;
	u32 stream_cnt;
	u32 user_size;
	u32 stream_size;
	u32 cost;
} d_mem_cost;

/* synchronous flags used for rtnds_thread */
typedef struct _d_sync_params {
	int syn_cnt;
	int rst_cnt;
	int dns_cnt;
	PKT_IFDEV_TYPE ifdev;
	u16 dns_nak :1, tcp_syn :1, tcp_rst :1, exp_data :1;
} d_sync_params;

/* different processes will add d_stat_pkt to it`s owned queue */
typedef struct _d_write_block {
	int cpu;
	int count;
	int free_count;
	int used_count;
	d_stat_pkt s_pkt;
	d_stat_pkt r_pkt;
	d_stat_pkt n_pkt;
	d_stat_pkt* d_pkt;
	unsigned long sflags;
	spinlock_t npkt_lock;
	struct list_head free_list;
	struct list_head write_list;
}__attribute__((__aligned__(BITS_PER_LONG/8))) d_write_block;

/* static cache for supported net_device */
typedef struct _d_dev_info {
	int if_index;
	int ures_cnt;
	//int skuser_cnt;
	u32 d_last_req;
	D_MAP dns_req;
	D_MAP notify_map;
	__be32 gate_ipv4;
	D_MAP dpkt_enque_cnt;
	u32 dns_cnt;
	u32 exp_ucnt;
	u32 tcp_src_black_list_cnt;
	struct in_ifaddr ifadr;
	struct hlist_head user_head;
	__be32 dns[D_CACHE_DNS_CNT];
	d_res_info ures[D_NOTIFY_CODE_MAX];
	d_res_info dres[D_NOTIFY_CODE_MAX];
	__be32 tcp_src_black_list[D_CACHE_TCP_SRC_BLACK_LIST];
} d_dev_info;

/* static configuration for netdiagn */
typedef struct _d_config {
	int d_pkt_size;
	int s_pkt_size;
	int t_pkt_size;
	int t_pkt_ucnt;
	int user_size;
	int mob_ifn_size;
	D_MAP switch_map;
	int framework_wait;

	spinlock_t stat_lock;
	spinlock_t dres_lock;
	spinlock_t user_lock;// just used for pkt_thread and framework rtnds-thread

	d_stat_pkt* s_pkt;
	d_stat_pkt* r_pkt;
	d_stat_pkt* n_pkt;
	d_stat_pkt* d_pkt;
	d_stat_pkt* t_pkt;
	d_user_slot* user_slot;

	LOG_LEVEL log_level;
	int unreach_update_interval;
	int dnsnack_update_interval;
	int unreach_trigger_threshold;
	int dnsnack_trigger_threshold;

	int wake_threshold_unreach;
	int wake_threshold_dnsnack;
	int wake_threshold_tcpsync;
	int wake_threshold_tcpdrst;
	int wake_threshold_tcpsrst;

	unsigned long stat_flags;
	wait_queue_head_t stat_waitq;
	struct task_struct *stat_thread;
	d_dev_info dev_info[PKT_IFDEV_MAX];
	char mobile_name[D_INTF_NAME_SIZE];
} d_config;

/******************************************************************************
 *
 * 4.1.other common ops
 *
 */
#define DO_STREAM_HASH(key)	\
	( (u8)( (hash_long( (unsigned long)key,D_S_HASH_SHIFT)) & D_S_HASH_MASK ))

#define INIT_PKT_UID(pkt,sk)\
do{\
	pkt->sk = sk;\
	pkt->uid = __kuid_val(sock_i_uid(sk));\
}while(0)

#define DOPS_KFREE(ptr) \
do{\
	if(ptr) {\
		kfree(ptr);\
		ptr=NULL;\
	}\
}while(0)

#define DOPS_SET_SWITCH(str,val,bit)\
do{\
	val = s_atoi(&str);\
	if (DOPS_VALID_SWITCH_VAL(val)){\
		if (val)\
			dmap_set_bit(&dconfig->switch_map,bit);\
		else\
			dmap_clear_bit(&dconfig->switch_map,bit);\
	}\
}while(0)

#define DRES_VALID(res) (\
	((res->err_code == D_NOTIFY_ERR_DNS_NOACK && res->err_cnt >= dconfig->dnsnack_trigger_threshold)\
	|| (res->err_code != D_NOTIFY_ERR_DNS_NOACK && res->err_cnt >= dconfig->unreach_trigger_threshold)) &&\
	res->ifdev >= PKT_IFDEV_WLAN && res->ifdev < PKT_IFDEV_MAX && res->hw_index > 0 &&\
	res->err_code < D_NOTIFY_CODE_MAX && res->uid_cnt >0 && res->uid_cnt <= D_RES_UID_CNT)

#define IS_NOT_DNS(uh) (\
	!uh || (UDP_LEN(uh) <= SIZE_UDP_HDR)\
	|| ((UDP_SP(uh) != D_UDP_PORT_DNS) && (UDP_DP(uh) != D_UDP_PORT_DNS))\
	|| ((UDP_SP(uh) == D_UDP_PORT_DNS) && (UDP_DP(uh) == D_UDP_PORT_DNS)))

#define DOPS_VALID_STAT_THRESHOLD(val) (val >= D_PKT_WAKEUP_THRESHOLD_MIN && val <= D_PKT_WAKEUP_THRESHOLD_MAX)
#define DOPS_VALID_WAKE_THRESHOLD(val) (val >= D_PKT_WAKEUP_THRESHOLD_MIN && val <= D_PKT_WAKEUP_THRESHOLD_MAX)
#define DOPS_VALID_INTF_NAME_SIZE(val) (val > 0 && val < D_INTF_NAME_SIZE - 1)
#define DOPS_VALID_IPV4_S_LEN(val) (val > 0 && val <= D_IPV4_STR_LEN)
#define DOPS_VALID_SWITCH_VAL(val) (val >= 0 && val <= 1)
#define DOPS_VALID_NOTIFY_OP_CODE(val) (val >= D_NOTIFY_MAP_CLR_ONE_BIT && val<=D_NOTIFY_MAP_CLR_FIREWALL_BITS)
#define DOPS_VALID_NOTIFY_IFDEV(val) (val >= PKT_IFDEV_WLAN && val <= PKT_IFDEV_MOBILE)
#define DOPS_VALID_NOTIFY_ECODE(val) (val >= D_NOTIFY_NET_ACCESS_OK && val <= D_NOTIFY_CODE_MAX)
#define DOPS_VALID_PARAMS_OPCODE(val) (D_PARAMS_SET_LOG_LEVEL <=val  && val < D_PARAMS_OPS_INVALID)

/******************************************************************************
 *
 * 4.2.used for sync control of diagn_thread
 *
 */
#define D_THREAD_SYNC_SYN_IN_BIT 	(0)
#define D_THREAD_SYNC_RST_IN_BIT 	(1)
#define D_THREAD_SYNC_DNS_IN_BIT 	(2)
#define D_THREAD_SYNC_EXP_IN_BIT 	(3)
#define D_THREAD_SYNC_DEV_WLAN_BIT (4)

#define D_THREAD_SYNC_SYN_IN 	BIT(0)
#define D_THREAD_SYNC_RST_IN 	BIT(1)
#define D_THREAD_SYNC_DNS_IN 	BIT(2)
#define D_THREAD_SYNC_EXP_IN 	BIT(3)
#define D_THREAD_SYNC_DEV_WLAN BIT(4)

#define D_WAIT_Q_DATA_IN (D_THREAD_SYNC_SYN_IN | D_THREAD_SYNC_RST_IN |D_THREAD_SYNC_EXP_IN | D_THREAD_SYNC_DNS_IN )

/******************************************************************************
 *
 * 4.3.notify map
 * notify bit mask,write by dops_stat_thread,read by stack threads
 *
 */
#define GLOBAL_EXCEPTION_NOTIFY_MAP (\
	BIT(D_NOTIFY_ERR_DNS_NOACK) |\
	BIT(D_NOTIFY_ERR_DNS_REFUSED) |\
	BIT(D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP) |\
	BIT(D_NOTIFY_ERR_LOCAL_FAIL) |\
	BIT(D_NOTIFY_ERR_INTERNET_OFFLINE) |\
	BIT(D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_TX) |\
	BIT(D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_RX))

#define FIREWALL_EXCEPTION_NOTIFY_MAP (\
	BIT(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_TX) |\
	BIT(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_RX) |\
	BIT(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PORT) |\
	BIT(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PKT))

#define NOTIFY_MAP_VALUE(pmap) \
	((dmap_test_bit(pmap , D_NOTIFY_NET_ACCESS_OK))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_LOCAL_FAIL))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_DNS_NOACK))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_DNS_REFUSED))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_INTERNET_OFFLINE))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_TCP_WEB_RESET))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_TCP_SELF_RESET))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_IPTABLE_FORBIDDEN_TX))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_IPTABLE_FORBIDDEN_RX))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PORT))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PKT))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_TX))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_RX))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_INTERNET_ERROR_DNS))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_INTERNET_ERROR_IPTABLE_FORBIDDEN))),\
	((dmap_test_bit(pmap , D_NOTIFY_ERR_INTERNET_ERROR_OTHER)))

/******************************************************************************
 *
 * 4.4.convert original error code to D_ECODE_TYPE
 * notify bit mask,write by dops_stat_thread,read by stack threads
 *
 */
#define ICMP_DEST_UNREACH_CODE_2_RES(code,err)\
do{\
	switch(code){\
	case ICMP_NET_UNREACH:\
		err=D_ECODE_ICMP_NET_UNREACH;\
		break;\
	case ICMP_HOST_UNREACH:\
		err=D_ECODE_ICMP_HOST_UNREACH;\
		break;\
	case ICMP_PROT_UNREACH:\
		err=D_ECODE_ICMP_PROT_UNREACH;\
		break;\
	case ICMP_PORT_UNREACH:\
		err=D_ECODE_ICMP_PORT_UNREACH;\
		break;\
	case ICMP_FRAG_NEEDED:\
		err=D_ECODE_TIP_ICMP_FRAG_NEEDED;\
		break;\
	case ICMP_SR_FAILED:\
		err=D_ECODE_ICMP_SR_FAILED;\
		break;\
	case ICMP_NET_UNKNOWN:\
		err=D_ECODE_ICMP_NET_UNKNOWN;\
		break;\
	case ICMP_HOST_UNKNOWN:\
		err=D_ECODE_ICMP_HOST_UNKNOWN;\
		break;\
	case ICMP_HOST_ISOLATED:\
		err=D_ECODE_ICMP_HOST_ISOLATED;\
		break;\
	case ICMP_NET_ANO:\
		err=D_ECODE_ICMP_NET_ANO;\
		break;\
	case ICMP_HOST_ANO:\
		err=D_ECODE_ICMP_HOST_ANO;\
		break;\
	case ICMP_NET_UNR_TOS:\
		err=D_ECODE_ICMP_NET_UNR_TOS;\
		break;\
	case ICMP_HOST_UNR_TOS:\
		err=D_ECODE_ICMP_HOST_UNR_TOS;\
		break;\
	case ICMP_PKT_FILTERED:\
		err=D_ECODE_ICMP_PKT_FILTERED;\
		break;\
	case ICMP_PREC_VIOLATION:\
		err=D_ECODE_ICMP_PREC_VIOLATION;\
		break;\
	case ICMP_PREC_CUTOFF:\
		err=D_ECODE_ICMP_PREC_CUTOFF;\
		break;\
	default:\
		err=D_ECODE_TIP_NR_ICMP_UNREACH;\
		break;\
	}\
}while(0)

#define ICMP_CODE_2_ERR(type,code,err)\
do{\
	switch (type) {\
	case ICMP_DEST_UNREACH:\
		ICMP_DEST_UNREACH_CODE_2_RES(code,err);\
		break;\
	case ICMP_SOURCE_QUENCH:\
		err = D_ECODE_TIP_ICMP_SOURCE_QUENCH;\
		break;\
	case ICMP_REDIRECT:\
		err = D_ECODE_TIP_ICMP_REDIRECT;\
		break;\
	case ICMP_TIME_EXCEEDED:\
		err =  D_ECODE_ERR_ICMP_TIME_EXCEEDED;\
		break;\
	case ICMP_PARAMETERPROB:\
		err = D_ECODE_ERR_ICMP_PARAMETERPROB;\
		break;\
	default:\
		err = D_ECODE_OK;\
		break;\
	}\
}while(0)

/******************************************************************************
 *
 * 5.debug ops
 *
 */
#define D_LOG_TAG "RTNDS "
#define CH_LINE "\n"
#define TAB_BLANK "\t"
#define FLINE "%-30s[%-4d]\t"
#define FU_LINE __FUNCTION__,__LINE__
#define LOG_PRINT(L,...)	\
do{\
	if( dconfig->log_level <= L ){\
		printk("[%s](%4d)",D_LOG_TAG,__LINE__);\
		printk(__VA_ARGS__);\
	}\
}while(0)

#define LOG_NTAG(L,...)	\
do{\
	if( dconfig->log_level <= L ){\
		printk(__VA_ARGS__);\
	}\
}while(0)

#define LOG_EXCE(...) 	LOG_PRINT(D_MSG_EXCE, __VA_ARGS__)
#define LOG_DUMP(...) 	LOG_PRINT(D_MSG_DUMP, __VA_ARGS__)
#define LOG_DEBUG(...) 	LOG_PRINT(D_MSG_DEBUG, __VA_ARGS__)
#define LOG_ERROR(...) 	LOG_PRINT(D_MSG_ERROR, "Error " __VA_ARGS__)
#define LOG_ERRNO() 	LOG_PRINT(D_MSG_DEBUG,"E[%d]:%s",errno,strerror(errno))
#define LOGD_NTAG(...) 	LOG_NTAG(D_MSG_DEBUG,__VA_ARGS__)

#define IP_V4_FMT 		", %u.%u.%u.%u"
#define IP_V4_ARY(ip) 	((u8*)&ip)[0],((u8 *)&ip)[1],((u8 *)&ip)[2],((u8 *)&ip)[3]
#define IP_V4_PARY(ip) 	((u8*)ip)[0],((u8 *)ip)[1],((u8 *)ip)[2],((u8 *)ip)[3]

#define SFUNC_HERE() 		printk(D_LOG_TAG "%s[%d]\n",__FUNCTION__,__LINE__)
#define SFUNC_INT(var) 		printk(D_LOG_TAG "%s[%d]:%s=%d\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_LINT(var) 		printk(D_LOG_TAG "%s[%d]:%s=%ld\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_UINT(var) 		printk(D_LOG_TAG "%s[%d]:%s=%u\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_LUINT(var)		printk(D_LOG_TAG "%s[%d]:%s=%lu\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_LLUINT(var) 	printk(D_LOG_TAG "%s[%d]:%s=%llu\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_DEX(var) 		printk(D_LOG_TAG "%s[%d]:%s=0x%x\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_LLEX(var) 	printk(D_LOG_TAG "%s[%d]:%s=0x%llx\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_PTR(var) 		printk(D_LOG_TAG "%s[%d]:%s=0x%p\n",__FUNCTION__,__LINE__,#var,var)
#define SFUNC_INFO(var) 	printk(D_LOG_TAG "%s[%d]:%s\n",__FUNCTION__,__LINE__,var)
#define SFUNC_STR(var) 		printk(D_LOG_TAG "%s[%d]:%s=%s\n",__FUNCTION__,__LINE__,#var,var);
#define SFUNC_ENTER() 		printk(D_LOG_TAG "%s[%d]---------enter{\n",__FUNCTION__,__LINE__)
#define SFUNC_EXIT() 			printk(D_LOG_TAG "}%s[%d]---------exit\n\n",__FUNCTION__,__LINE__)
#define SFUNC_IP(ip) 			printk(D_LOG_TAG "%s[%d]:%s=[%d.%d.%d.%d]\n",__FUNCTION__,__LINE__,#ip,IP_V4_ARY(ip));
#define SFUNC_PIP(ip) 		printk(D_LOG_TAG "%s[%d]:%s=[%d.%d.%d.%d]\n",__FUNCTION__,__LINE__,#ip,IP_V4_PARY(ip));
#define SFUNC_MAC(a) 		if(a){printk(D_LOG_TAG "%s[%d]:mac="_MACSTR"\n",__FUNCTION__,__LINE__,_MAC2STR(a));}

#define IP_HDR_S_FMT \
	" I[tos:%u," "tot_len:%u," "id:%u," "frag:%u," "ttl:%u," \
	"tpro:%u" IP_V4_FMT IP_V4_FMT "]"

#define IP_HDR_U_FMT \
	" I[tos:%u," "l:%u," "t:%u," "p:%u" IP_V4_FMT IP_V4_FMT "]"

#define TCP_HDR_FMT \
	" T[" "sp:%u," "dp:%u," "seq:%u," "aseq:%u," "res:%u,"\
	"hdr_len:%u," "fin: %u," "syn: %u," "rst: %u," "psh: %u,"\
	"ack:%u," "ace:%u," "cwr:%u," "win:%u," "urg:%u" "]"

#define UDP_HDR_FMT \
	" U[" "sp:%u," "dp:%u," "data_len:%u" "]"

#define PKT_SF_FMT " [u:%d %s,%s,%s,%s,%s,%s]"

#define PKT_UF_FMT " [u:%d %u,%u,%u,%u,%u,%u]"

#define NOTIFY_MAP_FMT "MAP: %u%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u%u\n"

#define IP_HDR_S_VARGS(hdr)\
	hdr->tos,\
	be16_to_cpu(hdr->tot_len),\
	be16_to_cpu(hdr->id),\
	be16_to_cpu(hdr->frag_off),\
	hdr->ttl,hdr->protocol,\
	IP_V4_ARY(hdr->saddr),\
	IP_V4_ARY(hdr->daddr)

#define IP_HDR_U_VARGS(hdr)\
	hdr->tos,\
	be16_to_cpu(hdr->tot_len),\
	hdr->ttl,hdr->protocol,\
	IP_V4_ARY(hdr->saddr),\
	IP_V4_ARY(hdr->daddr)

#define TCP_HDR_VARGS(hdr)\
	be16_to_cpu(hdr->source),\
	be16_to_cpu(hdr->dest),\
	be32_to_cpu(hdr->seq),\
	be32_to_cpu(hdr->ack_seq),\
	hdr->res1,hdr->doff*4,\
	hdr->fin,hdr->syn,\
	hdr->rst,hdr->psh,\
	hdr->ack,hdr->ece,hdr->cwr,\
	be16_to_cpu(hdr->window),\
	be16_to_cpu(hdr->urg_ptr)

#define UDP_HDR_VARGS(hdr)\
	be16_to_cpu(hdr->source),\
	be16_to_cpu(hdr->dest),\
	be16_to_cpu(hdr->len)

#define PKT_SF_VARGS(pkt)\
	pkt->uid,\
	D_PKT_IFDEV_STR(pkt->ifdev),\
	D_PKT_SRC_STR(pkt->src),\
	D_PKT_OUT_TYPE_STR(pkt->out),\
	D_NET_PKT_TYPE_STR(pkt->c_type),\
	D_ECODE_STR(pkt->icode),\
	D_NOTIFY_CODE_STR(pkt->err)

#define PKT_UF_VARGS(pkt)\
	pkt->uid,\
	pkt->ifdev,\
	pkt->src,\
	pkt->out,\
	pkt->c_type,\
	pkt->icode,\
	pkt->err

#define DUMP_SEEK(pkt,buf,written,size,...)\
do{\
	if( size > written ){\
		written += snprintf(buf+written,(size-written),__VA_ARGS__);\
	}else{\
		return written;\
	}\
}while(0)

#define DUMP_PKT(pkt,buf,written,size)\
do{\
	if( pkt ){\
		DUMP_SEEK(pkt,buf,written,size,PKT_SF_FMT CH_LINE,PKT_SF_VARGS(pkt));\
		DUMP_SEEK(pkt,buf,written,size,IP_HDR_S_FMT CH_LINE,IP_HDR_S_VARGS((&pkt->s_hdr)));\
		DUMP_SEEK(pkt,buf,written,size,IP_HDR_S_FMT CH_LINE,IP_HDR_S_VARGS((&pkt->i_hdr)));\
		if( pkt->i_hdr.protocol == IPPROTO_TCP ){\
			DUMP_SEEK(pkt,buf,written,size,TCP_HDR_FMT CH_LINE,TCP_HDR_VARGS((&pkt->t_hdr)));\
		}else{\
			DUMP_SEEK(pkt,buf,written,size,UDP_HDR_FMT CH_LINE,UDP_HDR_VARGS((&pkt->u_hdr)));\
		}\
		DUMP_SEEK(pkt,buf,written,size,CH_LINE CH_LINE);\
	}\
}while(0)

#define PRINT_PKT(pkt)\
do{\
	if( pkt && dconfig->log_level <= D_MSG_DEBUG ){\
		LOG_DEBUG(PKT_UF_FMT CH_LINE,PKT_UF_VARGS(pkt));\
		if(dconfig->log_level <= D_MSG_DUMP)\
		LOG_DEBUG(PKT_SF_FMT CH_LINE,PKT_SF_VARGS(pkt));\
		LOG_DEBUG(IP_HDR_S_FMT CH_LINE,IP_HDR_S_VARGS((&pkt->s_hdr)));\
		switch(pkt->c_type){\
		case D_NET_PKT_RX_ICMP_UNREACH:\
			LOG_DEBUG(IP_HDR_S_FMT CH_LINE,IP_HDR_S_VARGS((&pkt->i_hdr)));\
			if( pkt->i_hdr.protocol == IPPROTO_TCP ){\
				LOG_DEBUG(TCP_HDR_FMT CH_LINE,TCP_HDR_VARGS((&pkt->t_hdr)));\
			}else{\
				LOG_DEBUG(UDP_HDR_FMT CH_LINE,UDP_HDR_VARGS((&pkt->u_hdr)));\
			}\
			break;\
		case D_NET_PKT_TX_DNS_QUERY:\
		case D_NET_PKT_RX_DNS_RESPONSE:\
			LOG_DEBUG(UDP_HDR_FMT CH_LINE,UDP_HDR_VARGS((&pkt->u_hdr)));\
			break;\
		case D_NET_PKT_TX_TCP:\
		case D_NET_PKT_RX_TCP:\
			LOG_DEBUG(TCP_HDR_FMT CH_LINE,TCP_HDR_VARGS((&pkt->t_hdr)));\
			break;\
		default:\
			break;\
		}\
		LOG_DEBUG("\n");\
	}\
}while(0)

#define PRINT_DRES(dres,index,uid_index)\
do{\
	LOG_DEBUG("DRES [%d] {IF[%d] %d %d %d :", index,dres->hw_index, dres->err_code,dres->err_cnt, dres->uid_cnt);\
	for (uid_index = 0; uid_index < dres->uid_cnt && uid_index < D_RES_UID_CNT; uid_index++)\
 		LOGD_NTAG("%d,", dres->uid[uid_index]);\
 	LOGD_NTAG("}\n");\
}while(0)

#define PRINT_NOTIFY_MAP(map,ifdev,uid) \
do {\
	if(dmap_test(map)){\
		LOG_DEBUG("[%u] %-6u:" NOTIFY_MAP_FMT, ifdev, uid,NOTIFY_MAP_VALUE((map)));\
	}\
}while(0)

#define PRINT_USER(pos,ifdev) \
do {\
	if(dmap_test(&pos->notify_map)){\
		LOG_DEBUG("[%u] %-6u: as:%u " NOTIFY_MAP_FMT, ifdev, pos->uid,pos->u_last_add,NOTIFY_MAP_VALUE((&pos->notify_map)));\
	}\
}while(0)

/******************************************************************************
 *
 * 6.static definition
 *
 */

static u8 LP_0_ADDR[D_IPV4_LEN] = { 0x7f, 0x0, 0x0, 0x1 };
static int major;
static struct class *diagn_class;
static struct device *diagn_device;
static d_config _dconfig, *dconfig = &_dconfig;
static d_write_block __percpu *dwrite_block;
static DECLARE_WAIT_QUEUE_HEAD( diagn_waitq);

static void dops_npkt_enqueue(net_pkt* pkt);
static bool dops_need_do_stat(PKT_IFDEV_TYPE ifdev, uid_t uid,
		D_NOTIFY_CODE code);

/******************************************************************************
 *
 * 7.Macro to const string ops
 *
 */

const char* D_PKT_IFDEV_STR(int type) {
	switch (type) {
	C2S(PKT_IFDEV_LP0)
	C2S(PKT_IFDEV_WLAN)
	C2S(PKT_IFDEV_P2P0)
	C2S(PKT_IFDEV_MOBILE)
	C2S(PKT_IFDEV_OTHER)
	default:
		return "";
	}
}

const char* D_PKT_SRC_STR(int type) {
	switch (type) {
	C2S(PKT_SRC_KERNEL)
	C2S(PKT_SRC_GATE_WAY)
	C2S(PKT_SRC_INTERNET)
	C2S(PKT_SRC_SUBNET)
	default:
		return "";
	}
}

const char* D_PKT_OUT_TYPE_STR(int type) {
	switch (type) {
	C2S(PKT_OUT_INTERNET)
	C2S(PKT_OUT_SUBNET)
	C2S(PKT_OUT_GATEWAY)
	default:
		return "";
	}
}

const char* D_NET_PKT_TYPE_STR(int type) {
	switch (type) {
	C2S(D_NET_PKT_RX_ICMP_UNREACH)
	C2S(D_NET_PKT_TX_DNS_QUERY)
	C2S(D_NET_PKT_RX_DNS_RESPONSE)
	C2S(D_NET_PKT_TX_TCP)
	C2S(D_NET_PKT_RX_TCP)
	default:
		return "";
	}
}

const char* D_NOTIFY_CODE_STR(int type) {
	switch (type) {
	C2S(D_NOTIFY_NET_ACCESS_OK)
	C2S(D_NOTIFY_ERR_DNS_NOACK)
	C2S(D_NOTIFY_ERR_DNS_REFUSED)
	C2S(D_NOTIFY_ERR_TCP_WEB_RESET)
	C2S(D_NOTIFY_ERR_TCP_SELF_RESET)
	C2S(D_NOTIFY_ERR_LOCAL_FAIL)
	C2S(D_NOTIFY_ERR_INTERNET_OFFLINE)
	C2S(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_TX)
	C2S(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_RX)
	C2S(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PORT)
	C2S(D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PKT)
	C2S(D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_TX)
	C2S(D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_RX)

	default:
		return "";
	}
}

const char* D_ECODE_STR(int type) {
	switch (type) {
	C2S(D_ECODE_OK)
	C2S(D_ECODE_RCU_DNS_QUERY)
	C2S(D_ECODE_RCU_DNS_RESPONSE)
	C2S(D_ECODE_RCU_DNS_RESPONSE_C5_REFUSED)
	C2S(D_ECODE_RCU_DNS_RESPONSE_GATEWAY_IP)
	C2S(D_ECODE_RCU_TCP_RX_SYN_ACK)
	C2S(D_ECODE_RCU_TCP_RX_RST)
	C2S(D_ECODE_RCU_TCP_RX_FIN)
	C2S(D_ECODE_RCU_TCP_TX_RST)
	C2S(D_ECODE_RCU_TCP_TX_FIN)
	C2S(D_ECODE_TIP_ICMP_REDIRECT)
	C2S(D_ECODE_TIP_ICMP_SOURCE_QUENCH)
	C2S(D_ECODE_TIP_ICMP_FRAG_NEEDED)
	C2S(D_ECODE_TIP_NR_ICMP_UNREACH)
	C2S(D_ECODE_ERR_ICMP_PARAMETERPROB)
	C2S(D_ECODE_ERR_ICMP_TIME_EXCEEDED)
	C2S(D_ECODE_ICMP_NET_UNREACH)
	C2S(D_ECODE_ICMP_HOST_UNREACH)
	C2S(D_ECODE_ICMP_PROT_UNREACH)
	C2S(D_ECODE_ICMP_PORT_UNREACH)
	C2S(D_ECODE_ICMP_SR_FAILED)
	C2S(D_ECODE_ICMP_NET_UNKNOWN)
	C2S(D_ECODE_ICMP_HOST_UNKNOWN)
	C2S(D_ECODE_ICMP_HOST_ISOLATED)
	C2S(D_ECODE_ICMP_NET_ANO)
	C2S(D_ECODE_ICMP_HOST_ANO)
	C2S(D_ECODE_ICMP_NET_UNR_TOS)
	C2S(D_ECODE_ICMP_HOST_UNR_TOS)
	C2S(D_ECODE_ICMP_PKT_FILTERED)
	C2S(D_ECODE_ICMP_PREC_VIOLATION)
	C2S(D_ECODE_ICMP_PREC_CUTOFF)
	C2S(D_ECODE_TCP_WRN_TCPMINTTLDROP)
	C2S(D_ECODE_TCP_WRN_FP_OUTOFWINDOWICMPS)

	default:
		return "";
	}
}

const char* D_OPS_RET_STR(int type) {
	switch (type) {
	C2S(D_SUCCESS)
	C2S(D_INVALID_PARAMS)
	C2S(D_INVALID_IP_HDR)
	C2S(D_INVALID_DNS_RSP)
	C2S(D_INVALID_DNS_RSP_AQ_CNT)
	C2S(D_THREAD_START_FAIL)
	C2S(D_NOT_FIND_INET)
	C2S(D_NOT_FIND_USER)
	C2S(D_NON_INITED_WLAN)
	C2S(D_NON_INITED_MOBILE)
	C2S(D_OUT_RANGE_UID)
	C2S(D_OUT_RANGE_UID_X)
	C2S(D_OUT_RANGE_UID_Y)
	C2S(D_OUT_RANGE_IFDEV)
	C2S(D_OUT_RANGE_OPS_TYPE)
	C2S(D_OUT_RANGE_PKT_ECODE)
	C2S(D_PKT_NO_NEED_TO_STAT)
	C2S(D_TCP_SRC_IN_SUBNET)
	C2S(D_TCP_SRC_IN_BLACK_LIST)
	C2S(D_TCP_DST_IN_SUBNET)
	C2S(D_UNREACH_UNSPOORT_PROTO)
	C2S(D_UNREACH_SRC_SUBNET)
	C2S(D_UNREACH_SRC_INTERNET)
	C2S(D_UNREACH_UDP_OUT_SUBNET)
	C2S(D_UNREACH_UDP_OUT_GATE_NO_DNS)
	C2S(D_UNREACH_TCP_OUT_NOT_INTERNET)
	C2S(D_MOBILE_GLOBAL_DISABLED)
	default:
		return "";
	}
}

/******************************************************************************
 *
 * 8.common ops
 *
 */
#define dmap_get(v) atomic_read(v)
#define dmap_reset(v) atomic_set(v, 0)
#define dmap_test(v) ((atomic_read(v)) > 0 ? 1 : 0)
#define dmap_test_bit(v,bit) (((atomic_read(v)) & BIT(bit)) > 0 ? 1 : 0)
#define dmap_test_mask(v,mask) (((atomic_read(v)) & mask) > 0 ? 1 : 0)
#define dmap_set_bit(v,bit) do {if(!dmap_test_bit(v, bit)) atomic_add(BIT(bit), v);}while(0)
#define dmap_clear_bit(v,bit) do {if(dmap_test_bit(v, bit)) atomic_sub(BIT(bit), v);}while(0)
#define dmap_set_all_bits(v) atomic_set(v, D_MAX_BIT_VALUE(31))
#define dmap_inc(v) atomic_inc(v)
#define dmap_dec(v) atomic_dec(v)
inline void dmap_set_mask(D_MAP* v, u32 mask) {
	u8 bit = 0;

	while (mask && bit <= D_MAX_BIT_VALUE(5)) {
		if (mask & 0x1) {
			dmap_set_bit(v, bit);
		}
		bit++;
		mask >>= 1;
	}
}

static inline void dmap_clear_mask(D_MAP* v, u32 mask) {
	u8 bit = 0;

	while (mask && bit <= D_MAX_BIT_VALUE(4)) {
		if (mask & 0x1) {
			dmap_clear_bit(v, bit);
		}
		bit++;
		mask >>= 1;
	}
}

static inline bool is_in_set(u32*a, int len, u32 b) {
	int i = 0;
	for (i = 0; i < len; i++) {
		if (a[i] == b)
			return true;
	}
	return false;
}

static inline bool rm_from_set(int*a, int len, int b) {
	int i = 0, j = -1;
	for (i = 0; i < len; i++) {
		if (a[i] == b) {
			j = i;
			break;
		}
	}

	if (j < 0)
		return false;

	for (i = j; i < len - 1; i++) {
		a[i] = a[i + 1];
	}
	return true;
}

static inline int s_atoi(const char **s) {
	int i = 0;
	int valid = 0;
	while (isdigit(**s)) {
		valid = 1;
		i = i * 10 + *((*s)++) - '0';
	}

	return valid ? i : (-1);
}

static inline bool is_addr_equal(void *a, void *b, int af) {
	int rc = -1;
	switch (af) {
	case AF_INET:
		rc = memcmp(a, b, SIZE_IPV4_ADDR);
		break;
	case AF_INET6:
		rc = memcmp(a, b, SIZE_IPV6_ADDR);
		break;

	default:
		break;
	}
	return rc ? false : true;
}

static inline bool is_iphdr_valid(struct iphdr* ip) {
	return !ip ? false : (ip->tot_len >= 20 && ip->version == 4);
}

static inline bool ipv4_strto_addr(const char *ip, __be32* s_addr) {
	char c;
	u32 val;
	int base, n;
	u32 parts[4];
	u32 *pp = parts;

	c = *ip;
	memset(parts,0,sizeof(parts));

	for (;;) {
		if (!isdigit(c)) {
			return false;
		}

		val = 0;
		base = 10;
		if (c == '0') {
			c = *++ip;
			if (c == 'x' || c == 'X') {
				base = 16;
				c = *++ip;
			} else {
				base = 8;
			}
		}

		for (;;) {
			if (isascii(c) && isdigit(c)) {
				val = (val * base) + (c - '0');
				c = *++ip;
			} else if (base == 16 && isascii(c) && isxdigit(c)) {
				val = (val << 4) |(c + 10 - (islower(c) ? 'a' : 'A'));
				c = *++ip;
			} else {
				break;
			}
		}

		if (c == '.') {
			if (pp >= parts + 3) {
				return false;
			}
			*pp++ = val;
			c = *++ip;
		} else {
			break;
		}
	}

	if (c != '\0' && (!isascii(c)|| !isspace(c))) {
		return false;
	}

	n = pp - parts + 1;
	/* initial nondigit */
	/* a -- 32 bits */
	/* a.b -- 8.24 bits */
	/* a.b.c -- 8.8.16 bits */
	/* a.b.c.d -- 8.8.8.8 bits */
	switch (n) {
		case IPV4_BIT_LEN_0:
		return false;

		case IPV4_BIT_LEN_1:
		break;

		case IPV4_BIT_LEN_2:
		if ((val > 0xffffff) || (parts[0] > 0xff)) {
			return false;
		}
		val |= parts[0] << 24;
		break;

		case IPV4_BIT_LEN_3:
		if ((val > 0xffff) || (parts[0] > 0xff) || (parts[1] > 0xff)) {
			return false;
		}
		val |= (parts[0] << 24) | (parts[1] << 16);
		break;

		case IPV4_BIT_LEN_4:
		if ((val > 0xff) || (parts[0] > 0xff) || (parts[1] > 0xff) || (parts[2] > 0xff)) {
			return false;
		}
		val |= (parts[0] << 24) | (parts[1] << 16) | (parts[2] << 8);
		break;
	}

	*s_addr = cpu_to_be32(val);
	return true;
}

static int get_ndev_ifaddr(struct net_device *any_dev, char* ifname,
		struct in_ifaddr* if_out, int* hw_index) {
	int ret = D_SUCCESS;
	struct net *inet = NULL;
	struct in_device *in_dev = NULL;
	struct in_ifaddr **ifap = NULL;
	struct in_ifaddr *ifa = NULL;
	struct net_device *dev;

	if (!any_dev || !ifname || !if_out || !hw_index)
		return D_INVALID_PARAMS;

	if (!strcmp(any_dev->name, ifname)) {
		dev = any_dev;
	} else {
		if ((inet = dev_net(any_dev)) == NULL) {
			return D_NOT_FIND_INET;
		}

		//rtnl_lock();
		ret = -ENODEV;
		dev = __dev_get_by_name(inet, ifname);
	}

	if (!dev)
		goto out;

	memset(if_out, 0, SIZE_IN_IFADDR);
	in_dev = __in_dev_get_rtnl(dev);
	if (in_dev) {
		if (!ifa) {
			for (ifap = &in_dev->ifa_list; (ifa = *ifap) != NULL;
					ifap = &ifa->ifa_next) {
				if (!strcmp(ifname, ifa->ifa_label)) {
					memcpy(if_out, ifa, SIZE_IN_IFADDR);
					*hw_index = dev->ifindex;
					ret = D_SUCCESS;
					break;
				}
			}
		}
	}
	out: //rtnl_unlock();
	return ret;
}

static int set_ndev_ifaddr(PKT_IFDEV_TYPE type) {
	int ret = D_SUCCESS;
	struct net_device *if_dev = NULL;
#ifndef CONFIG_NET_NS
	switch (type) {
	case PKT_IFDEV_WLAN:
		if ((if_dev = __dev_get_by_name(&init_net, D_INTF_NAME_WLAN)))
			ret = get_ndev_ifaddr(if_dev, D_INTF_NAME_WLAN,
					&dconfig->dev_info[PKT_IFDEV_WLAN].ifadr,
					&dconfig->dev_info[PKT_IFDEV_WLAN].if_index);
		break;

	case PKT_IFDEV_MOBILE:
		if ((if_dev = __dev_get_by_name(&init_net, dconfig->mobile_name)))
			ret = get_ndev_ifaddr(if_dev, dconfig->mobile_name,
					&dconfig->dev_info[PKT_IFDEV_MOBILE].ifadr,
					&dconfig->dev_info[PKT_IFDEV_MOBILE].if_index);
		break;

	default:
		break;
	}
#endif
	return ret;
}

static int get_hw_info(PKT_IFDEV_TYPE type, struct in_ifaddr *ifadr,
		int* hw_index) {
	switch (type) {
	case PKT_IFDEV_WLAN:
		if (!dmap_test_bit(&dconfig->switch_map,
				D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN)) {
			return D_NON_INITED_WLAN;
		}
		break;

	case PKT_IFDEV_MOBILE:
		if (!dmap_test_bit(&dconfig->switch_map,
				D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE)) {
			return D_NON_INITED_MOBILE;
		}
		break;

	default:
		return D_OUT_RANGE_IFDEV;
	}

	*hw_index = dconfig->dev_info[type].if_index;
	memset(ifadr, 0, SIZE_IN_IFADDR);
	memcpy(ifadr, &dconfig->dev_info[type].ifadr, SIZE_IN_IFADDR);
	return D_SUCCESS;
}

static int get_gate_ip(PKT_IFDEV_TYPE type, __be32* ip) {
	switch (type) {
		case PKT_IFDEV_WLAN:
		if (!dmap_test_bit(&dconfig->switch_map,D_SWITCH_BIT_GATE_IP_INITED_WLAN)) {
			return D_NON_INITED_WLAN;
		}
		break;

		case PKT_IFDEV_MOBILE:
		if (!dmap_test_bit(&dconfig->switch_map,D_SWITCH_BIT_GATE_IP_INITED_MOBILE)) {
			return D_NON_INITED_MOBILE;
		}
		break;

		default:
		return D_OUT_RANGE_IFDEV;
	}

	memset(ip,0,SIZE_IPV4_BE32);
	memcpy(ip,&dconfig->dev_info[type].gate_ipv4,SIZE_IPV4_BE32);
	return D_SUCCESS;
}

static bool is_addr_eq_gateway(PKT_IFDEV_TYPE type, __be32 da, int af) {
	__be32 ip;

	switch (af) {
	case AF_INET:
		if (!get_gate_ip(type, &ip) && is_addr_equal(&da, &ip, AF_INET))
			return true;
		break;

	case AF_INET6:
		break;

	default:
		break;
	}
	return false;
}

static inline bool is_addr_in_subnet(PKT_IFDEV_TYPE type, __be32 da, int af) {
	int hw_index = 0;
	struct in_ifaddr ifadr;

	switch (af) {
	case AF_INET:
		if (!get_hw_info(type, &ifadr, &hw_index)) {
			if ((ifadr.ifa_mask & da) == (ifadr.ifa_mask & ifadr.ifa_local))
				return true;
		}
		break;

	default:
		break;
	}

	return false;
}

static unsigned long get_lastrx_of_netdev(PKT_IFDEV_TYPE type) {
	struct net_device *if_dev = NULL;
#ifndef CONFIG_NET_NS
	switch (type) {
	case PKT_IFDEV_WLAN:
		if_dev = __dev_get_by_name(&init_net, D_INTF_NAME_WLAN);
		break;

	case PKT_IFDEV_MOBILE:
		if_dev = __dev_get_by_name(&init_net, dconfig->mobile_name);
		break;

	default:
		break;
	}
#endif
	if (if_dev)
		return if_dev->last_rx;
	else
		return 0;
}

static int uid_hash_func(PKT_IFDEV_TYPE ifdev, uid_t uid, uid_hash* hash) {
	int y_axis = 0;
	int x_axis = D_UID_HASH_INVALID;

	if (ifdev >= PKT_IFDEV_MAX)
		return D_OUT_RANGE_IFDEV;

	if (uid == 0)/* 0 */
		x_axis = UID_HASH_ROOT;
	else if (0 < uid && uid < 1000)/* (0,1000) */
		x_axis = UID_HASH_KERNEL;
	else if (uid == 1000)/* 1000 */
		x_axis = UID_HASH_SYSTEM;
	else if (1000 < uid && uid < FIRST_APP_UID)/* (1000,10000) */
		x_axis = UID_HASH_ANDROID;
	else if ( FIRST_APP_UID <= uid && uid <= LAST_APP_UID) {/* [10000, 19999] */
		x_axis = (uid % FIRST_APP_UID) % D_RE1_HASH_SIZE + UID_HASH_APP_START;
		y_axis = (uid % FIRST_APP_UID) / D_RE1_HASH_SIZE;
	} else
		return D_OUT_RANGE_UID;

	if (x_axis >= 0)
		x_axis += ifdev * D_CACHE_USLOT_CNT_HALF;

	if (x_axis >= ((1 + ifdev) * D_CACHE_USLOT_CNT_HALF))
		return D_OUT_RANGE_UID_X;

	if (y_axis >= D_USLOT_Y_CNT)
		return D_OUT_RANGE_UID_Y;

	hash->x = x_axis;
	hash->y = y_axis;

	return D_SUCCESS;
}

static bool driver_not_ready(void) {
	if (dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_FRAMEWORK_DISABLED) /*user disable this feature*/
	|| dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_FRAMEWORK_ENABLE_PPPOE) /*user enable pppoe*/
	|| dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_FRAMEWORK_HTTP_DIAGN) /*framework diagn with http ongoing*/
	|| !dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_MEM_INITED) /*ring buf alloc fail*/
	|| !dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_DRIVER_INITED) /*driver init fail*/
	|| (!dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_GATE_IP_INITED_WLAN) /*  */
	&& !dmap_test_bit(&dconfig->switch_map, D_SWITCH_BIT_GATE_IP_INITED_MOBILE))/*interface not ready*/
			|| (!dmap_test_bit(&dconfig->switch_map,
					D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN) /**/
					&& !dmap_test_bit(&dconfig->switch_map,
							D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE))/*interface not ready*/
			) {
		return true;
	} else {
		return false;
	}
}

/******************************************************************************
 *
 * net pkt ops
 *
 */

/*
 When the transport_protocol is not IPPROTO_ICMP, the net_device referenced by
 sock will be set with real device by router core,so we can recognize the device
 type explicitly.Otherwise,skb->dev->name & dst->dev->name will be set l0 or wlan
 or ccmnix.
 */
static int npkt_set_interface(net_pkt* pkt, struct sock *sk,
		struct sk_buff *skb, struct iphdr *s_hdr) {
	int ret = 0;
	int* hw_index = &pkt->hw_index;
	struct dst_entry * dst = NULL;
	struct net_device* dev = NULL;
	struct in_ifaddr _ifadr, *ifadr = &_ifadr;

	dst = __sk_dst_get(sk);
	dev = dst ? dst->dev : skb->dev;

	if (!dev || !dev->name) {
		return D_INVALID_PARAMS;
	}

	if (DEV_IS_LP0(dev)) {
		pkt->ifdev = PKT_IFDEV_LP0;
	} else if (DEV_IS_P2P0(dev)) {
		pkt->ifdev = PKT_IFDEV_P2P0;
	} else if (DEV_IS_WLAN0(dev)) {
		pkt->ifdev = PKT_IFDEV_WLAN;
	} else if (DEV_IS_MOBILE(dev)) {
		pkt->ifdev = PKT_IFDEV_MOBILE;
	} else {
		pkt->ifdev = PKT_IFDEV_OTHER;
	}

	switch (pkt->ifdev) {
	case PKT_IFDEV_WLAN:
	case PKT_IFDEV_MOBILE:
		ret = get_hw_info(pkt->ifdev, ifadr, hw_index);
		break;

	case PKT_IFDEV_LP0:
		if (s_hdr) {
			if (is_addr_equal(&s_hdr->daddr, LP_0_ADDR, AF_INET)) {
				pkt->ifdev = PKT_IFDEV_LP0;
			} else {
				if (!(ret = get_hw_info(PKT_IFDEV_WLAN, ifadr, hw_index))
						== 0) {
					if (is_addr_equal(&ifadr->ifa_local, &s_hdr->saddr,
					AF_INET)) {
						pkt->ifdev = PKT_IFDEV_WLAN;
					} else {
						if (!(ret = get_hw_info(PKT_IFDEV_MOBILE, ifadr,
								hw_index))) {
							if (is_addr_equal(&ifadr->ifa_local, &s_hdr->saddr,
							AF_INET)) {
								pkt->ifdev = PKT_IFDEV_MOBILE;
							} else {
								pkt->ifdev = PKT_IFDEV_OTHER;
							}
						}
					}
				}
			}
		}
		break;

	default:
		break;
	}

	if (ret)
		return ret;

	switch (pkt->ifdev) {
	case PKT_IFDEV_WLAN:
		return D_SUCCESS;

	case PKT_IFDEV_MOBILE:
		if (dmap_test_bit(&dconfig->switch_map,
				D_SWITCH_BIT_MOBILE_GLOBAL_DISABLED))
			return D_MOBILE_GLOBAL_DISABLED;
		return D_SUCCESS;

	default:
		return D_OUT_RANGE_IFDEV;
	}
}

static int npkt_transport_proto_check(net_pkt* pkt, struct sock *sk,
		struct sk_buff *skb) {
	int type = 0;
	int code = 0;
	u32 seq, snd_una;
	int err = D_ECODE_OK;
	int t_proto = 0;
	struct tcp_sock *tp;
	struct tcphdr *th = NULL;
	struct udphdr *uh = NULL;
	struct iphdr *s_hdr = NULL;
	struct iphdr *i_hdr = NULL;
	struct request_sock *fastopen = NULL;

	s_hdr = ip_hdr(skb);
	/* Ignore IPV6 */
	if (!is_iphdr_valid(s_hdr))
		return D_ECODE_OK;
	else
		t_proto = s_hdr->protocol;

	if (t_proto == IPPROTO_TCP) {
		th = tcp_hdr(skb);
		if (!th || !(th->syn || th->rst || th->fin) || (th->syn && th->rst)
				|| (th->syn && th->fin) || (th->rst && th->fin))
			return D_ECODE_OK;

		if (npkt_set_interface(pkt, sk, skb, s_hdr))
			return D_ECODE_OK;

		/* for rx */
		if (is_addr_in_subnet(pkt->ifdev, s_hdr->saddr, AF(s_hdr)))
			return D_ECODE_OK;

		if (th->syn && th->ack)
			return D_ECODE_RCU_TCP_RX_SYN_ACK;

		if (th->rst)
			return D_ECODE_RCU_TCP_RX_RST;

		if (th->fin)
			return D_ECODE_RCU_TCP_RX_FIN;

		return D_ECODE_OK;
	} else if (t_proto == IPPROTO_UDP) {
		uh = udp_hdr(skb);
		if (IS_NOT_DNS(uh)) {
			return D_ECODE_OK;
		}

		if (npkt_set_interface(pkt, sk, skb, s_hdr))
			return D_ECODE_OK;

		if (UDP_DP(uh) == D_UDP_PORT_DNS)
			err = D_ECODE_RCU_DNS_QUERY;
		else
			err = D_ECODE_RCU_DNS_RESPONSE;
	} else if (t_proto == IPPROTO_ICMP) {
		type = icmp_hdr(skb)->type;
		code = icmp_hdr(skb)->code;
		i_hdr = (struct iphdr *) skb->data;

		if (i_hdr == NULL)
			return D_ECODE_OK;

		switch (i_hdr->protocol) {
		case IPPROTO_TCP:
			switch (sk->sk_state) {
			case TCP_TIME_WAIT:
			case TCP_CLOSE:
			case TCP_LISTEN:
				return D_ECODE_OK;
			default:
				break;
			}

			if (unlikely(i_hdr->ttl < inet_sk(sk)->min_ttl)) {
				return D_ECODE_TCP_WRN_TCPMINTTLDROP;
			}

			th = (struct tcphdr *) (TRANSPORT_PTR(skb, i_hdr));
			if (!th || !(th->syn || th->ack || th->fin || th->rst)) {
				return D_ECODE_OK;
			}

			tp = tcp_sk(sk);
			seq = ntohl(th->seq);
			fastopen = tp->fastopen_rsk;
			snd_una = fastopen ? tcp_rsk(fastopen)->snt_isn : tp->snd_una;
			if (sk->sk_state != TCP_LISTEN
					&& !between(seq, snd_una, tp->snd_nxt)) {
				return D_ECODE_TCP_WRN_FP_OUTOFWINDOWICMPS;
			}
			break;

		case IPPROTO_UDP:
		case IPPROTO_ICMP:
			break;

		default:
			return D_ECODE_OK;
		}

		ICMP_CODE_2_ERR(type, code, err);
	}
	return err;
}

/*
 when internet is offline,some ap router will return bogus pkt,such as:
 For huawei ws8833,gateway will return tcp-syn-ack like: ip.src = 129.129.129.129
 For xiaomi router,gateway will return dns like: ip.src = 1.6.11.12
 */
static int npkt_tcp_rx_init(net_pkt* pkt, struct sk_buff* skb) {
	d_dev_info* dev_info = NULL;
	struct tcphdr *th = tcp_hdr(skb);
	struct iphdr *s_hdr = ip_hdr(skb);

	if (dconfig->log_level <= D_MSG_DEBUG && th && s_hdr) {
		memcpy(&pkt->s_hdr, s_hdr, SIZE_IP_HDR);
		memcpy(&pkt->t_hdr, th, SIZE_TCP_HDR);
	}

	if (is_addr_in_subnet(pkt->ifdev, s_hdr->saddr, AF(s_hdr))) {
		return D_TCP_SRC_IN_SUBNET;
	}

	if (pkt->uid == 1000) {
		dev_info = &dconfig->dev_info[pkt->ifdev];
		if (is_in_set(dev_info->tcp_src_black_list,
				dev_info->tcp_src_black_list_cnt, s_hdr->saddr))
			return D_TCP_SRC_IN_BLACK_LIST;
	}

	pkt->src = PKT_SRC_GATE_WAY;
	pkt->out = PKT_SRC_INTERNET;
	pkt->c_type = D_NET_PKT_RX_TCP;
	return D_SUCCESS;
}

static int npkt_tcp_tx_init(net_pkt* pkt, struct sk_buff* skb) {
	struct tcphdr *th = tcp_hdr(skb);
	struct iphdr *s_hdr = ip_hdr(skb);

	if (is_addr_in_subnet(pkt->ifdev, s_hdr->daddr, AF(s_hdr))) {
		return D_TCP_DST_IN_SUBNET;
	}

	if (dconfig->log_level <= D_MSG_DEBUG && th && s_hdr) {
		memcpy(&pkt->s_hdr, s_hdr, SIZE_IP_HDR);
		memcpy(&pkt->t_hdr, th, SIZE_TCP_HDR);
	}

	pkt->src = PKT_SRC_GATE_WAY;
	pkt->out = PKT_SRC_INTERNET;
	pkt->c_type = D_NET_PKT_RX_TCP;
	return D_SUCCESS;
}

static int npkt_unreach_record_init(net_pkt* pkt, struct sk_buff* skb) {
	int ret = 0;
	struct tcphdr *th = NULL;
	struct udphdr *uh = NULL;

	struct iphdr *s_hdr = NULL;
	struct iphdr *i_hdr = NULL;

	if (!skb || !skb->dev)
		return D_INVALID_PARAMS;

	s_hdr = ip_hdr(skb);
	i_hdr = (struct iphdr *) skb->data;

	if (!is_iphdr_valid(s_hdr) || !is_iphdr_valid(i_hdr))
		return D_INVALID_IP_HDR;

	/* we just take care of (icmp+ip+tcp/udp), ignore (icmp+ip+icmp) */
	switch (i_hdr->protocol) {
	case IPPROTO_TCP:
	case IPPROTO_UDP:
		/* 1.PKT_IFDEV_TYPE check */
		if ((ret = npkt_set_interface(pkt, pkt->sk, skb, s_hdr)))
			return ret;

		/* 2.PKT_SRC_TYPE for rx */
		if (is_addr_equal(&s_hdr->saddr, &s_hdr->daddr, AF(s_hdr))) {
			pkt->src = PKT_SRC_KERNEL;
		} else if (is_addr_eq_gateway(pkt->ifdev, s_hdr->saddr, AF(i_hdr))) {
			pkt->src = PKT_SRC_GATE_WAY;
		} else if (is_addr_in_subnet(pkt->ifdev, s_hdr->saddr, AF(i_hdr))) {
			pkt->src = PKT_SRC_SUBNET;
		} else
			pkt->src = PKT_SRC_INTERNET;

		/* 3.PKT_OUT_TYPE */
		if (!is_addr_in_subnet(pkt->ifdev, i_hdr->daddr, AF(i_hdr)))
			pkt->out = PKT_OUT_INTERNET;
		else if (is_addr_eq_gateway(pkt->ifdev, i_hdr->daddr, AF(i_hdr)))
			pkt->out = PKT_OUT_GATEWAY;
		else
			pkt->out = PKT_OUT_SUBNET;

		switch (pkt->src) {
		case PKT_SRC_SUBNET:
			return D_UNREACH_SRC_SUBNET;

		case PKT_SRC_INTERNET:
			if (dmap_test_bit(&dconfig->switch_map,
					D_SWITCH_BIT_UNREACH_SRC_INTERNET_ENABLED))
				break;
			else
				return D_UNREACH_SRC_INTERNET;

		default:
			break;
		}

		break;

	default:
		return D_UNREACH_UNSPOORT_PROTO;
	}

	/* 4.PKT_TRANSPORT_PROTO_TYPE */
	switch (i_hdr->protocol) {
	case IPPROTO_TCP:
		if (pkt->out != PKT_OUT_INTERNET)
			return D_UNREACH_TCP_OUT_NOT_INTERNET;

		if (dconfig->log_level <= D_MSG_DEBUG) {
			th = (struct tcphdr *) (TRANSPORT_PTR(skb, i_hdr));
			memcpy(&pkt->t_hdr, th, SIZE_TCP_HDR);
		}
		break;

	case IPPROTO_UDP:
		uh = (struct udphdr *) (TRANSPORT_PTR(skb, i_hdr));
		switch (pkt->out) {
		case PKT_OUT_SUBNET:
			return D_UNREACH_UDP_OUT_SUBNET;

		case PKT_OUT_GATEWAY:
			if (pkt->uid != 0 || IS_NOT_DNS(uh))
				return D_UNREACH_UDP_OUT_GATE_NO_DNS;
			break;

		default:
			break;
		}

		if (dconfig->log_level <= D_MSG_DEBUG) {
			memcpy(&pkt->u_hdr, uh, SIZE_UDP_HDR);
		}
		break;

	default:
		break;
	}

	pkt->c_type = D_NET_PKT_RX_ICMP_UNREACH;

	if (dconfig->log_level <= D_MSG_DEBUG) {
		memcpy(&pkt->s_hdr, s_hdr, SIZE_IP_HDR);
		memcpy(&pkt->i_hdr, i_hdr, SIZE_IP_HDR);
	}

	return D_SUCCESS;
}

static int npkt_dns_parse(net_pkt* pkt, struct iphdr * s_hdr,
		const dns_hdr* dns, const u8* data, int len) {
	int index = 0;
	int offset = 0;
	u16 q_cnt = 0;
	u16 a_cnt = 0;
	u16 aw_type = 0;
	u16 aw_dlen = 0;
	int aw_offset = 0;
	u8* rsp_address = NULL;
	int dns_addr_eqgate_cnt = 0;

	dns_aw_hdr* aw = NULL;
	u8* pos = (u8*) data;
	u8* answer_qr = (u8*) data;

	if (!pkt || !s_hdr || !dns || !data || !len)
		return D_INVALID_PARAMS;

	q_cnt = be16_to_cpu(dns->q_cnt);
	a_cnt = be16_to_cpu(dns->a_cnt);

	if (!q_cnt || !a_cnt)
		return D_INVALID_DNS_RSP_AQ_CNT;

	for (index = 0; index < q_cnt; index++) {
		while (offset++ < len && *pos++ != 0)
			;
		offset += 4;
		pos += 4;
	}

	answer_qr = pos;

	for (index = 0; index < a_cnt && offset <= len; index++) {
		aw = (dns_aw_hdr*) answer_qr;
		aw_type = be16_to_cpu(aw->type);
		aw_dlen = be16_to_cpu(aw->dlen);
		aw_offset = SIZE_DNS_ANSWER_R_HDR + be16_to_cpu(aw->dlen);
		if (aw_type == D_DNS_REPLY_ANSWER_R_TYPE_HOST_ADDR && aw_dlen == 0x4) {
			rsp_address = answer_qr + SIZE_DNS_ANSWER_R_HDR;
			if (is_addr_eq_gateway(pkt->ifdev,*(rsp_address), AF_INET)) {
				dns_addr_eqgate_cnt++;
			}
		}
		answer_qr += aw_offset;
		offset += aw_offset;
	}

	if (dns_addr_eqgate_cnt && (dns_addr_eqgate_cnt == a_cnt || dns_addr_eqgate_cnt == (a_cnt - 1)))
		pkt->icode = D_ECODE_RCU_DNS_RESPONSE_GATEWAY_IP;

	return D_SUCCESS;
}

static int npkt_dns_record_init(net_pkt* pkt, struct sk_buff* skb) {
	int dns_len = 0;
	int rply_code = 0;
	dns_hdr* dns = NULL;
	u8* dns_data = NULL;
	d_dev_info* dev_info = NULL;

	struct iphdr *s_hdr = NULL;
	struct udphdr *uh = NULL;

	s_hdr = ip_hdr(skb);
	uh = udp_hdr(skb);

	if (dconfig->log_level <= D_MSG_DEBUG) {
		memcpy(&pkt->s_hdr, s_hdr, SIZE_IP_HDR);
		memcpy(&pkt->u_hdr, uh, SIZE_UDP_HDR);
	}

	switch (pkt->icode) {
	case D_ECODE_RCU_DNS_QUERY:
		pkt->src = PKT_SRC_KERNEL;
		pkt->c_type = D_NET_PKT_TX_DNS_QUERY;

		/* 3.PKT_OUT_TYPE */
		if (!is_addr_in_subnet(pkt->ifdev, s_hdr->daddr, AF(s_hdr)))
			pkt->out = PKT_OUT_INTERNET;
		else if (is_addr_eq_gateway(pkt->ifdev, s_hdr->daddr, AF(s_hdr)))
			pkt->out = PKT_OUT_GATEWAY;
		else
			pkt->out = PKT_OUT_SUBNET;

		break;

	case D_ECODE_RCU_DNS_RESPONSE:
		pkt->src = PKT_SRC_GATE_WAY;
		pkt->out = PKT_OUT_INTERNET;
		pkt->c_type = D_NET_PKT_RX_DNS_RESPONSE;

		dns_len = be16_to_cpu(uh->len) - SIZE_UDP_HDR;
		if (dns_len <= SIZE_DNS_HDR)
			return D_INVALID_DNS_RSP;

		dns = (dns_hdr*) ((u8*) skb->data + SIZE_UDP_HDR);
		dns_data = ((u8*) skb->data + SIZE_UDP_HDR + SIZE_DNS_HDR);
		rply_code = (be16_to_cpu(dns->flags)) & D_DNS_REPLY_MASK;
		LOG_DEBUG("DNS:L=%d,C=%d\n", dns_len, rply_code);

		switch (rply_code) {
		case D_DNS_REPLY_REFUSED:
			pkt->icode = D_ECODE_RCU_DNS_RESPONSE_C5_REFUSED;
			break;

		case D_DNS_REPLY_SUCCESS:
			dev_info = &dconfig->dev_info[pkt->ifdev];
			if (!is_in_set(dev_info->dns, dev_info->dns_cnt, s_hdr->saddr)) {
				return npkt_dns_parse(pkt, s_hdr, dns, dns_data,dns_len - SIZE_DNS_HDR);
			}
			break;

		default:
			break;
		}
		break;
	default:
		break;
	}

	return D_SUCCESS;
}

static int npkt_is_invalid(net_pkt* pkt) {
	int ret = 0;
	uid_hash hash = { 0, 0 };

	switch (pkt->ifdev) {
	case PKT_IFDEV_WLAN:
	case PKT_IFDEV_MOBILE:
		break;
	default:
		return D_OUT_RANGE_IFDEV;
	}

	if (pkt->err >= D_NOTIFY_CODE_MAX) {
		return D_OUT_RANGE_PKT_ECODE;
	}

	if ((ret = uid_hash_func(pkt->ifdev, pkt->uid, &hash))) {
		return ret;
	}

	if (!dops_need_do_stat(pkt->ifdev, pkt->uid, pkt->err)) {
		return D_PKT_NO_NEED_TO_STAT;
	}

	return D_SUCCESS;
}

static void npkt_analysis_ecode(net_pkt* pkt) {
	int err = 0xff;
	int icode = pkt->icode;
	switch (pkt->src) {
	case PKT_SRC_KERNEL:
		switch (icode) {
		case D_ECODE_RCU_DNS_QUERY:
			err = D_NOTIFY_ERR_DNS_NOACK;
			break;
		case D_ECODE_RCU_TCP_TX_FIN:
		case D_ECODE_RCU_TCP_TX_RST:
			err = D_NOTIFY_ERR_TCP_SELF_RESET;
			break;

		case D_ECODE_ICMP_HOST_UNREACH:
			err = D_NOTIFY_ERR_LOCAL_FAIL;
			break;

		case D_ECODE_ICMP_NET_ANO:
			err = D_NOTIFY_ERR_IPTABLE_FORBIDDEN_TX;
			break;
		case D_ECODE_ICMP_HOST_ANO:
			err = D_NOTIFY_ERR_IPTABLE_FORBIDDEN_RX;
			break;
		case D_ECODE_ICMP_PORT_UNREACH:
			err = D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PORT;
			break;
		case D_ECODE_ICMP_PKT_FILTERED:
			err = D_NOTIFY_ERR_IPTABLE_FORBIDDEN_PKT;
			break;
		default:
			break;
		}

		break;
	case PKT_SRC_GATE_WAY:
		switch (icode) {
		case D_ECODE_RCU_TCP_RX_SYN_ACK:
			err = D_NOTIFY_NET_ACCESS_OK;
			break;
		case D_ECODE_RCU_TCP_RX_RST:
			err = D_NOTIFY_ERR_TCP_WEB_RESET;
			break;
		case D_ECODE_RCU_TCP_RX_FIN:
			err = D_NOTIFY_ERR_TCP_WEB_RESET;
			break;
		case D_ECODE_RCU_DNS_RESPONSE:
			err = D_NOTIFY_ERR_DNS_NOACK;
			break;
		case D_ECODE_RCU_DNS_RESPONSE_C5_REFUSED:
			err = D_NOTIFY_ERR_DNS_REFUSED;
			break;
		case D_ECODE_RCU_DNS_RESPONSE_GATEWAY_IP:
			err = D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP;
			break;
		case D_ECODE_ICMP_HOST_UNREACH:
		case D_ECODE_ICMP_NET_UNREACH:
			err = D_NOTIFY_ERR_INTERNET_OFFLINE;
			break;
		case D_ECODE_ICMP_NET_ANO:
			err = D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_TX;
			break;
		case D_ECODE_ICMP_HOST_ANO:
			err = D_NOTIFY_ERR_GATE_IPTABLE_FORBIDDEN_RX;
			break;
		default:
			break;
		}

		break;

	case PKT_SRC_INTERNET:
		break;

	default:
		break;
	}
	pkt->err = err;
}

void net_pkt_analysis(struct sock*sk, struct sk_buff* skb) {
	int ret = 0;
	net_pkt _pkt, *pkt = &_pkt;

	if (!sk || !sk->sk_socket || !skb || !skb->data || driver_not_ready()) {
		return;
	}

	memset(pkt, 0, sizeof(net_pkt));
	/* 1.covert all kinds of protocol reason code to diagn code set */
	pkt->icode = npkt_transport_proto_check(pkt, sk, skb);
	if (pkt->icode == D_ECODE_OK) {
		return;
	}

	/* 2.get uid from sk */
	/* sock_i_uid always use read_lock_bh,in soft_irq,maybe result in exception 
	 * so we use D_SOCK_UID instead.
	 */
	INIT_PKT_UID(pkt, sk);

	/* 3.init net_pkt content with skb and sk*/
	switch (pkt->icode) {
	case D_ECODE_RCU_TCP_RX_SYN_ACK:
	case D_ECODE_RCU_TCP_RX_RST:
	case D_ECODE_RCU_TCP_RX_FIN:
		ret = npkt_tcp_rx_init(pkt, skb);
		break;

	case D_ECODE_RCU_TCP_TX_RST:
	case D_ECODE_RCU_TCP_TX_FIN:
		ret = npkt_tcp_tx_init(pkt, skb);
		break;

	case D_ECODE_RCU_DNS_QUERY:
	case D_ECODE_RCU_DNS_RESPONSE:
		if (pkt->uid == 0)
			ret = npkt_dns_record_init(pkt, skb);
		else
			ret = 1;
		break;

	default:
		ret = npkt_unreach_record_init(pkt, skb);
		break;
	}

	/* 4.generate diagn error code */
	npkt_analysis_ecode(pkt);

	/*
	 * 	5.Check whether this pkt is need to be enqueued to stat ring buf
	 *	and wake up diagn_thread
	 * 
	 *	Invalid case as follows:
	 * 	1.packet interface is not PKT_IFDEV_WLAN or PKT_IFDEV_MOBILE
	 *
	 * 	2.pkt->src is not PKT_SRC_KERNEL or PKT_SRC_GATE_WAY,to avoid receiving
	 *		so many hack packets from special web server
	 *
	 * 	3.pkt->err >= D_NOTIFY_CODE_MAX,avoid these kind of packet
	 *
	 * 	4.uid hash fail,we just support these uids:
	 *		0				--->	0;
	 *		(0,1000)		--->	1;
	 *		1000			--->	2;
	 *		(1000,10000)	--->	3;
	 *		10000			--->	4;
	 *		..
	 *		.
	 *		10507			--->	511;
	 *
	 *	5.dops_need_do_stat,to make sure the stack of tcp/ip will wake up diagn_thread 
	 *		just once for this special kind of packet
	 *
	 *	1).return value:
	 *		= 0	no need to wake diagn_thread
	 *		= 1	new excpetion happen,so we need to wake diagn_thread
	 *
	 *	2).detail process
	 *	A.for tcp syn+ack,
	 *		if we found global exception in dconfig->notify_map
	 *			add net_pkt to ring buf,then wake up diagn_thread
	 *		else
	 *			just return,diagn_thread will be always sleep.
	 *
	 *		if we found local firewall exception in skuer_of_this_uid->notify_map
	 *			add net_pkt to ring buf,then wake up diagn_thread
	 *		else
	 *			just return
	 *
	 *	B.for tcp rst by server
	 *		always add net_pkt to ring buf,then wake up diagn_thread
	 *
	 *	C.for global exception
	 *		if we found the individual exception in skuer_of_this_uid->notify_map
	 *		with match condition( 
	 *			ifdev,		# PKT_IFDEV_TYPE
	 *			uid,		# 
	 *			err_code	# D_NOTIFY_CODE
	 *			)
	 *			we will wake up diagn_thread
	 *		else
	 *			just return
	 *
	 *	3).conclusion
	 *		This can make sure that the same error identified by (ifdev,uid,err_code)
	 *		will be notified to framework once before network recover or switch.
	 *
	 *		The new packet coming from TCP/IP-STACK,will not wake up diagn_thread 
	 *		after the same error had been notified to framework.
	 *
	 */
	if (ret || (ret = npkt_is_invalid(pkt))) {
		if (dconfig->log_level <= D_MSG_DEBUG) {
			PRINT_PKT(pkt);
			LOG_DEBUG("%s\n", D_OPS_RET_STR(ret));
		}
		return;
	}

	dops_npkt_enqueue(pkt);
	return;
}

/******************************************************************************
 *
 */
static int dpkt_mem_ops(d_write_block* block, D_MEM_OPS_TYPE op_type,
		d_stat_pkt** data) {
	int index = 0;
	u8 invalid = 0;
	d_stat_pkt* dpkt = NULL;
	struct list_head* fhead = NULL, *whead = NULL;

	switch (op_type) {
	case D_MEM_OPS_GET_NODE:
	case D_MEM_OPS_GET_DNS_NAK_NODE:
	case D_MEM_OPS_GET_TCP_SYN_NODE:
	case D_MEM_OPS_GET_TCP_RST_NODE:
		if (!data)
			invalid |= 1;

	default:
		if (!block)
			invalid |= 2;
		break;
	}

	if (invalid) {
		return D_INVALID_PARAMS;
	}

	fhead = &block->free_list;
	whead = &block->write_list;

	switch (op_type) {
	case D_MEM_OPS_RESET:
		dpkt = block->d_pkt;
		memset(dpkt, 0, SIZE_D_STAT_PKT * D_CACHE_NPKT_CNT);
		INIT_LIST_HEAD(fhead);
		INIT_LIST_HEAD(whead);
		for (index = 0; index < D_CACHE_NPKT_CNT; index++) {
			list_add_tail(&dpkt->flist, fhead);
			dpkt++;
		}

		block->used_count = 0;
		block->free_count = D_CACHE_NPKT_CNT;
		break;

	case D_MEM_OPS_GET_TCP_SYN_NODE:
		*data = &block->s_pkt;
		break;

	case D_MEM_OPS_GET_TCP_RST_NODE:
		*data = &block->r_pkt;
		break;

	case D_MEM_OPS_GET_DNS_NAK_NODE:
		*data = &block->n_pkt;
		break;

	case D_MEM_OPS_GET_NODE:
		if (list_empty(fhead)) {
			dpkt = NULL;
		} else {
			dpkt = list_first_entry(fhead, d_stat_pkt, flist);
			__list_del_entry(&dpkt->flist);
			memset(dpkt, 0, SIZE_D_STAT_PKT);
			list_add_tail(&dpkt->wlist, whead);
			block->used_count++;
			block->free_count--;
		}
		*data = dpkt;
		break;

	default:
		return D_OUT_RANGE_OPS_TYPE;
	}
	/*
	 LOG_DEBUG("CPU[%d] OP:%d\tUC:%d\tFC:%d\n", block->cpu, op_type,
	 block->used_count, block->free_count);
	 */
	return D_SUCCESS;
}

static void dops_mem_stat_get(PKT_IFDEV_TYPE ifdev, d_mem_cost* cost) {
	memset(cost, 0, sizeof(d_mem_cost));
	cost->skuser_cnt = dconfig->dev_info[ifdev].exp_ucnt;
	cost->stream_size += cost->stream_cnt * sizeof(d_stream);
	cost->user_size += cost->skuser_cnt * sizeof(d_user_slot);
	cost->cost = cost->user_size + cost->stream_size;
}

static int dops_mem_stat_dump(char* buf, int max, PKT_IFDEV_TYPE ifdev) {
	int size = 0;
	d_mem_cost cost;

	dops_mem_stat_get(ifdev, &cost);

	if (size < max)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
				"mem stat for %s\n",
				ifdev == PKT_IFDEV_WLAN ?
				D_INTF_NAME_WLAN :
											dconfig->mobile_name);

	if (size < max)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
				"cost.user_size  = %u [ %d * %lu ]\n", cost.user_size,
				cost.skuser_cnt, sizeof(d_user_slot));

	if (size < max)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
				"cost.stream_size  = %u [ %d * %lu ]\n", cost.stream_size,
				cost.stream_cnt, sizeof(d_stream));

	if (size < max)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
				"cost.cost  = %u Bytes, %u kBytes\n\n", cost.cost,
				cost.cost / 1024);

	return size;
}

/******************************************************************************
 *
 */
static void dops_config_reset(PKT_IFDEV_TYPE ifdev) {
	D_MAP* dns_req = NULL;
	D_MAP* dpkt_enque_cnt = NULL;
	d_dev_info* dev_info = NULL;

	dev_info = &dconfig->dev_info[ifdev];
	dns_req = &dev_info->dns_req;
	dpkt_enque_cnt = &dev_info->dpkt_enque_cnt;

	dev_info->d_last_req = 0;
	dmap_reset(dns_req);
	dmap_reset(dpkt_enque_cnt);
	memset(dev_info->dres, 0, sizeof(dev_info->dres));
}

static void dops_dpkt_init(d_stat_pkt* dpkt, net_pkt* pkt) {
	dpkt->add_ts = CURRENT_MS;
	dpkt->c_type = pkt->c_type;
	dpkt->uid = pkt->uid;
	dpkt->hw_index = pkt->hw_index;
	dpkt->ifdev = pkt->ifdev;
	dpkt->err = pkt->err;
	dpkt->icode = pkt->icode;
}

static void dops_npkt_enqueue(net_pkt* pkt) {
	int cpu = 0;
	int bit1 = 0;
	int bit2 = 0;
	u32 past = 0;
	int enque_cnt = 0;
	int need_wake = 0;
	int need_enqueue = 0;
	int wake_threshold = 0;
	D_MEM_OPS_TYPE op = 0;
	D_MAP* dns_req = NULL;
	u32* d_last_req = NULL;
	d_stat_pkt* dpkt = NULL;
	d_write_block* block = NULL;
	D_MAP* dpkt_enque_cnt = NULL;
	unsigned long flags ;
	bit1 = D_THREAD_SYNC_DEV_WLAN_BIT;
	dns_req = &dconfig->dev_info[pkt->ifdev].dns_req;
	d_last_req = &dconfig->dev_info[pkt->ifdev].d_last_req;
	dpkt_enque_cnt = &dconfig->dev_info[pkt->ifdev].dpkt_enque_cnt;

	switch (pkt->c_type) {
	case D_NET_PKT_RX_TCP:
		switch (pkt->err) {
		case D_NOTIFY_NET_ACCESS_OK:
			bit2 = D_THREAD_SYNC_SYN_IN_BIT;
			op = D_MEM_OPS_GET_TCP_SYN_NODE;
			wake_threshold = dconfig->wake_threshold_tcpsync;
			break;

		case D_NOTIFY_ERR_TCP_WEB_RESET:
			bit2 = D_THREAD_SYNC_RST_IN_BIT;
			op = D_MEM_OPS_GET_TCP_RST_NODE;
			wake_threshold = dconfig->wake_threshold_tcpdrst;
			break;

		default:
			return;
		}
		need_wake = 1;
		need_enqueue = 1;
		break;

	case D_NET_PKT_TX_TCP:
		switch (pkt->err) {
		case D_NOTIFY_ERR_TCP_SELF_RESET:
			bit2 = D_THREAD_SYNC_RST_IN_BIT;
			op = D_MEM_OPS_GET_TCP_RST_NODE;
			wake_threshold = dconfig->wake_threshold_tcpsrst;
			break;

		default:
			return;
		}
		break;

	case D_NET_PKT_TX_DNS_QUERY:
		switch (pkt->err) {
		case D_NOTIFY_ERR_DNS_NOACK:
			dmap_inc(dns_req);
			if (!(*d_last_req)) {
				*d_last_req = CURRENT_MS;
			}
			past = CURRENT_MS - *d_last_req;

			if (dmap_get(dns_req) >= dconfig->dnsnack_trigger_threshold
					/*&& (dconfig->dnsnack_update_interval) / 2 <= past
					&& past <= dconfig->dnsnack_update_interval*/) {
				need_wake = 1;
				need_enqueue = 1;
				op = D_MEM_OPS_GET_DNS_NAK_NODE;
				bit2 = D_THREAD_SYNC_DNS_IN_BIT;
				wake_threshold = dconfig->wake_threshold_dnsnack;
				*d_last_req = 0;
				LOG_DEBUG("DNS NAK");
			}
			break;
		default:
			return;
		}
		break;

	case D_NET_PKT_RX_DNS_RESPONSE:
		switch (pkt->err) {
		case D_NOTIFY_ERR_DNS_NOACK:
			break;

		case D_NOTIFY_ERR_DNS_REFUSED:
		case D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP:
			need_enqueue = 1;
			op = D_MEM_OPS_GET_NODE;
			bit2 = D_THREAD_SYNC_EXP_IN_BIT;
			pkt->err = D_NOTIFY_ERR_INTERNET_OFFLINE;
			wake_threshold = dconfig->wake_threshold_unreach;
			break;

		default:
			return;
		}
		if (dmap_get(dns_req) > 0) {
			dmap_reset(dns_req);
			*d_last_req = 0;
		}

		break;

	case D_NET_PKT_RX_ICMP_UNREACH:
		need_enqueue = 1;
		op = D_MEM_OPS_GET_NODE;
		bit2 = D_THREAD_SYNC_EXP_IN_BIT;
		wake_threshold = dconfig->wake_threshold_unreach;
		break;

	default:
		return;
	}

	LOG_DEBUG("dns: %d",dmap_get(dns_req));
	if (!need_enqueue)
		return;

	preempt_disable();
	cpu = smp_processor_id();
	block = (d_write_block *) per_cpu_ptr(dwrite_block, cpu);
	if (block) {
		D_NPKT_SPIN_LOCK(block,flags);
		if (!dpkt_mem_ops(block, op, &dpkt) && dpkt) {
			dops_dpkt_init(dpkt, pkt);
			dmap_inc(dpkt_enque_cnt);
			if (!need_wake
					&& (enque_cnt = dmap_get(dpkt_enque_cnt))
							>= wake_threshold) {
				need_wake = 1;
				dmap_reset(dpkt_enque_cnt);
			}
		}
		D_NPKT_SPIN_UNLOCK(block,flags);
	}
	preempt_enable();

	LOG_DEBUG(" en:%d,wake:%d,pkt:%d\n", need_enqueue, need_wake, enque_cnt);
	PRINT_PKT(pkt);
    flags = 0;
	if (need_wake
			&& !dmap_test_bit(&dconfig->switch_map,
					D_SWITCH_BIT_DIAGN_THREAD_ONLINE)) {
		D_STAT_SPIN_LOCK(flags);
		if (pkt->ifdev == PKT_IFDEV_WLAN)
			__set_bit(D_THREAD_SYNC_DEV_WLAN_BIT, &dconfig->stat_flags);
		__set_bit(bit2, &dconfig->stat_flags);
		D_STAT_SPIN_UNLOCK(flags);
		wake_up_interruptible(&dconfig->stat_waitq);
	}
}

/******************************************************************************
 *
 */
static void dops_dres_init(d_res_info* res, d_stat_pkt* dpkt, int dcnt,
		D_NOTIFY_CODE code, int err_cnt) {
	int index = 0;
	memset(res, 0, sizeof(d_res_info));
	res->ifdev = dpkt->ifdev;
	res->hw_index = dpkt->hw_index;
	res->err_code = code;
	res->err_cnt = err_cnt;
	for (index = 0; index < dcnt && res->uid_cnt < D_RES_UID_CNT; index++) {
		if (!is_in_set(res->uid, res->uid_cnt, dpkt->uid)) {
			res->uid[res->uid_cnt++] = dpkt->uid;
		}
	}
}

static void dops_dres_add(d_res_info* res, d_stat_pkt* dpkt) {
	res->ifdev = dpkt->ifdev;
	res->err_code = dpkt->err;
	res->hw_index = dpkt->hw_index;
	res->err_cnt++;
	if (res->uid_cnt < D_RES_UID_CNT) {
		if (!is_in_set(res->uid, res->uid_cnt, dpkt->uid)) {
			res->uid[res->uid_cnt++] = dpkt->uid;
		}
	}
}

static void dops_dres_remove(d_res_info* res, d_stat_pkt* dpkt) {
	if (res->uid_cnt > 0) {
		if (rm_from_set(res->uid, res->uid_cnt, dpkt->uid)) {
			res->uid_cnt--;
		}
	}

	if (res->err_cnt > 0)
		res->err_cnt--;
}

static int dops_dres_stat(d_res_info* d_res, PKT_IFDEV_TYPE ifdev) {
	int i = 0, j = 0;
	int index = 0;
	int count = 0;
	d_res_info tmp;
	d_res_info *pos = NULL;
	d_dev_info *dev_info = NULL;

	dev_info = &dconfig->dev_info[ifdev];
	for (index = 0; index < D_NOTIFY_CODE_MAX; index++) {
		pos = &dev_info->dres[index];
		if (pos->err_cnt && pos->uid_cnt && count < D_NOTIFY_CODE_MAX) {
			memcpy(d_res + count, pos, sizeof(d_res_info));
			count++;
		}
	}

	for (i = 1; i < count; i++) {
		for (j = 0; j < count - i; j++) {
			if (d_res[j].err_cnt < d_res[j + 1].err_cnt) {
				memset(&tmp, 0, sizeof(d_res_info));
				memcpy(&tmp, &d_res[j], sizeof(d_res_info));
				memcpy(&d_res[j], &d_res[j + 1], sizeof(d_res_info));
				memcpy(&d_res[j + 1], &tmp, sizeof(d_res_info));
			}
		}
	}

	if (dconfig->log_level <= D_MSG_DEBUG) {
		for (i = 0; i < count; i++) {
			PRINT_DRES((&d_res[i]), i, j);
		}
		LOG_DEBUG("\n\n");
	}

	return count;
}

static int dops_dres_read(void* buf, int len) {
	int write = 0;
	int index = 0;
	int ifdev = 0;
	int uid_index = 0;
	d_dev_info* dev = NULL;
	d_res_info* ures = NULL;

	for (ifdev = 0; ifdev < PKT_IFDEV_MAX; ifdev++) {
		dev = &dconfig->dev_info[ifdev];
		for (index = 0; index < dev->ures_cnt && index < D_NOTIFY_CODE_MAX;
				index++) {
			ures = &dev->ures[index];
			if (ures->err_cnt && ures->uid_cnt) {
				if (len - write)
					write += snprintf(buf + write, len - write, "%d\t",
							ures->ifdev);

				if (len - write)
					write += snprintf(buf + write, len - write, "%d\t",
							ures->err_code);

				if (len - write)
					write += snprintf(buf + write, len - write, "%d\t",
							ures->err_cnt);

				if (len - write)
					write += snprintf(buf + write, len - write, "%d\t",
							ures->uid_cnt);

				for (uid_index = 0;
						uid_index < ures->uid_cnt && uid_index < D_RES_UID_CNT;
						uid_index++) {
					if (len - write)
						write += snprintf(buf + write, len - write, "%d,",
								ures->uid[uid_index]);
				}

				if (len - write)
					write += snprintf(buf + write, len - write, ";");

			}
		}

		dev->ures_cnt = 0;
		memset(dev->ures, 0, sizeof(dev->ures));
	}

	return write;
}

static int dops_dres_ready(void) {
	return dconfig->dev_info[PKT_IFDEV_WLAN].ures_cnt
			+ dconfig->dev_info[PKT_IFDEV_MOBILE].ures_cnt;
}

/******************************************************************************
 *
 */
static d_user* dops_user_query(PKT_IFDEV_TYPE ifdev, uid_t uid) {
	uid_hash hash = { 0, 0 };
	d_user* user = NULL;
	d_user_slot* user_slot = NULL;
	//struct hlist_head* u_head = NULL;

	if (!dconfig->user_slot || ifdev >= PKT_IFDEV_MAX)
		return NULL;

	if (uid_hash_func(ifdev, uid, &hash))
		return NULL;

	//u_head = &dconfig->dev_info[ifdev].user_head;
	user_slot = &dconfig->user_slot[hash.x];
	if (!user_slot)
		return NULL;

	user = &user_slot->duser[hash.y];
	if (!user)
		return NULL;

	if (!user->inited) {
		//memset(user, 0, sizeof(d_user_slot));
		//user->ifdev = ifdev;
		//INIT_HLIST_NODE(&user->hlist);
		//hlist_add_head(&user->hlist, u_head);
		user->uid = uid;
		//dconfig->dev_info[ifdev].skuser_cnt++;
		//user_slot->u_cnt++;
		user->inited = 1;
	}
	return user;
}

static int dops_user_dpkt_enqueue(d_stat_pkt* dpkt) {
	u8 ifdev = 0;
	u32 now = 0;
	u32 past = 0;
	d_user* user = NULL;
	d_res_info* dres = NULL;

	if (!dpkt || dpkt->ifdev >= PKT_IFDEV_MAX) {
		return D_INVALID_PARAMS;
	}

	if ((user = dops_user_query(dpkt->ifdev, dpkt->uid)) == NULL) {
		return D_NOT_FIND_USER;
	}

	now = CURRENT_MS;
	ifdev = dpkt->ifdev;
	dres = &dconfig->dev_info[ifdev].dres[dpkt->err];

	if (!user->u_last_add) {
		user->u_last_add = now;
	}

	past = now - user->u_last_add;
	if (past > dconfig->unreach_update_interval) {
		user->u_last_add = 0;
		dops_dres_remove(dres, dpkt);
		LOG_DEBUG("Unreach out of date\n");
	}

	dops_dres_add(dres, dpkt);
	return D_SUCCESS;
}

static void dops_user_list_update(d_user *user,PKT_IFDEV_TYPE ifdev,int add){
	struct hlist_head* u_head = NULL;
	if(!user || ifdev >= PKT_IFDEV_MAX)
		return;

	u_head = &dconfig->dev_info[ifdev].user_head;

	D_USER_SPIN_LOCK();
	if(add){
		if(!user->exped){
			INIT_HLIST_NODE(&user->hlist);
			hlist_add_head(&user->hlist, u_head);
			dconfig->dev_info[ifdev].exp_ucnt++;
			user->exped = 1;
		}
	} else {
		if (!HLIST_EMPTY(u_head)){
			hlist_del_init(&user->hlist);
			user->exped = 0;
			if(dconfig->dev_info[ifdev].exp_ucnt > 0)
				dconfig->dev_info[ifdev].exp_ucnt--;
		}
	}
	D_USER_SPIN_UNLOCK();
}

void dops_user_print(PKT_IFDEV_TYPE ifdev) {
	d_user *pos = NULL;
	struct hlist_node* tmp = NULL;
	struct hlist_head* u_head = NULL;

	u_head = &dconfig->dev_info[ifdev].user_head;
	D_USER_SPIN_LOCK();
	if (!HLIST_EMPTY(u_head)) {
		hlist_for_each_entry_safe(pos, tmp, u_head, hlist)
		{
			PRINT_USER(pos, ifdev);
		}
	}
	D_USER_SPIN_UNLOCK();

	PRINT_NOTIFY_MAP(&dconfig->dev_info[ifdev].notify_map, ifdev, 0);
}

static void dops_user_reset(PKT_IFDEV_TYPE ifdev) {
	d_user *pos = NULL;
	struct hlist_node* tmp = NULL;
	struct hlist_head* u_head = NULL;

	u_head = &dconfig->dev_info[ifdev].user_head;
	D_USER_SPIN_LOCK();
	if (!HLIST_EMPTY(u_head)) {
		hlist_for_each_entry_safe(pos, tmp, u_head, hlist)
		{
			pos->u_last_add = 0;
		}
	}
	D_USER_SPIN_UNLOCK();
}

/******************************************************************************
 *
 */
static int dops_notify_map_ops(D_MAP* pmap, D_NOTIFY_MAP_OPS_TYPE op,
		D_NOTIFY_CODE code) {
	switch (op) {
	case D_NOTIFY_MAP_SET_ONE_BIT:
		dmap_set_bit(pmap, code);
		break;

	case D_NOTIFY_MAP_SET_ALL_BITS:
		dmap_set_all_bits(pmap);
		break;

	case D_NOTIFY_MAP_CLR_ONE_BIT:
		dmap_clear_bit(pmap, code);
		break;

	case D_NOTIFY_MAP_CLR_ALL_BITS:
		dmap_reset(pmap);
		break;

	case D_NOTIFY_MAP_CLR_GLOBAL_BITS:
		dmap_clear_mask(pmap, GLOBAL_EXCEPTION_NOTIFY_MAP);
		break;

	case D_NOTIFY_MAP_CLR_FIREWALL_BITS:
		dmap_clear_mask(pmap, FIREWALL_EXCEPTION_NOTIFY_MAP);
		break;

	case D_NOTIFY_MAP_QUERY_INDIVIDUAL:
		if (dmap_test_bit(pmap, code))
			return 1;
		break;

	case D_NOTIFY_MAP_QUERY_GLOBAL_ERR:
		if (dmap_test_mask(pmap, GLOBAL_EXCEPTION_NOTIFY_MAP))
			return 1;
		break;

	case D_NOTIFY_MAP_QUERY_LOCAL_FIREWALL_ERR:
		if (dmap_test_mask(pmap, FIREWALL_EXCEPTION_NOTIFY_MAP))
			return 1;
		break;
	default:
		return 0;
	}
	return 0;
}

static int dops_notify_map_dump(char* buf, int max, PKT_IFDEV_TYPE ifdev) {
	int size = 0;
	d_user *pos = NULL;
	struct hlist_node* tmp = NULL;
	struct hlist_head* u_head = NULL;

	u_head = &dconfig->dev_info[ifdev].user_head;

	D_USER_SPIN_LOCK();
	if (!HLIST_EMPTY(u_head)) {
		hlist_for_each_entry_safe(pos, tmp, u_head, hlist)
		{
			if (dmap_test(&pos->notify_map) && size < max) {
				size += snprintf(buf + size, max - size,
						"[%u] %-6u:" NOTIFY_MAP_FMT, ifdev, pos->uid,
						NOTIFY_MAP_VALUE((&pos->notify_map)));
			}
		}
	}
	D_USER_SPIN_UNLOCK();

	if (size && size < max)
		size += snprintf(buf + size, max - size, "[%u] %-6u:" NOTIFY_MAP_FMT,
				ifdev, 0,
				NOTIFY_MAP_VALUE((&dconfig->dev_info[ifdev].notify_map)));

	if (size && size < max)
		size += snprintf(buf + size, max - size, "ECN: %-6u:" ,dconfig->dev_info[ifdev].exp_ucnt);

	if (size && size < max)
		size += snprintf(buf + size, max - size, "\n\n");

	return size;
}

static void dops_notify_map_clear(PKT_IFDEV_TYPE ifdev, uid_t uid,
		D_NOTIFY_CODE code, D_NOTIFY_MAP_OPS_TYPE op) {
	struct hlist_node* tmp = NULL;
	struct hlist_head* u_head = NULL;
	d_user* user = NULL, *pos = NULL;

	if (ifdev >= PKT_IFDEV_MAX || code >= D_NOTIFY_CODE_MAX
			|| op < D_NOTIFY_MAP_CLR_ONE_BIT
			|| op > D_NOTIFY_MAP_CLR_FIREWALL_BITS)
		return;

	/* do operation on all online uid */
	if (uid == 0) {
		u_head = &dconfig->dev_info[ifdev].user_head;
		D_USER_SPIN_LOCK();
		if (!HLIST_EMPTY(u_head)) {
			hlist_for_each_entry_safe(pos, tmp, u_head, hlist)
			{
				dops_notify_map_ops(&pos->notify_map, op, code);
				if (!dmap_get(&pos->notify_map)){
					hlist_del_init(&pos->hlist);
					pos->exped = 0;
					if(dconfig->dev_info[ifdev].exp_ucnt>0)
						dconfig->dev_info[ifdev].exp_ucnt--;
				}
			}
		}
		D_USER_SPIN_UNLOCK();

		dops_notify_map_ops(&dconfig->dev_info[ifdev].notify_map, op, code);
	} else {
		if ((user = dops_user_query(ifdev, uid))) {
			dops_notify_map_ops(&user->notify_map, op, code);
			if (!dmap_get(&user->notify_map)){
				dops_user_list_update(user, ifdev, 0);
			}
		}
	}
}

static void dops_notify_map_update(d_res_info* d_res, int count) {
	int ifdev = 0;
	int i = 0, j = 0;
	int code = 0;
	uid_t uid = 0;
	d_user* user = NULL;
	d_res_info* res = NULL;
	D_NOTIFY_MAP_OPS_TYPE op = D_NOTIFY_MAP_SET_ONE_BIT;

	for (i = 0; i < count; i++) {
		res = &d_res[i];
		ifdev = res->ifdev;
		code = res->err_code;
		if (DRES_VALID(res)) {
			for (j = 0; j < res->uid_cnt && j < D_RES_UID_CNT; j++) {
				uid = res->uid[j];
				if ((user = dops_user_query(ifdev, uid))) {
					dops_notify_map_ops(&user->notify_map, op, code);
					dops_user_list_update(user, ifdev, 1);
				}
			}
			switch (res->err_code) {
			case D_NOTIFY_ERR_LOCAL_FAIL:
			case D_NOTIFY_ERR_DNS_NOACK:
			case D_NOTIFY_ERR_DNS_REFUSED:
			case D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP:
			case D_NOTIFY_ERR_INTERNET_OFFLINE:
				dops_notify_map_ops(&dconfig->dev_info[ifdev].notify_map, op,
						code);
				break;

			default:
				break;
			}
		}
	}
}

static void dops_notify_send(d_res_info* d_res, int dcnt, PKT_IFDEV_TYPE ifdev) {
	int index = 0;
	int count = 0;
	d_res_info* ures = dconfig->dev_info[ifdev].ures;

	if (!dconfig->framework_wait || !dcnt || dcnt > D_NOTIFY_CODE_MAX)
		return;

	D_DRES_SPIN_LOCK();
	for (index = 0; index < dcnt; index++) {
		if (DRES_VALID((&d_res[index]))) {
			memcpy(ures, &d_res[index], sizeof(d_res_info));
			count++;
			ures++;
		}
	}
	D_DRES_SPIN_UNLOCK();

	if (!count) {
		return;
	}

	dconfig->dev_info[ifdev].ures_cnt = count;

	switch (d_res[0].err_code) {
	case D_NOTIFY_NET_ACCESS_OK:
	case D_NOTIFY_ERR_TCP_WEB_RESET:
	case D_NOTIFY_ERR_TCP_SELF_RESET:
		break;

	default:
		dops_notify_map_update(d_res, D_NOTIFY_CODE_MAX);
		break;
	}

	LOG_DEBUG("DO NOTIFY");
	//dops_user_print(ifdev);
	dops_config_reset(ifdev);
	dops_user_reset(ifdev);
	wake_up_interruptible(&diagn_waitq);

	return;
}

static bool dops_need_do_stat(PKT_IFDEV_TYPE ifdev, uid_t uid,
		D_NOTIFY_CODE code) {
	int ret = 0;
	d_user* user = NULL;
	D_NOTIFY_MAP_OPS_TYPE op = 0;

	switch (code) {
	case D_NOTIFY_NET_ACCESS_OK:
		/* find gloabl exception,if ok,we should do stat */
		op = D_NOTIFY_MAP_QUERY_GLOBAL_ERR;
		PRINT_NOTIFY_MAP(&dconfig->dev_info[ifdev].notify_map, ifdev, uid);
		ret = dops_notify_map_ops(&dconfig->dev_info[ifdev].notify_map, op, 0);

		/* find special exception,if ok,we should do stat */
		if (!ret && (user = dops_user_query(ifdev, uid))) {
			PRINT_NOTIFY_MAP(&user->notify_map, ifdev, uid);
			op = D_NOTIFY_MAP_QUERY_LOCAL_FIREWALL_ERR;
			ret = dops_notify_map_ops(&user->notify_map, op, 0);
		}
		return ret ? true : false;

	case D_NOTIFY_ERR_TCP_WEB_RESET:
		return true;

	case D_NOTIFY_ERR_DNS_NOACK:
	case D_NOTIFY_ERR_DNS_REFUSED:
	case D_NOTIFY_ERR_DNS_REPLY_GATEWAY_IP:
	case D_NOTIFY_ERR_INTERNET_OFFLINE:
		op = D_NOTIFY_MAP_QUERY_GLOBAL_ERR;
		PRINT_NOTIFY_MAP(&dconfig->dev_info[ifdev].notify_map, ifdev, uid);
		ret = dops_notify_map_ops(&dconfig->dev_info[ifdev].notify_map, op, 0);
		return ret ? false : true;

	default:
		if ((user = dops_user_query(ifdev, uid))) {
			op = D_NOTIFY_MAP_QUERY_INDIVIDUAL;
			PRINT_NOTIFY_MAP(&user->notify_map, ifdev, uid);
			ret = dops_notify_map_ops(&user->notify_map, op, code);
			return ret ? false : true;
		}
		break;
	}
	return true;
}

static void dops_read_sync_flags(d_sync_params* params) {
	unsigned long* flag = &dconfig->stat_flags;
	memset(params, 0, sizeof(d_sync_params));
    unsigned long flags = 0;
	
	D_STAT_SPIN_LOCK(flags);
	params->ifdev =
			__test_and_clear_bit(D_THREAD_SYNC_DEV_WLAN_BIT, flag) ?
					PKT_IFDEV_WLAN : PKT_IFDEV_MOBILE;

	params->dns_nak = __test_and_clear_bit(D_THREAD_SYNC_DNS_IN_BIT, flag);

	params->tcp_syn = __test_and_clear_bit(D_THREAD_SYNC_SYN_IN_BIT, flag);

	params->tcp_rst = __test_and_clear_bit(D_THREAD_SYNC_RST_IN_BIT, flag);

	params->exp_data = __test_and_clear_bit(D_THREAD_SYNC_EXP_IN_BIT, flag);

	D_STAT_SPIN_UNLOCK(flags);
}

static int dops_read_single_data(d_write_block* block, d_sync_params* p,
		int flag) {
	int tmp = 0;
	int* p_cnt = NULL;
	d_stat_pkt* spkt = NULL;
	d_stat_pkt* dpkt = NULL;
	unsigned long flags = 0;
	switch (flag) {
	case D_THREAD_SYNC_SYN_IN_BIT:
		if (p->syn_cnt < D_CPU_COUNT) {
			p_cnt = &p->syn_cnt;
			spkt = &block->s_pkt;
			dpkt = dconfig->s_pkt + p->syn_cnt;
		}
		break;

	case D_THREAD_SYNC_RST_IN_BIT:
		if (p->rst_cnt < D_CPU_COUNT) {
			p_cnt = &p->rst_cnt;
			spkt = &block->r_pkt;
			dpkt = dconfig->r_pkt + p->rst_cnt;
		}
		break;

	case D_THREAD_SYNC_DNS_IN_BIT:
		if (p->dns_cnt < D_CPU_COUNT) {
			p_cnt = &p->dns_cnt;
			spkt = &block->n_pkt;
			dpkt = dconfig->n_pkt + p->dns_cnt;
		}
		break;

	default:
		break;
	}

	if (!spkt || !dpkt)
		return D_INVALID_PARAMS;

	memset(dpkt, 0, SIZE_D_STAT_PKT);
	if (spkt->add_ts) {
		D_NPKT_SPIN_LOCK(block,flags);
		memcpy(dpkt, spkt, SIZE_D_STAT_PKT);
		memset(spkt, 0, SIZE_D_STAT_PKT);
		D_NPKT_SPIN_UNLOCK(block,flags);
		tmp = (*p_cnt) + 1;
		*p_cnt = tmp++;
	}
	return D_SUCCESS;
}

static int dops_read_data_list(d_write_block* block, d_sync_params* p) {
	int index = 0;
	d_stat_pkt* tbuf = dconfig->t_pkt;
	d_stat_pkt* dpkt = NULL, *tmp = NULL;
	struct list_head *whead = &block->write_list;
	unsigned long flags = 0;
	if (!tbuf || !whead) {
		return D_INVALID_PARAMS;
	}

	dconfig->t_pkt_ucnt = 0;
	D_NPKT_SPIN_LOCK(block,flags);
	if (!list_empty(whead)) {
		list_for_each_entry_safe(dpkt,tmp,whead,wlist)
		{
			if (dconfig->t_pkt_ucnt < D_CACHE_NPKT_CNT) {
				memcpy(tbuf, dpkt, SIZE_D_STAT_PKT);
				dconfig->t_pkt_ucnt++;
				tbuf++;
			}
		}
	}
	dpkt_mem_ops(block, D_MEM_OPS_RESET, NULL);
	D_NPKT_SPIN_UNLOCK(block,flags);

	if (dconfig->t_pkt_ucnt) {
		LOG_DEBUG("Add [%d] pkts:C[%d] I[%d] to user\n", dconfig->t_pkt_ucnt,
				block->cpu, p->ifdev);
		tbuf = dconfig->t_pkt;
		for (index = 0; index < dconfig->t_pkt_ucnt; index++) {
			if (dops_user_dpkt_enqueue(&tbuf[index]))
				break;
		}

		memset(dconfig->t_pkt, 0, dconfig->t_pkt_ucnt * SIZE_D_STAT_PKT);
		dconfig->t_pkt_ucnt = 0;
	}
	return D_SUCCESS;
}

static int dops_tryto_send_notify(d_sync_params* p, int flag) {
	int p_cnt = 0;
	int d_cnt = 0;
	int threshold = 0;
	d_stat_pkt* spkt = NULL;
	D_NOTIFY_CODE code = 0;
	d_res_info d_res[D_NOTIFY_CODE_MAX];

	switch (flag) {
	case D_THREAD_SYNC_SYN_IN_BIT:
		p_cnt = p->syn_cnt;
		spkt = dconfig->s_pkt;
		code = D_NOTIFY_NET_ACCESS_OK;
		threshold = dconfig->unreach_trigger_threshold;
		break;

	case D_THREAD_SYNC_RST_IN_BIT:
		p_cnt = p->rst_cnt;
		spkt = dconfig->r_pkt;
		code = D_NOTIFY_ERR_TCP_WEB_RESET;
		threshold = dconfig->unreach_trigger_threshold;
		break;

	case D_THREAD_SYNC_DNS_IN_BIT:
		p_cnt = p->dns_cnt;
		spkt = dconfig->n_pkt;
		code = D_NOTIFY_ERR_DNS_NOACK;
		threshold = dconfig->dnsnack_trigger_threshold;
		break;

	case D_THREAD_SYNC_EXP_IN_BIT:
		d_cnt = 0;
		memset(d_res, 0, sizeof(d_res));
		if ((d_cnt = dops_dres_stat(d_res, p->ifdev))) {
			dops_notify_send(d_res, d_cnt, p->ifdev);
		}
		return D_SUCCESS;

	default:
		break;
	}

	if (!spkt)
		return D_INVALID_PARAMS;

	memset(d_res, 0, sizeof(d_res));
	dops_dres_init(d_res, spkt, p_cnt, code, threshold);
	memset(spkt, 0, dconfig->s_pkt_size);
	dops_notify_send(d_res, 1, p->ifdev);

	return D_SUCCESS;
}

/******************************************************************************
 *
 */
static int dops_stat_thread(void* data) {
	int ret = 0;
	int cpu = 0;
	d_write_block* block = NULL;
	d_sync_params params;

	LOG_ERROR("RealtimeNetworkDiagnos thread started!");

	while (1) {
		do {
			ret = wait_event_interruptible(dconfig->stat_waitq,
					((dconfig->stat_flags & D_WAIT_Q_DATA_IN) != 0));
		} while (ret != 0);

		dmap_set_bit(&dconfig->switch_map, D_SWITCH_BIT_DIAGN_THREAD_ONLINE);
		dops_read_sync_flags(&params);
		LOG_DEBUG("Wake up...syn:%d\trst:%d\texp:%d\tdns:%d\n", params.tcp_syn,
				params.tcp_rst, params.exp_data, params.dns_nak);

		memset(dconfig->n_pkt, 0, dconfig->s_pkt_size);
		memset(dconfig->s_pkt, 0, dconfig->s_pkt_size);
		memset(dconfig->r_pkt, 0, dconfig->s_pkt_size);

		for_each_possible_cpu (cpu)
		{
			block = (d_write_block *) per_cpu_ptr(dwrite_block, cpu);
			if (!block)
				continue;

			if (params.tcp_syn)
				dops_read_single_data(block, &params, D_THREAD_SYNC_SYN_IN_BIT);

			if (params.tcp_rst)
				dops_read_single_data(block, &params, D_THREAD_SYNC_RST_IN_BIT);

			if (params.dns_nak)
				dops_read_single_data(block, &params, D_THREAD_SYNC_DNS_IN_BIT);

			if (params.exp_data) {
				dops_read_data_list(block, &params);
			}
		}

		LOG_ERROR("Processing...syn:%d\trst:%d\texp:%d\tdns:%d\n",
				params.syn_cnt, params.rst_cnt, params.exp_data,
				params.dns_cnt);

		if (params.syn_cnt) {
			dops_tryto_send_notify(&params, D_THREAD_SYNC_SYN_IN_BIT);
		}

		if (params.rst_cnt) {
			dops_tryto_send_notify(&params, D_THREAD_SYNC_RST_IN_BIT);
		}

		if (params.dns_cnt) {
			dops_tryto_send_notify(&params, D_THREAD_SYNC_DNS_IN_BIT);
		}

		if (params.exp_data) {
			dops_tryto_send_notify(&params, D_THREAD_SYNC_EXP_IN_BIT);
		}

		dmap_clear_bit(&dconfig->switch_map, D_SWITCH_BIT_DIAGN_THREAD_ONLINE);
	}
	return ret;
}

static inline void dops_set_dns(void) {
	char buf[128];
	d_dev_info*dev_wlan = &dconfig->dev_info[PKT_IFDEV_WLAN];
	d_dev_info*dev_mobile = &dconfig->dev_info[PKT_IFDEV_MOBILE];
	__be32* dst_wlan = dev_wlan->dns;
	__be32* dst_mobile = dev_mobile->dns;

	memset(buf, 0, sizeof(buf));
	strcpy(buf, D_DEFAULT_DNS_F8);
	buf[strlen(D_DEFAULT_DNS_F8)] = '\0';
	ipv4_strto_addr(buf, dst_wlan);
	ipv4_strto_addr(buf, dst_mobile);

	memset(buf, 0, sizeof(buf));
	strcpy(buf, D_DEFAULT_DNS_F114);
	buf[strlen(D_DEFAULT_DNS_F114)] = '\0';
	ipv4_strto_addr(buf, ++dst_wlan);
	ipv4_strto_addr(buf, ++dst_mobile);

	dev_wlan->dns_cnt = 2;
	dev_mobile->dns_cnt = 2;
}

/******************************************************************************
 *
 */
static bool dnode_params_invalid(const char *buf, size_t count) {
	return (!buf || !count || buf[count] != '\0' || strlen(buf) != count);
}

static int dnode_get_value(const char* str, const char* child) {
	int val = -1;
	const char* pos = NULL;
	if ((pos = strstr(str, child))) {
		pos += strlen(child);
		val = s_atoi(&pos);
	}
	return val;
}

static void dnode_set_ip_list(const char* pos1, const char* pos2, int ifdev,
		D_PARAMS_OPS_TYPE op) {
	u8 bit = 0;
	int len = 0;
	int var = 0;
	int max = 0;
	__be32 ip = 0;

	int* cnt = NULL;
	char tmp[32];
	char* pos = NULL;
	__be32* dst = NULL;
	d_dev_info*dev_info = &dconfig->dev_info[ifdev];

	switch (op) {
	case D_PARAMS_SET_WLAN_GATE_IP:
	case D_PARAMS_SET_MOBILE_GATE_IP:
		dst = &dev_info->gate_ipv4;
		break;

	case D_PARAMS_SET_MOBILE_DNS:
	case D_PARAMS_SET_WLAN_DNS:
		max = D_CACHE_DNS_CNT;
		dst = dev_info->dns;
		cnt = &dev_info->dns_cnt;
		break;

	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		max = D_CACHE_TCP_SRC_BLACK_LIST;
		dst = dev_info->tcp_src_black_list;
		cnt = &dev_info->tcp_src_black_list_cnt;
		break;

	default:
		return;
	}

	if (!dst)
		return;

	memset(tmp, 0, sizeof(tmp));

	switch (op) {
	case D_PARAMS_SET_WLAN_GATE_IP:
	case D_PARAMS_SET_MOBILE_GATE_IP:
		len = pos2 - pos1;
		if (!DOPS_VALID_IPV4_S_LEN(len))
			return;

		ip = 0;
		memset(tmp, 0, sizeof(tmp));
		memcpy(tmp, pos1, len);
		tmp[len] = '\0';
		if (ipv4_strto_addr(tmp, &ip) && !is_addr_equal(&ip, dst, AF_INET)) {
			memcpy(dst, &ip, SIZE_IPV4_BE32);

			if (op == D_PARAMS_SET_WLAN_GATE_IP) {
				bit = D_SWITCH_BIT_GATE_IP_INITED_WLAN;
			} else {
				bit = D_SWITCH_BIT_GATE_IP_INITED_MOBILE;
			}

			dops_user_reset(ifdev);
			dops_config_reset(ifdev);
			dmap_set_bit(&dconfig->switch_map, bit);

			LOG_DEBUG("Set %s ipv4 success\n",
					op == D_PARAMS_SET_WLAN_GATE_IP ? "wlan" : "mobile");
		}
		break;

	case D_PARAMS_SET_WLAN_DNS:
	case D_PARAMS_SET_MOBILE_DNS:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		if (!cnt)
			return;

		var = *cnt;
		while (pos1 < pos2 && (pos = strchr(pos1, ',')) && (len = pos - pos1)
				&& DOPS_VALID_IPV4_S_LEN(len) && var < max) {
			ip = 0;
			memset(tmp, 0, sizeof(tmp));
			memcpy(tmp, pos1, len);
			tmp[len] = '\0';
			if (ipv4_strto_addr(tmp, &ip) && !is_in_set(dst, var, ip)) {
				memcpy(dst + (var++), &ip, SIZE_IPV4_BE32);
			}
			pos1 = pos + 1;
		}
		if (var > *cnt)
			LOG_DEBUG("Set [%d] ipv4 success\n", op);

		*cnt = var;
		break;
	default:
		return;
	}
}

static int dops_dump_ip_list(char* buf, int max_size, D_PARAMS_OPS_TYPE op) {
	int size = 0;
	int max = 0;
	int index = 0;
	char tmp[64];
	__be32 * dst = NULL;
	d_dev_info*dev_info = NULL;

	memset(tmp, 0, sizeof(tmp));
	switch (op) {
	case D_PARAMS_SET_WLAN_DNS:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
		dev_info = &dconfig->dev_info[PKT_IFDEV_WLAN];
		break;

	case D_PARAMS_SET_MOBILE_DNS:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		dev_info = &dconfig->dev_info[PKT_IFDEV_MOBILE];
		break;

	default:
		return 0;
	}

	if (!dev_info)
		return 0;

	switch (op) {
	case D_PARAMS_SET_WLAN_DNS:
	case D_PARAMS_SET_MOBILE_DNS:
		dst = dev_info->dns;
		max = dev_info->dns_cnt;
		break;

	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		dst = dev_info->tcp_src_black_list;
		max = dev_info->tcp_src_black_list_cnt;
		break;

	default:
		return 0;
	}

	if (!max || !dst)
		return 0;

	switch (op) {
	case D_PARAMS_SET_WLAN_DNS:
		strcpy(tmp, D_PARAMS_SHOW_DNS_LIST_WLAN);
		tmp[strlen(D_PARAMS_SHOW_DNS_LIST_WLAN)] = '\0';
		break;

	case D_PARAMS_SET_MOBILE_DNS:
		strcpy(tmp, D_PARAMS_SHOW_DNS_LIST_MOBILE);
		tmp[strlen(D_PARAMS_SHOW_DNS_LIST_MOBILE)] = '\0';
		break;

	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
		strcpy(tmp, D_PARAMS_SHOW_TCPSRC_BLIST_WLAN);
		tmp[strlen(D_PARAMS_SHOW_TCPSRC_BLIST_WLAN)] = '\0';
		break;

	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		strcpy(tmp, D_PARAMS_SHOW_TCPSRC_BLIST_MOBILE);
		tmp[strlen(D_PARAMS_SHOW_TCPSRC_BLIST_MOBILE)] = '\0';
		break;

	default:
		return 0;
	}

	if (size < max_size && max)
		size += snprintf(buf + size, max_size - size, "%s", tmp);

	for (index = 0; index < max; index++) {
		if (size < max_size)
			size += snprintf(buf + size, max_size - size, IP_V4_FMT ",",
					IP_V4_ARY(dst[index]));
	}

	if (size < max_size)
		size += snprintf(buf + size, max_size - size, "\n");

	return size;
}

/*
 fw=/sys/class/rtnetds_cls/netdiagn/params;
 # 1.set loglevel
 echo "opcode:1,value {3}">$fw;
 echo "opcode:1,value {1}">$fw;
 
 # 2.set wlan name
 echo "opcode:2,value {wlan0}" >$fw;

 # 3.set mobile name
 echo "opcode:3,value {ccmni0}" >$fw;

 # 4.set ipv4 address of wlan ap router
 echo "opcode:4,value {192.168.1.1}" >$fw;

 # 5.set ipv4 address of mobile ap router
 echo "opcode:5,value {10.112.110.10}" >$fw;
 
 # 6.set threshold for update interval and trgger notify
 echo "opcode:6,value {ui.u=8,ui.d=8,nt.u=4,nt.d=5}" >$fw;

 # 7.set pkt number threshold for wakeup stat thread
 echo "opcode:7,value {pw.t.u=5,pw.t.d=1,pw.t.syn=1,pw.t.drst=1,pw.t.srst=1}" >$fw;
 
 # 30
 echo "opcode:30,value {8.8.8.8,114.114.114.114,}" >$fw;

 # 31
 echo "opcode:31,value {8.8.8.8,114.114.114.114,}" >$fw;

 # 32
 echo "opcode:32,value {1.1.1.1,2.2.2.2,129.129.129.124,}" >$fw;

 # 33
 echo "opcode:33,value {1.1.1.1,2.2.2.2,129.129.129.124,}" >$fw;

 # 40
 echo "opcode:40,value {nopCode=2,ifdev=0,uid=0,errCode=3}" >$fw;

 # 41
 echo "opcode:41,value {4}" >$fw;
 echo "opcode:41,value {5}" >$fw;
 echo "opcode:41,value {30}" >$fw;
 echo "opcode:41,value {31}" >$fw;
 echo "opcode:41,value {32}" >$fw;
 echo "opcode:41,value {33}" >$fw;

 # 50.1.disable rtnds
 echo "opcode:50,value {1}" >$fw;

 # 50.2.enable rtnds
 echo "opcode:50,value {0}" >$fw;

 # 51.1.framework is doing http diagn
 echo "opcode:51,value {1}" >$fw;

 # 51.2.framework http diagn done
 echo "opcode:51,value {0}" >$fw;

 # 52.1.framework disable global exception for mobile
 echo "opcode:52,value {1}" >$fw;

 # 52.2.framework enable global exception for mobile
 echo "opcode:52,value {0}" >$fw;

 # 53.1.framework enable unreachable from internet
 echo "opcode:52,value {1}" >$fw;

 # 53.2.framework disable  unreachable from internet
 echo "opcode:52,value {0}" >$fw;

 cat $fw;
 */
static ssize_t dnode_params_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int len = 0;
	int val = -1;
	u8 valid = 0;
	uid_t uid = 0;

	const char* pos = NULL;
	const char* pos1 = NULL;
	const char* pos2 = NULL;
	PKT_IFDEV_TYPE ifdev = 0;
	D_NOTIFY_CODE err_code = 0;
	D_NOTIFY_MAP_OPS_TYPE nop = 0;

	int opcode = D_PARAMS_OPS_INVALID;
	if (dnode_params_invalid(buf, count)) {
		LOG_ERROR("Invalid params\n");
		return count;
	}

	LOG_DEBUG("Rcv: %s\n", buf);
	opcode = dnode_get_value(buf, D_PARAMS_OP_CODE);
	if (!DOPS_VALID_PARAMS_OPCODE(opcode)) {
		LOG_ERROR("Invalid op\n");
		return count;
	}

	if ((pos = strstr(buf, D_PARAMS_VALUE_START))) {
		pos += strlen(D_PARAMS_VALUE_START);
		pos1 = pos;
	} else {
		LOG_ERROR("Invalid start\n");
		return count;
	}

	if ((pos = strchr(buf, D_PARAMS_VALUE_END))
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
	case D_PARAMS_SET_LOG_LEVEL:
		if ((val = s_atoi(&pos1)) > 0) {
			dconfig->log_level = val;
		}
		break;

	case D_PARAMS_SET_WLAN_NAME:
		if (DOPS_VALID_INTF_NAME_SIZE(len)
				&& !strncmp(pos1, D_INTF_NAME_WLAN, len)) {
			if (set_ndev_ifaddr(PKT_IFDEV_WLAN) == D_SUCCESS) {
				dmap_set_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN);
				LOG_DEBUG("Set wlan success\n");
			} else {
				dmap_clear_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN);
			}
		}
		break;

	case D_PARAMS_SET_MOBILE_NAME:
		if (DOPS_VALID_INTF_NAME_SIZE(len)) {
			memset(dconfig->mobile_name, 0, D_INTF_NAME_SIZE);
			memcpy(dconfig->mobile_name, pos1, len);
			dconfig->mobile_name[len + 1] = '\0';
			if (set_ndev_ifaddr(PKT_IFDEV_MOBILE) == D_SUCCESS) {
				dmap_set_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE);
				LOG_DEBUG("Set mobile success\n");
			} else {
				dmap_clear_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE);
			}
		}
		break;

	case D_PARAMS_SET_WLAN_GATE_IP:
		dnode_set_ip_list(pos1, pos2, PKT_IFDEV_WLAN, opcode);
		break;

	case D_PARAMS_SET_MOBILE_GATE_IP:
		dnode_set_ip_list(pos1, pos2, PKT_IFDEV_MOBILE, opcode);
		break;

	case D_PARAMS_SET_STAT_THRESHOLD_INTERVAL:
		val = dnode_get_value(pos1, D_PARAMS_UPDATE_INTERVAL_UNREACH);
		if (DOPS_VALID_STAT_THRESHOLD(val))
			dconfig->unreach_update_interval = val * 1000;

		val = dnode_get_value(pos1, D_PARAMS_UPDATE_INTERVAL_DNSNACK);
		if (DOPS_VALID_STAT_THRESHOLD(val))
			dconfig->dnsnack_update_interval = val * 1000;

		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_THRESHOLD_UNREACH);
		if (DOPS_VALID_STAT_THRESHOLD(val))
			dconfig->unreach_trigger_threshold = val;

		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_THRESHOLD_DNSNACK);
		if (DOPS_VALID_STAT_THRESHOLD(val))
			dconfig->dnsnack_trigger_threshold = val;
		break;

	case D_PARAMS_SET_PKT_WAKEUP_THRESHOLD:
		val = dnode_get_value(pos1, D_PARAMS_PKT_WAKEUP_THRESHOLD_UNREACH);
		if (DOPS_VALID_WAKE_THRESHOLD(val))
			dconfig->wake_threshold_unreach = val;

		val = dnode_get_value(pos1, D_PARAMS_PKT_WAKEUP_THRESHOLD_DNSNACK);
		if (DOPS_VALID_WAKE_THRESHOLD(val))
			dconfig->wake_threshold_dnsnack = val;

		val = dnode_get_value(pos1, D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPSYNC);
		if (DOPS_VALID_WAKE_THRESHOLD(val))
			dconfig->wake_threshold_tcpsync = val;

		val = dnode_get_value(pos1, D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPDRST);
		if (DOPS_VALID_WAKE_THRESHOLD(val))
			dconfig->wake_threshold_tcpdrst = val;

		val = dnode_get_value(pos1, D_PARAMS_PKT_WAKEUP_THRESHOLD_TCPSRST);
		if (DOPS_VALID_WAKE_THRESHOLD(val))
			dconfig->wake_threshold_tcpsrst = val;

		break;

	case D_PARAMS_SET_WLAN_DNS:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
		dnode_set_ip_list(pos1, pos2, PKT_IFDEV_WLAN, opcode);
		break;

	case D_PARAMS_SET_MOBILE_DNS:
	case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
		dnode_set_ip_list(pos1, pos2, PKT_IFDEV_MOBILE, opcode);
		break;

	case D_PARAMS_CLEAR_NOTIFYMAP:
		valid = 0;
		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_OPCODE);
		if (DOPS_VALID_NOTIFY_OP_CODE(val)) {
			nop = val;
			valid++;
		}

		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_IFDEV);
		if (DOPS_VALID_NOTIFY_IFDEV(val)) {
			ifdev = val;
			valid++;
		}

		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_UID);
		if (val >= 0) {
			uid = val;
			valid++;
		}

		val = dnode_get_value(pos1, D_PARAMS_NOTIFY_ECODE);
		if (DOPS_VALID_NOTIFY_ECODE(val)) {
			err_code = val;
			valid++;
		}

		if (valid == 4) {
			LOG_DEBUG("Restore notify map,[IF:%u,U:%u,C:%u,O:%u]\n", ifdev, uid,
					err_code, nop);
			dops_notify_map_clear(ifdev, uid, err_code, nop);
		}
		break;

	case D_PARAMS_CLEAR_ANY_SET:
		val = s_atoi(&pos1);
		if (DOPS_VALID_PARAMS_OPCODE(val)) {
			switch (val) {
			case D_PARAMS_SET_WLAN_GATE_IP:
				dmap_clear_bit(&dconfig->switch_map,
						D_SWITCH_BIT_GATE_IP_INITED_WLAN);
				memset(&dconfig->dev_info[PKT_IFDEV_WLAN].gate_ipv4, 0,
						SIZE_IPV4_BE32);
				dops_user_reset(PKT_IFDEV_WLAN);
				dops_config_reset(PKT_IFDEV_WLAN);
				break;

			case D_PARAMS_SET_MOBILE_GATE_IP:
				dmap_clear_bit(&dconfig->switch_map,
						D_SWITCH_BIT_GATE_IP_INITED_MOBILE);
				memset(&dconfig->dev_info[PKT_IFDEV_MOBILE].gate_ipv4, 0,
						SIZE_IPV4_BE32);
				dops_user_reset(PKT_IFDEV_MOBILE);
				dops_config_reset(PKT_IFDEV_MOBILE);
				break;

			case D_PARAMS_SET_WLAN_DNS:
				memset(dconfig->dev_info[PKT_IFDEV_WLAN].dns, 0,
						SIZE_IPV4_BE32 * D_CACHE_DNS_CNT);
				break;

			case D_PARAMS_SET_MOBILE_DNS:
				memset(dconfig->dev_info[PKT_IFDEV_MOBILE].dns, 0,
						SIZE_IPV4_BE32 * D_CACHE_DNS_CNT);
				break;

			case D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN:
				memset(dconfig->dev_info[PKT_IFDEV_WLAN].tcp_src_black_list, 0,
						SIZE_IPV4_BE32 * D_CACHE_TCP_SRC_BLACK_LIST);
				break;

			case D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE:
				memset(dconfig->dev_info[PKT_IFDEV_MOBILE].tcp_src_black_list,
						0, SIZE_IPV4_BE32 * D_CACHE_TCP_SRC_BLACK_LIST);
				break;

			default:
				break;
			}
		}
		break;

	case D_PARAMS_SET_FRAMEWORK_DISABLED:
		DOPS_SET_SWITCH(pos1, val, D_SWITCH_BIT_FRAMEWORK_DISABLED);
		break;

	case D_PARAMS_SET_FRAMEWORK_HTTPDIAGN_ONLINE:
		DOPS_SET_SWITCH(pos1, val, D_SWITCH_BIT_FRAMEWORK_HTTP_DIAGN);
		break;

	case D_PARAMS_SET_MOBILE_GLOBAL_DISABLED:
		DOPS_SET_SWITCH(pos1, val, D_SWITCH_BIT_MOBILE_GLOBAL_DISABLED);
		break;

	case D_PARAMS_SET_UNREACH_SRC_INTERNET_ENABLED:
		DOPS_SET_SWITCH(pos1, val, D_SWITCH_BIT_UNREACH_SRC_INTERNET_ENABLED);
		break;

	case D_PARAMS_SET_FRAMEWORK_ENABLE_PPPOE:
		DOPS_SET_SWITCH(pos1, val, D_SWITCH_BIT_FRAMEWORK_ENABLE_PPPOE);
		break;

	default:
		return count;
	}

	return count;
}

static ssize_t dnode_log_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count) {
	int val = -1;
	const char* pos = NULL;
	const char* pos1 = NULL;

	if ((pos = strstr(buf, D_PARAMS_LOG_LEVEL))) {
		pos += strlen(D_PARAMS_LOG_LEVEL);
		pos1 = pos;
	} else {
		LOG_ERROR("Invalid start\n");
		return count;
	}

	if ((val = s_atoi(&pos1)) > 0 && D_MSG_EXCE <= val && val <= D_MSG_ERROR) {
		dconfig->log_level = val;
	}
	return count;
}

static ssize_t dnode_params_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int size = 0;

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_LOG_LEVEL, dconfig->log_level);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = " IP_V4_FMT "\n",
		D_PARAMS_SHOW_GATE_IP_WLAN,
				IP_V4_ARY(dconfig->dev_info[PKT_IFDEV_WLAN].gate_ipv4));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %s\n",
		D_PARAMS_SHOW_INTF_NAME_MOBILE, dconfig->mobile_name);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_NEDV_IFADDR_INITED_WLAN,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_WLAN));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_NEDV_IFADDR_INITED_MOBILE,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_NDEV_IFADR_INITED_MOBILE));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = " IP_V4_FMT "\n",
		D_PARAMS_SHOW_GATE_IP_MOBILE,
				IP_V4_ARY(dconfig->dev_info[PKT_IFDEV_MOBILE].gate_ipv4));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_FRAMEWORK_DISABLED,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_FRAMEWORK_DISABLED));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_FRAMEWORK_HTTPDIAGN,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_FRAMEWORK_HTTP_DIAGN));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_MOBILE_GLOBAL_DISABLED,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_MOBILE_GLOBAL_DISABLED));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_UNREACH_SRC_INTERNET_ENABLED,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_UNREACH_SRC_INTERNET_ENABLED));

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %u\n",
		D_PARAMS_SHOW_FRAMEWORK_ENABLE_PPPOE,
				dmap_test_bit(&dconfig->switch_map,
						D_SWITCH_BIT_FRAMEWORK_ENABLE_PPPOE));

	/* update.interval */
	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d ms\n",
		D_PARAMS_SHOW_UPDATE_INTERVAL_UNREACH,
				dconfig->unreach_update_interval);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d ms\n",
		D_PARAMS_SHOW_UPDATE_INTERVAL_DNSNACK,
				dconfig->dnsnack_update_interval);

	/* notify.threshold */
	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_NOTIFY_THRESHOLD_UNREACH,
				dconfig->unreach_trigger_threshold);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n\n",
		D_PARAMS_SHOW_NOTIFY_THRESHOLD_DNSNACK,
				dconfig->dnsnack_trigger_threshold);

	/* pktwakeup.threshold */
	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_UNREACH,
				dconfig->wake_threshold_unreach);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_DNSNACK,
				dconfig->wake_threshold_dnsnack);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPSYNC,
				dconfig->wake_threshold_tcpsync);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPDRST,
				dconfig->wake_threshold_tcpdrst);

	if (size < D_RBUF_ATR_SIZE)
		size += snprintf(buf + size, D_RBUF_ATR_SIZE - size,
		D_LOG_TAG "%s = %d\n",
		D_PARAMS_SHOW_PKT_WAKEUP_THRESHOLD_TCPSRST,
				dconfig->wake_threshold_tcpsrst);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_dump_ip_list(buf + size, D_RBUF_ATR_SIZE - size,
				D_PARAMS_SET_WLAN_DNS);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_dump_ip_list(buf + size, D_RBUF_ATR_SIZE - size,
				D_PARAMS_SET_MOBILE_DNS);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_dump_ip_list(buf + size, D_RBUF_ATR_SIZE - size,
				D_PARAMS_SET_TCPSRC_BLACK_LIST_WLAN);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_dump_ip_list(buf + size, D_RBUF_ATR_SIZE - size,
				D_PARAMS_SET_TCPSRC_BLACK_LIST_MOBILE);

	return size;
}

static ssize_t dnode_wlan_rxts_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int size = 0;

	size += snprintf(buf, D_RBUF_ATR_SIZE - size,
	D_LOG_TAG "wlan last rx timestamp: %u ms\n",
			GET_MS(get_lastrx_of_netdev(PKT_IFDEV_WLAN)));
	return size;
}

static ssize_t dnode_mobile_rxts_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int size = 0;

	size += snprintf(buf, D_RBUF_ATR_SIZE - size,
	D_LOG_TAG "mobile last rx timestamp: %u ms\n",
			GET_MS(get_lastrx_of_netdev(PKT_IFDEV_MOBILE)));
	return size;
}

static ssize_t dnode_mem_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int size = 0;

	if (size < D_RBUF_ATR_SIZE)
		size += dops_mem_stat_dump(buf + size, D_RBUF_ATR_SIZE - size,
				PKT_IFDEV_WLAN);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_mem_stat_dump(buf + size, D_RBUF_ATR_SIZE - size,
				PKT_IFDEV_MOBILE);

	return size;
}

/*
 adb shell cat /sys/class/rtnetds_cls/netdiagn/notify;
 */
static ssize_t dnode_notify_show(struct device *dev,
		struct device_attribute *attr, char *buf) {
	int size = 0;

	if (size < D_RBUF_ATR_SIZE)
		size += dops_notify_map_dump(buf + size, D_RBUF_ATR_SIZE - size,
				PKT_IFDEV_WLAN);

	if (size < D_RBUF_ATR_SIZE)
		size += dops_notify_map_dump(buf + size, D_RBUF_ATR_SIZE - size,
				PKT_IFDEV_MOBILE);

	return size;
}

static ssize_t dnode_log_show(struct device *dev, struct device_attribute *attr,
		char *buf) {
	int size = 0;

	size += snprintf(buf, D_RBUF_ATR_SIZE - size,
	D_LOG_TAG D_PARAMS_LOG_LEVEL "%u\n", dconfig->log_level);
	return size;
}

static struct device_attribute diagn_device_attrs[] = { /* attribute for user */
__ATTR(params, 0660, dnode_params_show, dnode_params_store), /* get\set parameters */
__ATTR(notify, 0440, dnode_notify_show, NULL), /*	 get notify map */
__ATTR(wrxts, 0440, dnode_wlan_rxts_show, NULL), /*	get last rx timestamp of wlan */
__ATTR(mrxts, 0440, dnode_mobile_rxts_show, NULL), /*	get last rx timestamp mobile */
__ATTR(mem, 0440, dnode_mem_show, NULL), /* mem for all processed uid */
__ATTR(log, 0660, dnode_log_show, dnode_log_store), /* log settings */
__ATTR_NULL, /* end */
};

static int diagn_device_file_create(struct device* dev) {
	int ret = 0;
	struct device_attribute* attr = NULL;
	for (attr = diagn_device_attrs; attr->attr.name; attr++) {
		ret = device_create_file(dev, attr);
		if (ret) {
			return ret;
		}
	}
	return ret;
}

static void diagn_device_file_remove(struct device* dev) {
	struct device_attribute* attr = NULL;
	for (attr = diagn_device_attrs; attr->attr.name; attr++) {
		device_remove_file(dev, attr);
	}
}

static int netdiagn_dev_open(struct inode * inode, struct file * filp) {
	return 0;
}

static int netdiagn_dev_close(struct inode *inode, struct file *file) {
	return 0;
}

static ssize_t netdiagn_dev_read(struct file *file, char __user *user,
		size_t size, loff_t *ppos) {
	int read = 0;
	char buf[D_RBUF_DEV_SIZE];
	if (size < D_RBUF_DEV_SIZE)
		return -EINVAL;

	memset(buf, 0, D_RBUF_DEV_SIZE);
	if ((read = dops_dres_read(buf, D_RBUF_DEV_SIZE - 1))
			&& read <= D_RBUF_DEV_SIZE - 1) {
		buf[read] = '\0';
		if (!copy_to_user(user, buf, read)) {
			LOG_DEBUG(buf);
			return read;
		}
	}
	return 0;
}

static unsigned int netdiagn_dev_poll(struct file *file, poll_table *wait) {
	unsigned int mask = 0;

	dconfig->framework_wait = 1;
	poll_wait(file, &diagn_waitq, wait);

	if (dops_dres_ready() > 0) {
		dconfig->framework_wait = 0;
		mask |= POLLIN | POLLRDNORM;
	}
	return mask;
}

static const struct file_operations diagn_dev_fops = { /*  */
.owner = THIS_MODULE, /*  */
.open = netdiagn_dev_open, /*  */
.read = netdiagn_dev_read, /*  */
.release = netdiagn_dev_close, /*  */
.poll = netdiagn_dev_poll, /*  */
};

/******************************************************************************
 *
 */
static inline int dops_static_init(void) {
	int cpu = 0;
	int index = 0;
	d_write_block* block = NULL;

	if (!dconfig || dwrite_block)
		return -1;

	/* 1. static fields init */
	memset(dconfig, 0, sizeof(d_config));
	dconfig->log_level = D_MSG_ERROR;
	dconfig->unreach_update_interval = 8000;
	dconfig->dnsnack_update_interval = 20000;
	dconfig->unreach_trigger_threshold = 4;
	dconfig->dnsnack_trigger_threshold = 5;

	dconfig->wake_threshold_unreach = 4;
	dconfig->wake_threshold_dnsnack = 1;
	dconfig->wake_threshold_tcpsync = 1;
	dconfig->wake_threshold_tcpdrst = 1;
	dconfig->wake_threshold_tcpsrst = 1;

	strcpy(dconfig->mobile_name, D_INTF_NAME_MOB);
	dconfig->mob_ifn_size = strlen(D_INTF_NAME_MOB);

	dconfig->d_pkt_size = sizeof(d_stat_pkt) * D_CACHE_NPKT_CNT * D_CPU_COUNT;
	dconfig->s_pkt_size = sizeof(d_stat_pkt) * D_CPU_COUNT;
	dconfig->t_pkt_size = sizeof(d_stat_pkt) * D_CACHE_NPKT_CNT;
	dconfig->user_size = sizeof(d_user_slot) * D_CACHE_USLOT_CNT;

	dops_set_dns();

	LOG_ERROR(
			"Try to alloc memery:\n"
			D_LOG_TAG "DPKT :%d ( %lu * %d )\n"
			D_LOG_TAG "NPKT :%d ( %lu * %d )\n"
			D_LOG_TAG "SPKT :%d ( %lu * %d )\n"
			D_LOG_TAG "RPKT :%d ( %lu * %d )\n"
			D_LOG_TAG "TPKT :%d ( %lu * %d )\n"
			D_LOG_TAG "USER :%d ( %lu * %d )\n"
			D_LOG_TAG "BLOCK :%lu ( %lu * %d )\n\n",
			dconfig->d_pkt_size, sizeof(d_stat_pkt),
			(D_CACHE_NPKT_CNT * D_CPU_COUNT),/* d_pkt */
			dconfig->s_pkt_size, sizeof(d_stat_pkt), D_CPU_COUNT,/* n_pkt */
			dconfig->s_pkt_size, sizeof(d_stat_pkt), D_CPU_COUNT,/* s_pkt */
			dconfig->s_pkt_size, sizeof(d_stat_pkt), D_CPU_COUNT,/* r_pkt */
			dconfig->t_pkt_size, sizeof(d_stat_pkt), D_CACHE_NPKT_CNT,/* t_pkt */
			dconfig->user_size, sizeof(d_user_slot), D_CACHE_USLOT_CNT,/* user_slot */
			sizeof(d_write_block)*D_CPU_COUNT, sizeof(d_write_block),
			D_CPU_COUNT/*dwrite_block*/);

	/* 2.alloc dynamic memory */
	/* used for multi thread on percpu*/
	if ((dconfig->d_pkt = (d_stat_pkt*) kmalloc(dconfig->d_pkt_size, GFP_ATOMIC))
			== NULL) {
		return -ENOMEM;
	}

	if ((dconfig->n_pkt = (d_stat_pkt*) kmalloc(dconfig->s_pkt_size, GFP_ATOMIC))
			== NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		return -ENOMEM;
	}

	/* used for stat thread*/
	if ((dconfig->s_pkt = (d_stat_pkt*) kmalloc(dconfig->s_pkt_size, GFP_ATOMIC))
			== NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		DOPS_KFREE(dconfig->n_pkt);
		return -ENOMEM;
	}

	/* used for stat thread*/
	if ((dconfig->r_pkt = (d_stat_pkt*) kmalloc(dconfig->s_pkt_size, GFP_ATOMIC))
			== NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		DOPS_KFREE(dconfig->n_pkt);
		DOPS_KFREE(dconfig->s_pkt);
		return -ENOMEM;
	}

	/* used for stat thread*/
	if ((dconfig->t_pkt = (d_stat_pkt*) kmalloc(dconfig->t_pkt_size, GFP_ATOMIC))
			== NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		DOPS_KFREE(dconfig->n_pkt);
		DOPS_KFREE(dconfig->s_pkt);
		DOPS_KFREE(dconfig->r_pkt);
		return -ENOMEM;
	}

	/* used for stat thread,support wlan and mobile interface */
	if ((dconfig->user_slot = (d_user_slot*) kmalloc(dconfig->user_size,
	GFP_ATOMIC)) == NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		DOPS_KFREE(dconfig->n_pkt);
		DOPS_KFREE(dconfig->s_pkt);
		DOPS_KFREE(dconfig->r_pkt);
		DOPS_KFREE(dconfig->t_pkt);
		return -ENOMEM;
	}

	/* used for writer thread on percpu */
	if ((dwrite_block = alloc_percpu(d_write_block)) == NULL) {
		DOPS_KFREE(dconfig->d_pkt);
		DOPS_KFREE(dconfig->n_pkt);
		DOPS_KFREE(dconfig->s_pkt);
		DOPS_KFREE(dconfig->r_pkt);
		DOPS_KFREE(dconfig->t_pkt);
		DOPS_KFREE(dconfig->user_slot);
		return -ENOMEM;
	}

	/* 3.clear all dynamic memory */
	memset(dconfig->d_pkt, 0, dconfig->d_pkt_size);
	memset(dconfig->n_pkt, 0, dconfig->s_pkt_size);
	memset(dconfig->s_pkt, 0, dconfig->s_pkt_size);
	memset(dconfig->r_pkt, 0, dconfig->s_pkt_size);
	memset(dconfig->t_pkt, 0, dconfig->t_pkt_size);
	memset(dconfig->user_slot, 0, dconfig->user_size);

	/* 4.init dwrite_block percpu */
	for_each_possible_cpu (cpu)
	{
		block = ((d_write_block *) per_cpu_ptr(dwrite_block, cpu));
		if (block && index < D_CPU_COUNT) {
			memset(block, 0, sizeof(d_write_block));
			block->cpu = cpu;
			block->count = D_CACHE_NPKT_CNT;
			block->d_pkt = dconfig->d_pkt + index * D_CACHE_NPKT_CNT;
			dpkt_mem_ops(block, D_MEM_OPS_RESET, NULL);
			spin_lock_init(&block->npkt_lock);
			index++;
		}
	}

	/* 5.init spinlock and waitqueue for stat thread */
	spin_lock_init(&dconfig->dres_lock);
	spin_lock_init(&dconfig->stat_lock);
	spin_lock_init(&dconfig->user_lock);

	init_waitqueue_head(&dconfig->stat_waitq);
	INIT_HLIST_HEAD(&dconfig->dev_info[PKT_IFDEV_WLAN].user_head);
	INIT_HLIST_HEAD(&dconfig->dev_info[PKT_IFDEV_MOBILE].user_head);

	/* 6.start stat thread */
	dconfig->stat_thread = kthread_run(dops_stat_thread, NULL, "pkt_thread");

	if (dconfig->stat_thread) {
		dmap_set_bit(&dconfig->switch_map, D_SWITCH_BIT_MEM_INITED);
		return D_SUCCESS;
	} else {
		return D_THREAD_START_FAIL;
	}
}

void dops_static_deinit(void) {
	if (dwrite_block) {
		free_percpu(dwrite_block);
		dwrite_block = NULL;
	}

	DOPS_KFREE(dconfig->d_pkt);
	DOPS_KFREE(dconfig->n_pkt);
	DOPS_KFREE(dconfig->s_pkt);
	DOPS_KFREE(dconfig->r_pkt);
	DOPS_KFREE(dconfig->t_pkt);
	DOPS_KFREE(dconfig->user_slot);

	LOG_DEBUG("dblock deinited done.");
}

static int __init netdiagn_init(void) {
	if (dops_static_init() != 0) {
		LOG_ERROR("fail to init dblock");
		return -1;
	}

	major = register_chrdev(0, DIAGN_DEV_NAME, &diagn_dev_fops);
	if (major < 0) {
		LOG_ERROR("fail to register chrdev");
		return 0;
	}
	diagn_class = class_create(THIS_MODULE, DIAGN_CLASS_NAME);
	if (!diagn_class) {
		LOG_ERROR("fail to create class");
		goto fail1;
	}

	diagn_device = device_create(diagn_class, NULL, MKDEV(major, 0), NULL,DIAGN_DEVICE_NAME);
	if (!diagn_device) {
		LOG_ERROR("fail to create device");
		goto fail2;
	}

	if (diagn_device_file_create(diagn_device)) {
		LOG_ERROR("fail to create device attr file");
		goto fail3;
	}

	dmap_set_bit(&dconfig->switch_map,D_SWITCH_BIT_DRIVER_INITED);
	LOG_DEBUG("driver inited successfully.");
	return 0;

	LOG_DEBUG("driver inited fail !!");
fail3:

	diagn_device_file_remove(diagn_device);
	device_unregister(diagn_device);

fail2:
	class_destroy(diagn_class);

fail1:
	unregister_chrdev(major, DIAGN_DEV_NAME);

	dops_static_deinit();

	return -1;
}

static void __exit netdiagn_exit(void) {
	diagn_device_file_remove(diagn_device);
	device_unregister(diagn_device);
	class_destroy(diagn_class);
	unregister_chrdev(major, DIAGN_DEV_NAME);
	dops_static_deinit();
	LOG_DEBUG("driver deinited successfully.");
}

module_init(netdiagn_init);
module_exit(netdiagn_exit);
EXPORT_SYMBOL(net_pkt_analysis);
MODULE_ALIAS_NETPROTO(netdiagn);
MODULE_DESCRIPTION("Net core diagn");
MODULE_LICENSE("GPL");

#else
void net_pkt_analysis(struct sock*sk, struct sk_buff* skb)
{
	return ;
}
#endif
