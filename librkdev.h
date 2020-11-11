#ifndef LIBRKDEV_H
#define LIBRKDEV_H

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdbool.h>
#include <syslog.h>
#include <stdarg.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C"
{
#endif

int libdevctrl_init();
int libdevctrl_uninit();
void DictPen_log(int prio, const char *fmt, ...);

bool isIPValid(char *ipStr);
int _console_run(char *cmd , char *reply);

/********************************************************************/

#define MONO 1
#define STEREO 2

typedef enum{
    DEV_PLAYBACK = 0,
    DEV_RECORD = 1,
    DEV_MIXER = 2
}audio_dev_t;

typedef struct {
    unsigned int rate; // 8000, 16000, 44100, 48000, etc
    unsigned int chan; // 1 for Mono ,2 for stereo
}audio_info_t;

int audio_init(void **handle , audio_dev_t dev_type , audio_info_t *audio_info);
int audio_uninit(void *handle, audio_dev_t dev_type);

int audio_play(void *playback_handle ,char *pcm_data_buff , int buff_len);
int audio_play_stop(void *playback_handle);
int audio_record(void * record_handle, char * pcm_data_buff, int * buff_len, int sec_num);
int audio_record_stop(void * record_handle);

int audio_set_speaker_volume(void *mixer_handle , int volume);
int audio_get_speaker_volume(void *mixer_handle , int *volume);
int audio_set_mic_volume(void * mixer_handle, int volume);
int audio_get_mic_volume(void * mixer_handle, int * volume);

/********************************************************************/

typedef struct {
	uint16_t crop_top_OID;
	uint16_t crop_left_OID;
	uint16_t crop_width_OID;
	uint16_t crop_height_OID;
	uint16_t width_OID;
	uint16_t height_OID;
	uint16_t crop_top_OCR;
	uint16_t crop_left_OCR;
	uint16_t crop_width_OCR;
	uint16_t crop_height_OCR;
	uint16_t width_OCR;
	uint16_t height_OCR;
	uint16_t work_mode;
	uint8_t exposure_init;
	uint8_t gain_init;
	uint8_t exposure_OCR;
	uint8_t gain_OCR;
	uint8_t numerator;
	uint8_t denominator;
}video_param_t;
 
typedef struct{
       uint32_t start_frame;
       uint32_t end_frame;
       int expo;
       int gain;
       int Y;
       int level_1;
       int level_2;
}adjust_ctrl_param;

typedef enum{
	IMG_MODE_UNKOWN = 0,
	IMG_MODE_OCR,
	IMG_MODE_OID,
}img_mode_t;

typedef struct{
	uint32_t mode : 4;
	uint32_t width : 12;
	uint32_t height : 12;
	uint32_t reserver : 4;
}img_info_t;

enum{
	MODE_OCR = 0,
	MODE_G2_AUTO,
	MODE_CHERRY_AUTO,       
};

typedef enum{
	OCR_DEV = 0,
	OID_DEV,
}dev_id_t;

typedef struct{
	uint32_t sequence;
	uint32_t timestamp_s;
	uint32_t timestamp_us;
	uint32_t confirm:1;
	uint32_t mode:3;
	uint32_t width:12;
	uint32_t height:12;
	uint32_t reserver:4;
}frames_info_t;

typedef struct{
	int (* screen_cap_operation)(void * video_hd ,adjust_ctrl_param * ctrl_param  ,struct v4l2_buffer buf, void *frames_data ,uint32_t frames_len ,int max_width ,int max_height , int def_expo ,int def_gain);
	int (* OID_opreation)(void *video_hd ,video_param_t *video_param , struct v4l2_buffer *buf, void *frames_data ,int *frames_len , frames_info_t *frame_info);
}callback_op_t;

typedef enum {
    CAMERA_BUFF_EMPTY = 0,
    CAMERA_BUFF_NORMAL = 1,
    CAMERA_BUFF_FULL =2,
}camera_sw_buf_status;
typedef struct {
    int buf_total_count;
    int buf_now_cnt;
    camera_sw_buf_status buf_status;
}camera_sw_buf_info_t;

int camera_init(void **handle , char *dev_name[2] ,video_param_t *video_param );
int camera_uninit(void *handle);
int camera_set_callback(void * handle, callback_op_t * callback_op);
int camera_show_debuginfo(void *handle);

int camera_capture_start(void *handle);
int camera_capture_stop(void *handle);
int camera_get_frame(void *handle ,dev_id_t dev_id ,frames_info_t **frames_info, unsigned char **frame_buff , unsigned int *buff_len);
int camera_reset_sw_buff(void * handle , dev_id_t dev_id);
int camera_get_sw_buff_info(void *handle , dev_id_t dev_id ,camera_sw_buf_info_t *sw_buf_info);
int camera_set_sw_buff_timeout(void *handle ,dev_id_t dev_id, int time_out_msec);
int camera_get_img_mode(void * handle  , int* img_mode);
int camera_set_img_mode(void * handle  , int img_mode);
int camera_get_gain(void * video_hd , dev_id_t dev_id ,int* gain);
int camera_set_gain(void * video_hd , dev_id_t dev_id ,int gain);
int camera_get_exposure(void * video_hd ,dev_id_t dev_id, int* expo);
int camera_set_exposure(void * video_hd ,dev_id_t dev_id, int expo);
int camera_set_product_type(char* pro_name);

/********************************************************************/

typedef enum{
    button_unkown = 0,
    button_camera = 1,
    button_yd_evt_camera = 2,
    button_asr = 3,
    button_menu = 4,
    button_screenlock = 5,
    button_power = 6,
    bt_connect = 7,
    slip_back = 8,
    button_count,
}button_id_t;

typedef enum{
    button_act_ignore = 0,
    button_act_press,
    button_act_release,
    button_act_longpress_release,
    button_act_move,
}button_action_t;

enum {
	direction_right = 0,
	direction_left,
};

int button_init(void **handle , button_id_t button_num , struct timeval *wait_timeout);
int button_uninit(void *handle);
int button_is_trigger(void *handle , bool * trigger);
int button_status(void *handle , button_action_t *button_action , int *dir, int *x , int *y);
char *button_get_name(button_id_t button_id);


/********************************************************************/

typedef enum{
    led_IR_camera = 0,
    led_camera = 15,
}led_id_t;

int led_init(int id);
int led_uninit(int id);
int led_on(int id);
int led_off(int id);


/************wifi api start************************/
#define MAX_WIFI_NAME_LEN  512
#define MAX_WIFI_HEX_NAME_LEN  128
typedef struct{
    bool on;
    bool link;
    bool internet_connect;
    bool airkiss_info_valid;
    char ssid[MAX_WIFI_NAME_LEN];
    int  signal_qua;

}wifi_status_t;

typedef struct{
    char wifi_name[MAX_WIFI_NAME_LEN];      // ssid 常规字符串
    char wifi_name_hexstr[MAX_WIFI_HEX_NAME_LEN];   // wpa_supplicant读取
    char wifi_name_hex[MAX_WIFI_NAME_LEN];  // 
    char security[64];
    int  signal_qua;
    char identity[128];                     // for wpa-eap / ieee802.1x
    int  eap;                               // for wpa-eap / ieee802.1x eap type
    char ca[128];                           // for wpa-eap / ieee802.1x CA
    int  status_flag;                       // ENUM_NETWORK_STATUS{DISABLED, CURRENT, CANDIDATE, UNKNOWN}
    int  ssid_hidden_flag;
    int  signal_level;
}wifi_dev_t;
/*
 * EAP Method Types as allocated by IANA:
 * http://www.iana.org/assignments/eap-numbers
 */
typedef enum {
    EAP_TYPE_NONE = 0,
    EAP_TYPE_IDENTITY = 1 /* RFC 3748 */,
    EAP_TYPE_NOTIFICATION = 2 /* RFC 3748 */,
    EAP_TYPE_NAK = 3 /* Response only, RFC 3748 */,
    EAP_TYPE_MD5 = 4, /* RFC 3748 */
    EAP_TYPE_OTP = 5 /* RFC 3748 */,
    EAP_TYPE_GTC = 6, /* RFC 3748 */
    EAP_TYPE_TLS = 13 /* RFC 2716 */,
    EAP_TYPE_LEAP = 17 /* Cisco proprietary */,
    EAP_TYPE_SIM = 18 /* RFC 4186 */,
    EAP_TYPE_TTLS = 21 /* RFC 5281 */,
    EAP_TYPE_AKA = 23 /* RFC 4187 */,
    EAP_TYPE_PEAP = 25 /* draft-josefsson-pppext-eap-tls-eap-06.txt */,
    EAP_TYPE_MSCHAPV2 = 26 /* draft-kamath-pppext-eap-mschapv2-00.txt */,
    EAP_TYPE_TLV = 33 /* draft-josefsson-pppext-eap-tls-eap-07.txt */,
    EAP_TYPE_TNC = 38 /* TNC IF-T v1.0-r3; note: tentative assignment;
               * type 38 has previously been allocated for
               * EAP-HTTP Digest, (funk.com) */,
    EAP_TYPE_FAST = 43 /* RFC 4851 */,
    EAP_TYPE_PAX = 46 /* RFC 4746 */,
    EAP_TYPE_PSK = 47 /* RFC 4764 */,
    EAP_TYPE_SAKE = 48 /* RFC 4763 */,
    EAP_TYPE_IKEV2 = 49 /* RFC 5106 */,
    EAP_TYPE_AKA_PRIME = 50 /* RFC 5448 */,
    EAP_TYPE_GPSK = 51 /* RFC 5433 */,
    EAP_TYPE_PWD = 52 /* RFC 5931 */,
    EAP_TYPE_EKE = 53 /* RFC 6124 */,
    EAP_TYPE_EXPANDED = 254 /* RFC 3748 */
}ENUM_EAP_TYPE;

typedef enum {
    DISABLED = 0,
    CURRENT,
    CANDIDATE,
    TEMP_DISABLED,     //秘钥协商失败
    STATUS_UNKNOWN         // TODO
}ENUM_NETWORK_STATUS;

int set_wifi_onoff(bool on);
int get_wifi_status(wifi_status_t *wifi_status);
int wifi_start_airkiss(void);
int wifi_stop_airkiss(void);
int wifi_scan(wifi_dev_t wifi_list[64] , int *dev_count);
int wifi_connect(wifi_dev_t * wifi_dev , char *password);
int wifi_disconnect(wifi_dev_t * wifi_dev);
int wifi_remove(wifi_dev_t * wifi_dev);
int _trans_chinese_ssid(char *raw_info , char *ch_info);
/*********wifi api end*************/
typedef struct{
    char name[128];
    char addr[18];
    int signal_qua;
    bool online;
    bool connect;
}bluetooth_dev_t;
typedef struct{
    bool on;
    bool link_dev;
    bluetooth_dev_t bt_dev;
    char ble[16];
}bluetooth_status_t;

typedef struct{
    char mac[18];
    char sn_str[17];
    char dev_name[18];
    char version[18];
    uint32_t mem_size;
    uint32_t flash_size;
}system_info_t;

typedef enum{
    mode_init = -1,
    mode_normal = 0,
    mode_high = 1,
    mode_low = 2,
    mode_unkown
}performance_mode_t;

typedef enum{
    USB_MODE_DISABLEMODE = 0,
    USB_MODE_NOINSERT,
    USB_MODE_CHARGEONLY,
    USB_MODE_DATALINK,
}usb_mode_t;

typedef enum{
    GET_VERSION_IN_OTA,
    GET_VERSION_IN_BIN
}get_version_type_e;

int set_lcd_brightness(int brightness);
int get_lcd_brightness(int *brightness);

int bluetooth_onoff(bool on, const char *ble);
int bluetooth_send(const char *str);
int bluetooth_power(int val);
int bluetooth_is_connect_done();
int bluetooth_scan(bluetooth_dev_t bt_list[64] , int *dev_count);
int bluetooth_scan_start();
int bluetooth_scan_interrupt();
int bluetooth_connect(bluetooth_dev_t *bt_dev);
int bluetooth_disconnect(bluetooth_dev_t *bt_dev);
int bluetooth_remove(bluetooth_dev_t *bt_dev);
int bluetooth_status(bluetooth_status_t *bt_status);

int set_sn(char sn_str[17]);
int get_sn(char sn_str[17]);

int set_spk_volume(int volume);
int get_spk_volume(int *volume);

int get_mac(char mac[18]);
int get_version(char version[18] , char *version_num , char *version_subtype, int type);
int get_cpu_serial(char cpu_serial[24]);
int get_emmc_id(char emmc_id[64]);


int get_system_info(system_info_t * sys_info);
int get_battery_info(int* power , bool *charger_insert);

int check_usb_mode(usb_mode_t *usb_mode);
int ums_onoff(bool on, const char *content_path);
int mtp_onoff(bool on);
int adb_onoff(bool on);
int ssh_onoff(bool on);

void factory_reset(int reboot_sec);
void factory_and_clear(int reboot_sec);

int set_system_performance(performance_mode_t mode);
int get_system_performance(performance_mode_t *mode);

int set_cpu_core(uint8_t core_num, bool onoff);

int playback_onoff(bool on);

int capture_path_onoff(bool on);

/********************************************************************/

int get_uuid(char str[34]);

bool license_save(const char *str);
bool license_verify();

int hilink_power(int val);
int hilink_send(const char *str);
int hilink_save(const char *str);
/**********************************/
#if 0
typedef enum{
        SW_VERSION_CHS = 0, // 简体中文版(默认)
    SW_VERSION_ENG,     // 英文版
    SW_VERSION_KOR,     // 韩语版
    SW_VERSION_CHT      // 繁体中文版
}sw_version_t;

#define DEFAULT_SW_VERSION SW_VERSION_CHS	// 读取VENDOR版本信息出错时，置为该版本
/*
 * para: int *sw_version, 当return 0 时，该值有效，否则无效
 * return: 0,成功,*sw_version为读取到的版本值；<0:失败,此时*sw_version=DEFAULT_SW_VERSION
 */
int get_sw_version(int *sw_version);
#endif
/*
 * VERSION 枚举值
 */
typedef enum{
    D2 = 2,        // 二代笔
    D3         	   // 三代笔(默认)
}vs_version_t;

/*
 * REGION 枚举值
 */
typedef enum{
    SKU_CHN = 0, 	// 大陆版（默认）
    SKU_TWN,		// 中国-台湾版
    SKU_KOR,     	// 韩国版
    SKU_JPN,		// 日本版
    SKU_USA,		// 美国版
    SKU_GBR,		// 英国版
    SKU_HLK,		// Hilink定制版
    SKU_MG		    // 芒果版

}vs_region_t;

/*
 * HW_CFG 硬件配置枚举值
 */
typedef enum{
    ADV = 0,     	// 低配版(二代笔默认)
    PRO,  		    // 高配版
    STD			    // 标准版(三代笔默认)
}vs_hw_cfg_t;

typedef struct{
    int version;	// 词典笔版本:二代,三代
    int region;		// 词典笔渠道:国家,地区,渠道(华为Hilink、芒果版)
    int hw_cfg;		// 硬件配置 低配版(ADV),高配版(PRO)
}sku_t;

/*
 * para: sku_t *sku 当return 0 时，该值有效，否则无效
 * return: 0,成功,*sku为读取到的SKU品类；<0:失败,此时*sku无效,目前错误处理放在了该接口,故一定会返回0,并给出一个确定的sku品类
 */
int get_sku(sku_t *sku);

/*
 * pcba版本的枚举值，调用get_pcba_version获取到值后，通过该枚举值分支
 */
typedef enum{
        V0 = 0,         // 
        V4              // 
}pcba_version_t;        // 

/*
 * @para: char *pcba_version, 该值为pcba版本
 * 
 * return: 0,成功,*pcba_version为读取到的版本；<0:失败
 */
int get_pcba_version(char *pcba_version);


typedef struct{
    uint32_t firmware;  // 固件空间
    uint32_t resource;  // 资源空间
    uint32_t user;      // 用户空间
    uint32_t available;  // 剩余空间
}storage_info_t;
/*
 * @para: int get_storage_info, 返回空间使用情况
 * 
 * return: 0,成功,<0,失败
 */
int get_storage_info(storage_info_t *storage_info);

/*
 * @para: int prio, const char *fmt, ...
 *
 * return:
 */
void runtime_log(int prio, const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
