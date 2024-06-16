/*
 * Copyright (c) 2017 FIH Mobile Limited.
 */

#define fver_BLOCK 		"/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/systeminfo"
#define DEVICEINFO_LOCATION     "/dev/block/platform/mtk-msdc.0/11230000.msdc0/by-name/deviceinfo"
#define wifimac_BLOCK		"/vendor/nvdata/APCFG/APRDEB/WIFI"
#define btmac_BLOCK		"/vendor/nvdata/APCFG/APRDEB/BT_Addr"

#define STATUSROOT_LOCATION 	"/BBSYS/status.cfg"
//#define FIH_HWID_INFO 	"/system/etc/Fih_Hwid_Info"
//#define FIH_HWID_INFO 		"/sbin/Fih_Hwid_Info"
#define FIH_HWID_INFO 		"/vendor/etc/Fih_Hwid_Info"
#define FIH_HWID_TAG_SIZE 16
#define FIH_HWID_TAG 		"FIH_HWID_INFO"

#define MODULE_PROC 		"MODULE"
#define FVER_PROC 		"fver"
#define MODEL_PROC 		"devmodel"
#define HWMODEL_PROC 		"hwmodel"
#define DRAM_PROC 		"draminfo"
#define EMMC_PROC 		"emmcinfo"
#define CPU_PROC 		"cpu_info"
#define LCM_PROC 		"LCM"
#define POWERONCAUSE_PROC 	"poweroncause"
#define BASEBAND_PROC 		"baseband"
#define BASEBAND_SETTINGS_PROC 	"baseband_settings"
#define PCBA_DESCRIPTION_PROC 	"pcba_description"
#define HWIDV_PROC 		"hwidv"
#define SIM1_proc 		"simstate1"
#define SIM2_proc 		"simstate2"
#define IMEI_PROC 		"imei"
#define IMEI2_PROC 		"imei2"
#define CAVIS_D 		"DeviceRegistered"
#define CAVIS_R 		"RetryTime"
#define PRODUCTID_PROC 		"productid"
#define TOUCH_INFO   		"Touch"
#define AUDIO_PROC 		"fih_audio_ver"
#define PROC_STATUSROOT 	"statusroot"
#define UICOLOR_PROC 		"custom_sw_color"
#define HWINFO_PROC  		"hwid_info"
#define LCM_CABC  		"CABC_settings"
#define SIM_CARD_SLOT_PROC	"sim_card_slot"
#define BANDINFO		"bandinfo"
#define EFUSE_INFO_PROC		"efuse_state"
#define OTG_LAST_FLAG		"otg_last_flag"
#define SIM_NUMBER		"simnum"
#define NFC_SUPPORT		"nfc_support"
#define FQCXMLPATH		"fqc_xml"
#define RAMRESULT_PROC		"dramtest_result"
#define SKUID_PROC		"skuid"

#define FIH_PON_APR_MODEM_FATAL           "0x10000000\n"
#define FIH_PON_APR_KERNEL_PANIC          "0x20000000\n"
#define FIH_PON_APR_UNKNOWN_RESET         "0x40000000\n"
#define FIH_PON_APR_FRAMEWORK_EXCEPTION   "0x80000000\n"
#define FIH_PON_APR_ABNORMAL_POWER_DOWN   "0x01000000\n"
#define FIH_PON_APR_ABNORMAL_POWER_RESET  "0x02000000\n"

#define FIH_PROC_CDA_USER_DIR   "cda"
#define FIH_PROC_CDA_USER_PATH  "cda/user"

#define FIH_CDA_KERN_USER  (0)
#define FIH_CDA_KERN_ROOT  (1)
#define FIH_CDA_STAT_USER  (0)
#define FIH_CDA_STAT_ROOT  (1)

#define WIFI_MAC		"wifi_mac"
#define BT_MAC			"bt_mac"

int otg_last_flag = 0;

struct systeminfo {
	char	model_name[16];
	char	lcm_info[20];
	char	dram[15];
	char	emmc[20];
	char	cpu_name[16];
	char	phase_settings[16];
	char	phase[16];
	char	module[128];
	char	bandinfo[128];
	char	hw_family[16];
	char	pcba_description[112];
	char	hac[8];
	char	sim_num[8];
	char	audio_para[30];
	char    nfc_support[16];
};

struct fih_hwid_info {
	int	project_id;
	int	phase_id;
	int	module_id;
	char	project_name[16];
	char	phase_sw[16];
	char	phase_hw[16];
	char	cpu_name[16];
	char	module[128];
	char	bandinfo[128];
	char	hw_family[16];
	char	pcba_description[112];
	char	hac[8];
	char	sim_num[8];
	char    nfc_support[16];
};


struct systeminfo model[] = {
	{
		.model_name 	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings =	"unknow",
		.phase		=	"unknow",
		.module 	=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para 	=	"unknow",
		.nfc_support    =	"unknow"
	},
	{
		.model_name	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings	=	"unknow",
		.phase		=	"unknow",
		.module		=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para	=	"unknow",
		.nfc_support    =       "unknow"
	},
	{
		.model_name 	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings =	"unknow",
		.phase		=	"unknow",
		.module 	=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para 	=	"unknow",
		.nfc_support    =       "unknow"
	},
	{
		.model_name 	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings =	"unknow",
		.phase		=	"unknow",
		.module 	=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para 	=	"unknow",
		.nfc_support    =       "unknow"
	},
	{
		.model_name 	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings =	"unknow",
		.phase		=	"unknow",
		.module 	=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para 	=	"unknow",
		.nfc_support    =       "unknow"
	},
	{
		.model_name 	=	"unknow",
		.lcm_info	=	"unknow",
		.dram		=	"unknow",
		.emmc		=	"unknow",
		.cpu_name	=	"unknow",
		.phase_settings =	"unknow",
		.phase		=	"unknow",
		.module 	=	"unknow",
		.bandinfo	=	"unknow",
		.hw_family	=	"unknow",
		.hac		=	"unknow",
		.sim_num	=	"unknow",
		.audio_para 	=	"unknow",
		.nfc_support    =       "unknow"
	},
};
