/*
 * Copyright (c) 2017 FIH Mobile Limited.
 */

#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include "proc_info.h"
#include "Testflag.h"
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/unistd.h>
#include <linux/stat.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <asm/setup.h>
#include "../misc/mediatek/include/mt-plat/mt_boot_reason.h"
#include "../misc/mediatek/aee/common/aee-common.h"

#define FALSE  0
#define TRUE   1

#define SIZE_512M  (512*1024)  //kB
#define SIZE_1GB   (1024*1024) //kB
#define SIZE_2GB   (2048*1024) //kB
#define SIZE_3GB   (3072*1024) //kB
#define SIZE_4GB   (4096*1024) //kB

#define MEMINFO "/proc/meminfo"

extern unsigned short fih_hwid;
extern char *saved_command_line;
extern int sec_schip_enabled(void);

//extern unsigned short fih_gethwid(void);
extern unsigned short fih_get_memory_type(void);
extern unsigned short fih_get_memory_vendor(void);
extern unsigned long long fih_get_emmc_size(void);
extern unsigned long long fih_get_emmc_usersize(void);
extern unsigned int fih_get_ramtest_result(void);

char nfc_hw_support[16] = {'\0'};
struct mutex proc_read_mutex;

static char causeStr[99];
static char slot_suffix[8];
static char *fver_preload;
static int fver_len = 65536;
static int fver_open_times = 0;
static char sim_state='\0';
static char str_imei[32];
static char str_imei2[32];
static char str_pid[33];
static char str_uicolor[20];
static int wifimac_len = 20;
static char str_wifimac[20];
static int btmac_len = 20;
static char str_btmac[20];
static char *wifimac_preload = NULL;
static char *btmac_preload = NULL;

static char fih_emmc_info[20];
static char fih_emmc_vendor[10];
static char fih_emmc_size[5];
static char fih_dram_info[20];
static char fih_dram_vendor[10];
static char fih_dram_size[5];

extern u8 mem_cid[9];
static unsigned long long mtk_emmc_size = 0;

static char *root_status_load;
static int root_status_len = 4096;
static bool open_already = 0;
static int root_cda_user_flag = 0;

int devmodel_init = 0;
static char str_project[3];
static int sim_card_slot;

#define FIH_PROC_DIR   "AllHWList"
#define FIH_PROC_PATH  "AllHWList/draminfo"
#define FIH_PROC_TESTRESULT_PATH  "dramtest_result" //"AllHWList/dramtest_result"
#define FIH_PROC_SIZE  32

static char fih_proc_data[FIH_PROC_SIZE] = "";
static char fih_proc_test_result[FIH_PROC_SIZE] = {0};

#define FIH_MEM_ST_HEAD  0x6400000  /* HFME */
#define FIH_MEM_ST_TAIL  0x1400000  /* EMFT */

#define FIH_MEM_MEM_ADDR 0x6400000
#define FIH_MEM_MEM_SIZE 0x1400000

#define IMEI_SIZE        16
#define IMEI_OFFSET (4096+4096+16) //E2P+CDA+BSET

#define FQC_DEFAULT_PATH "vendor/etc/fqc_ds_ES2.xml"

struct st_fih_mem {
	unsigned int head;
	unsigned int mfr_id;
	unsigned int ddr_type;
	unsigned int size_mb;
	unsigned int test_result;
	unsigned int tail;
};
struct st_fih_mem dram;

static void fih_dram_setup_MEM(void)
{
	char *buf = fih_proc_data;
	struct st_fih_mem *p;
	char size[16];

	/* get dram in mem which lk write */
	p = (struct st_fih_mem *)ioremap(FIH_MEM_MEM_ADDR, sizeof(struct st_fih_mem));
	if (p == NULL) {
		pr_err("%s: ioremap fail\n", __func__);
		dram.head = FIH_MEM_ST_HEAD;
		dram.mfr_id = 0;
		dram.ddr_type = 0;
		dram.size_mb = 0;
		dram.test_result = 0;
		dram.tail = FIH_MEM_ST_TAIL;
	} else {
		memcpy(&dram, p, sizeof(struct st_fih_mem));
		iounmap(p);
	}

	/* check magic of dram */
	if ((dram.head != FIH_MEM_ST_HEAD)||(dram.tail != FIH_MEM_ST_TAIL)) {
		pr_err("%s: bad magic\n", __func__);
		dram.head = FIH_MEM_ST_HEAD;
		dram.mfr_id = 0;
		dram.ddr_type = 0;
		dram.size_mb = 0;
		dram.test_result = 0;//HCLai add for memory test in RUNIN
		dram.tail = FIH_MEM_ST_TAIL;
	}

	switch (dram.mfr_id) {
		case 0x01: strcat(buf, "SAMSUNG"); break;
		case 0x03: strcat(buf, "ELPIDA"); break;
		case 0x06: strcat(buf, "SK-HYNIX"); break;
		case 0xFF: strcat(buf, "MICRON"); break;
		default: strcat(buf, "UNKNOWN"); break;
	}

	switch (dram.ddr_type) {
		case 2: strcat(buf, " LPDDR2"); break;
		case 5: strcat(buf, " LPDDR3"); break;
		default: strcat(buf, " LPDDRX"); break;
	}

	snprintf(size, sizeof(size), " %dMB", dram.size_mb);
	strcat(buf, size);

	//To-Do: use switch case to identify PASS/FAIL
	snprintf(fih_proc_test_result, sizeof(fih_proc_test_result), "%d", dram.test_result);
}

static int draminfo_test_result_wtite(struct file *flip,const char __user *buf,
                                      size_t count,loff_t *f_pos)
{
	char temp[32] = {'\0'};
	if(copy_from_user(temp, buf, count))
		return -EFAULT;
#if 0
		struct st_fih_mem *mem = (struct st_fih_mem *)FIH_MEM_MEM_ADDR;
	//dprintf(INFO, "fih_mem_exit and set mem address %p\n",mem);
	 if (FIH_MEM_MEM_SIZE < sizeof(struct st_fih_mem)) {
		printk("%s: memory is too small\n", __func__);
		return ;
	}
	strcpy(mem->test_result,temp);
#endif
	memset(fih_proc_test_result, 0, sizeof(fih_proc_test_result));
	strcpy(fih_proc_test_result, temp);
	return count;
	//snprintf(fih_proc_test_result, sizeof(fih_proc_test_result), "%d",
  //simple_strtol(temp,NULL,0));
}

static int __init  slot_suffix_param(char *line)
{
        strlcpy(slot_suffix, line, sizeof(slot_suffix));
        return 1;
}
__setup("androidboot.slot_suffix=", slot_suffix_param);


static int fver_show(struct seq_file *s, void *unused)
{
	struct file *fver_filp = NULL;
	char str_fver[128];
	mm_segment_t oldfs;

	memset(str_fver, 0, 128);
	strcpy(str_fver, fver_BLOCK);
	strcat(str_fver, slot_suffix);

	if(!fver_open_times)
	{
		oldfs = get_fs();
		set_fs(KERNEL_DS);
		fver_filp = filp_open(str_fver, O_RDONLY, 0);

		if(!IS_ERR(fver_filp))
		{
			fver_filp->f_op->read(fver_filp, fver_preload,
                                              sizeof(char)*fver_len, &fver_filp->f_pos);
			filp_close(fver_filp, NULL);
			fver_open_times++;
			seq_printf(s, "%s\n", fver_preload);
		}
		else
		{
			printk("[dw]open %s fail\n", str_fver);
		}

		set_fs(oldfs);
	}
	else
	{
		seq_printf(s, "%s\n", fver_preload);
	}

	return 0;
}

static int model_show(struct seq_file *s, void *unused)
{
	unsigned short hw_project = (fih_hwid >> 8) & 0x000F;

	if(devmodel_init == 0)
	{
		memset(str_project, 0, sizeof(str_project));
		strcpy(str_project, model[hw_project].model_name);

		printk("%s: str_project = %s\n", __func__, str_project);
	}

	seq_printf(s, "%s\n", str_project);

	return 0;
}

static int hwmodel_show(struct seq_file *s, void *unused)
{
	unsigned short hw_project = (fih_hwid >> 8) & 0x000F;

	printk("%s: model_name = %s\n", __func__, model[hw_project].model_name);

	seq_printf(s, "%s\n", model[hw_project].model_name);

	return 0;
}

#define SIZE_1GB_EMMC (0x40000000)

static void get_emmc_vendor_and_size(void)
{
	u8 samsung_cid = 0x15;
	u8 hynix_cid = 0x90;
	u8 micron_cid = 0xfe;
	u8 sandisk_cid = 0x45;
	u8 kingston_cid = 0x70;
	u8 toshiba_cid = 0x11;

	unsigned short hw_preload = 0, num = 0;

	num = (fih_hwid >> 4) & 0x00F;
	hw_preload = (fih_hwid >> 8) & 0x00F;

	memset(fih_emmc_info, 0, 20);
	memset(fih_emmc_vendor, 0, 10);
	memset(fih_emmc_size, 0, 5);

	if (memcmp(mem_cid, &samsung_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Samsung");
	}
	else if (memcmp(mem_cid, &hynix_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Hynix");
	}
	else if (memcmp(mem_cid, &micron_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Micron");
	}
	else if (memcmp(mem_cid, &sandisk_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Sandisk");
	}
	else if (memcmp(mem_cid, &kingston_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Kingston");
	}
	else if (memcmp(mem_cid, &toshiba_cid, 1) == 0)
	{
		strcpy(fih_emmc_vendor, "Toshiba");
	}

	if(mtk_emmc_size == 0)
	{
		mtk_emmc_size = fih_get_emmc_size();
	}

	if(mtk_emmc_size/(SIZE_1GB_EMMC) < 4)
	{
		strcpy(fih_emmc_size,"4GB");
	}
	else if(mtk_emmc_size/(SIZE_1GB_EMMC) > 0x4 && mtk_emmc_size/(SIZE_1GB_EMMC) <= 0x8)
	{
		strcpy(fih_emmc_size,"8GB");
	}
	else if(mtk_emmc_size/(SIZE_1GB_EMMC) > 0x8 && mtk_emmc_size/(SIZE_1GB_EMMC) <= 0x10)
	{
		strcpy(fih_emmc_size,"16GB");
	}
	else if(mtk_emmc_size/(SIZE_1GB_EMMC) > 0x10 && mtk_emmc_size/(SIZE_1GB_EMMC) <= 0x20)
	{
		strcpy(fih_emmc_size,"32GB");
	}
	else if(mtk_emmc_size/(SIZE_1GB_EMMC) > 0x20 && mtk_emmc_size/(SIZE_1GB_EMMC) <= 0x40)
	{
		strcpy(fih_emmc_size,"64GB");
	}

	strcpy(fih_emmc_info, fih_emmc_vendor);
	strcat(fih_emmc_info, fih_emmc_size);
	strcpy(model[hw_preload].emmc, fih_emmc_info);
}

long power(int a, int b)
{
	int i = 0;
	long ret = 1;

	for(i=0;i<b;i++)
	{
		ret *= a;
	}

	//printk("a= %d, b=%d, ret = %d\n", a, b, ret);

	return ret;
}

static bool fih_read_meminfo(char *temp, int size)
{
	struct file *fdata_filp = NULL;
	mm_segment_t oldfs;
	bool ret = FALSE;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fdata_filp = filp_open(MEMINFO, O_RDONLY, 0);

	if(!IS_ERR(fdata_filp))
	{
		memset(temp, 0, size);
		fdata_filp->f_op->read(fdata_filp, temp, size, &fdata_filp->f_pos);

		if(0 == strcmp(temp, "0"))
		{
			ret = FALSE;
		}
		else
		{
			ret = TRUE;
		}

		filp_close(fdata_filp, NULL);
	}
	else
	{
		printk("%s: open %s failed\n", __func__, MEMINFO);
	}

	printk("%s: qyf %s \n", __func__, temp);
	set_fs(oldfs);

	return ret;
}

static int dram_show(struct seq_file *s, void *unused)
{
	char info[30];
	char size[10];

	int i = 0, count = 0, final = 0;

	unsigned short hw_preload = 0, dram_num = 0;
	unsigned int memory_type = 0, ddr_verdor = 0;

	memory_type = fih_get_memory_type();
	ddr_verdor = fih_get_memory_vendor();

	dram_num = (fih_hwid >> 4) & 0x00F;
	hw_preload = (fih_hwid >> 8) & 0x00F;

	memset(fih_dram_info, 0, 20);
	memset(fih_dram_vendor, 0, 10);
	memset(fih_dram_size, 0, 5);

	printk("qyf memory_type = 0x%x, ddr_verdor =0x%x\n", memory_type, ddr_verdor);

	/* nuc start for fqc info show*/
	if((memory_type & 0xF00) != 0)   //for EMCP project:H1M
	{
		printk("qyf111 fih_emmc_vendor = %s\n", fih_emmc_vendor);

		get_emmc_vendor_and_size();
		strcpy(fih_dram_vendor, fih_emmc_vendor);
	}
	else
	{
		printk("qyf222 fih_emmc_vendor = %s\n", fih_emmc_vendor);

		if(ddr_verdor == 0x1)
		{
			strcpy(fih_dram_vendor, "Samsung");
		}
		else if(ddr_verdor == 0x3)
		{
			strcpy(fih_dram_vendor, "Elpida");
		}
		else if(ddr_verdor == 0x5)
		{
			strcpy(fih_dram_vendor, "Kingston");
		}
		else if(ddr_verdor == 0x6)
		{
			strcpy(fih_dram_vendor, "Hynix");
		}
	}

	memset(info, 0, 30);
	memset(size, 0, 10);
	fih_read_meminfo(info, 30);

	printk("%s: qyf info = %s \n", __func__, info);

	for(i = 0; i < 30; i++)
	{
		//printk("%s: qyf info[%d] = %d \n", __func__,  i, info[i]);
		if(info[i] >= '0' && info[i] <= '9')
		{
			size[count] = info[i];

			//printk("%s: qyf size[%d] = %d \n", __func__, count, size[count]);
			count++;
		}
	}
	//printk("%s: qyf count = %d \n", __func__, count);

	for(i = 0; i < count; i++)
	{
		final += ((int)((size[i] - '0')*(power(10, (count-1-i)))));
	}

	printk("%s: qyf %d \n", __func__, final);

	if(final >= SIZE_512M && final <= SIZE_1GB)
		strcpy(fih_dram_size, "1GB");
	else if(final >= SIZE_1GB && final <= SIZE_2GB)
		strcpy(fih_dram_size, "2GB");
	else if(final >= SIZE_2GB && final <= SIZE_3GB)
		strcpy(fih_dram_size, "3GB");
	else if(final >= SIZE_3GB && final <= SIZE_4GB)
		strcpy(fih_dram_size, "4GB");

	printk("qyf fih_dram_vendor = %s, fih_dram_size = %s\n", fih_dram_vendor, fih_dram_size);

	strcpy(fih_dram_info,fih_dram_vendor);
	strcat(fih_dram_info,fih_dram_size);

	printk("qyf fih_dram_info = %s\n", fih_dram_info);

	strcpy(model[hw_preload].dram, fih_dram_info);
	seq_printf(s, "%s\n", model[hw_preload].dram);

	return 0;
}

static int emmc_show(struct seq_file *s, void *unused)
{
	extern u8 mem_cid[9];
	unsigned short hw_preload = 0,emmc_num = 0;

	emmc_num = (fih_hwid >> 4) & 0x00F;
	hw_preload = (fih_hwid >> 8) & 0x00F;

	get_emmc_vendor_and_size();  /* nuc start for fqc info show */
	seq_printf(s, "%s\n", model[hw_preload].emmc);

	return 0;
}

static int poweroncause_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%s", causeStr);
	return 0;
}

static int poweroncause_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char buff[25] = {'\0'};

	if(copy_from_user(buff, buf, count))
		return -EFAULT;

	printk("kernel: poweroncause virtual file created.\n");

	strcpy(causeStr, buff);
	return count;
}

static int cpu_show(struct seq_file *s, void *unused)
{
	unsigned short hw_project = (fih_hwid >> 8) & 0x00F;

	printk("%s: cpu_name = %s\n", __func__, model[hw_project].cpu_name);
	seq_printf(s,"%s\n", model[hw_project].cpu_name);

	return 0;
}

static char fih_lcm_info[256] = "unknown";

static int lcm_show(struct seq_file *s, void *unused)
{
	//add for ZM1 FQC lcm info.
	if (!strcmp(fih_lcm_info, "nt35521_hd720_dsi_vdo_innolux"))
	{
		seq_printf(s, "NT35521 5.0' 720P\n");
	}
	else
	{
		seq_printf(s, "unknow\n");
	}

	return 0;
}

static char fih_touch[32] = "unknown";

void fih_info_set_touch(char *info)
{
	printk("touch: fih_info_set_touch.\n");
	strcpy(fih_touch, info);
}

static int fih_touch_read_proc(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	int len;

	printk("touch: allhwlist touch virtual file read.\n");
	return snprintf(page, PAGE_SIZE, "%s\n", fih_touch);
}


void fih_info_set_lcm(char *info)
{
	printk("lcm: fih_info_set_lcm.\n");
	strcpy(fih_lcm_info, info);
}

static int sim1_show(struct seq_file *s, void *unused)
{
	switch(sim_state)
	{
		case '0':
			seq_printf(s,"sim1:not inserted\n");
			break;
		case '1':
			seq_printf(s,"sim1:inserted\n");
			break;
		case '2':
			seq_printf(s,"sim1:not inserted\n");
			break;
		case '3':
			seq_printf(s,"sim1:inserted\n");
			break;
		default:
			seq_printf(s,"not UNKNOW\n");
	}

	return 0;
}

static int sim2_show(struct seq_file *s, void *unused)
{
	switch(sim_state)
	{
		case '0':
			seq_printf(s,"sim2:not inserted\n");
			break;
		case '1':
			seq_printf(s,"sim2:not inserted\n");
			break;
		case '2':
			seq_printf(s,"sim2:inserted\n");
			break;
		case '3':
			seq_printf(s,"sim2:inserted\n");
			break;
		default:
			seq_printf(s,"not UNKNOW\n");
	}

	return 0;
}

static int sim_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[25] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	if((temp[0] - '0') < 4)
		sim_state = temp[0];

	return count;
}

static int imei_show(struct seq_file *s, void *unused)
{
	printk("imei_show enter\n");

	seq_printf(s, "%s\n", str_imei);

	return 0;
}

static int imei2_show(struct seq_file *s, void *unused)
{
	printk("imei_show enter\n");

	seq_printf(s,"%s\n",str_imei2);

	return 0;
}

static int cavis_d_show(struct seq_file *s, void *unused)
{
	char a[50] = {0};

	printk("cavis_d_show enter\n");

	fih_read_CAVIS(a, 1);
	seq_printf(s, "%s\n", a);

	return 0;
}

static int cavis_r_show(struct seq_file *s, void *unused)
{
	char a[50] = {0};

	printk("cavis_r_show enter\n");

	fih_read_CAVIS(a, 2);
	seq_printf(s, "%s\n", a);

	return 0;
}

static int module_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].module);

	seq_printf(s, "%s\n", model[project_id].module);

	return 0;
}

static int pid_show(struct seq_file *s, void *unused)
{
	printk("pid_show enter\n");

	seq_printf(s, "%s\n", str_pid);
	return 0;
}

static int pid_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[32] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	printk("[dw]pid_write = %s*\n", temp);

	memset(str_pid, 0, sizeof(str_pid));
	strcpy(str_pid, temp);

	return count;
}

static int model_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[4] = {0};

	//printk("model_write enter count = %d\n", count);
	if(copy_from_user(temp, buf, (count>3?3:count)))
		return -EFAULT;

	printk("model_write string = %s\n", temp);

	memset(str_project, 0, sizeof(str_project));
	strncpy(str_project, temp, (count > 3 ? 3 : count));

	devmodel_init = 1;
	return count;
}

static int touch_show(struct seq_file *s, void *unused)
{
	printk("touch_show enter\n");

	seq_printf(s, "%s\n", fih_touch);
	return 0;
}

static int touch_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[32] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	printk("[dw]touch_write = %s*\n",temp);

	memset(fih_touch, 0, sizeof(fih_touch));
	strcpy(fih_touch, temp);

	return count;
}

static int lcm_info_show(struct seq_file *s, void *unused)
{
	printk("lcm_info_show enter\n");

	seq_printf(s,"%s\n", fih_lcm_info);
	return 0;
}

static int lcm_info_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[256] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	printk("[dw]lcm_info_write=%s*\n",temp);

	memset(fih_lcm_info, 0, sizeof(fih_lcm_info));
	strcpy(fih_lcm_info, temp);

	return count;
}

#if 0
static int ram_result_show(struct seq_file *s, void *unused)
{
        printk("ram_result_show enter\n");

        seq_printf(s,"0x%x\n", fih_get_ramtest_result());
        return 0;
}
#endif

static int fih_proc_test_result_show(struct seq_file *m, void *v)
{
	printk("fih_proc_test_result_show enter\n");
	//sunjie seq_printf(m, "%s\n", fih_proc_test_result);
	sprintf(fih_proc_test_result, "%d\n", fih_get_ramtest_result());
	seq_printf(m, "%s\n", fih_proc_test_result);
	return 0;
}

static int draminfo_test_result_open(struct inode *inode, struct file *file)
{
	return single_open(file, fih_proc_test_result_show, &inode->i_private);
};

static void fih_imei_setup(int flag, char *info)
{
	struct file *pid_filp = NULL;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	pid_filp = filp_open(DEVICEINFO_LOCATION, O_RDWR, S_IRWXU);

	if (IS_ERR(pid_filp)) {
		printk("[%s] Fail to open %s\n", __func__, DEVICEINFO_LOCATION);
		set_fs(oldfs);
		return;
	}

	if(flag == 1) {
		memset(str_imei, 0, sizeof(str_imei));
		snprintf(str_imei, sizeof(str_imei), "%s", info);
		pid_filp->f_op->llseek(pid_filp, IMEI_OFFSET, SEEK_SET);
		pid_filp->f_op->write(pid_filp, (char *)str_imei, IMEI_SIZE, &pid_filp->f_pos);
	} else if(flag == 2) {
		memset(str_imei2, 0, sizeof(str_imei2));
		snprintf(str_imei2, sizeof(str_imei2), "%s", info);
		pid_filp->f_op->llseek(pid_filp, IMEI_OFFSET+IMEI_SIZE, SEEK_SET);
		pid_filp->f_op->write(pid_filp, (char *)str_imei2, IMEI_SIZE, &pid_filp->f_pos);
	} else {
		printk("[%s] Invalid IMEI\n", __func__);
	}

	vfs_fsync(pid_filp, 0);
	filp_close(pid_filp, NULL);
	set_fs(oldfs);
	return;

}

static int imei_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char p_imei[32] = {0};

	printk("[dw]imei_write enter\n");

	if(count <=2)
	{
		pr_info("input node is NULL \n");
		return -EFAULT;
	}
	if(copy_from_user(p_imei, buf, count))
		return -EFAULT;

	printk("[dw]imei_write = %s*\n", p_imei);

	fih_imei_setup(1,p_imei);

	return count;
}

static int imei2_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char p_imei[32] = {0};

	printk("[dw]imei_write enter\n");

	if(count <=2)
	{
		pr_info("input node is NULL \n");
		return -EFAULT;
	}
	if(copy_from_user(p_imei, buf, count))
		return -EFAULT;

	printk("[dw]imei_write = %s*\n", p_imei);

	fih_imei_setup(2,p_imei);

	return count;
}

static int cavis_d_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[25] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	memset(temp+2, 0, sizeof(temp)-2);
	fih_write_CAVIS(temp, 1);

	return count;
}

static int cavis_r_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
	char temp[25] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	memset(temp+2,0,sizeof(temp)-2);
	fih_write_CAVIS(temp,2);

	return count;
}

static int baseband_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].phase);

	seq_printf(s, "%s\n", model[project_id].phase);

	return 0;
}

static int baseband_settings_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0,module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].phase_settings);

	seq_printf(s, "%s\n", model[project_id].phase_settings);

	return 0;
}

static int pcba_description_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].pcba_description);

	seq_printf(s, "%s\n", model[project_id].pcba_description);

	return 0;
}

static int hwidv_show(struct seq_file *s, void *unused)
{
	int data[4];
	int proj, phase, module, rawvalue;

	proj   = (fih_hwid >> 8) & 0x00F;
	phase  = (fih_hwid >> 4) & 0x00F;
	module = fih_hwid & 0x00F;

	printk("%d, %d, %d\n", proj, phase, module);
	seq_printf(s, "proj %d, phase %d, module %d\n", proj, phase, module);

	return 0;
}

static int audio_info_show(struct seq_file *s, void *unused)
{
#if 0
	unsigned short project_id = (fih_hwid >> 8) & 0x00F;

	printk("audio parameters version %s, %x\n", model[project_id].audio_para, project_id);
	seq_printf(s, "%s\n", model[project_id].audio_para);

	return 0;
#endif
	char audio_parameters_version[10] = "5719";
	printk("audio parameters version: %s\n",audio_parameters_version);
	seq_printf(s, "%s\n",audio_parameters_version );
	return 0;
}

static int audio_info_write(struct file *flip, const char __user *buf, size_t count, loff_t *f_pos)
{
    unsigned short project_id = (fih_hwid >> 8) & 0x000F;
    char kbuf[6] = {'\0'};//default the audio version can not exceed 6 bit

    int i;
    char *para = model[project_id].audio_para;

    memset(model[project_id].audio_para, 0, 30);
    printk("audio_info_write para=%s,model[project_id].model_name=%s\n", para,model[project_id].model_name);
    //the default count is 6
    if (count > 6) {
        printk("audio_info_write failed \n");
        return -EFAULT;
    }
    if (copy_from_user(kbuf, buf, count))
        return -EFAULT;

    for (i = 0; i < 6; i++) {
        *(para + i) = *(kbuf + i);
    }
    *(para + i) = '\0';

    printk("audio_info_write  para=%s\n", para);
    return count;
}
//jennyxu add audio ]
static int uicolor_show(struct seq_file *s,void *unused)
{
	printk("uicolor_show enter\n");
	seq_printf(s,"%s\n",str_uicolor);
	return 0;
}

static int uicolor_write(struct file *flip,const char __user *buf,size_t count,loff_t *f_pos)
{
	char temp[32] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	memset(str_uicolor, 0, sizeof(str_uicolor));
	strcpy(str_uicolor, temp);
	//printk("[dw]str_uicolor=%s*\n",str_uicolor);

	return count;
};

static int sim_card_slot_show(struct seq_file *s,void *unused)
{
	printk("sim_card_slot_show enter\n");
        unsigned short project_id = 0;
        //FIH Houwuyang 2018/06/05 set sim_card_slot start.
        project_id = (fih_hwid >> 8) & 0x00F;

	printk("Current project id = %d, sim_num = %s.\n", project_id, model[project_id].sim_num);
	if(0 == strncmp(model[project_id].sim_num, "Single", strlen("Single")))
	{
		seq_printf(s,"%d\n",1);
		printk("set sim_card_slot = 1\n");
		sim_card_slot = 1;
	}
	else
	{
		seq_printf(s,"%d\n",2);
		printk("set sim_card_slot = 2\n");
	}
	return 0;
        //FIH Houwuyang 2018/06/05 set sim_card_slot end.
}

static int sim_card_slot_write(struct file *flip,const char __user *buf,size_t count,loff_t *f_pos)
{
	char temp[32] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	sim_card_slot = simple_strtol(temp,NULL,0);
	//printk("[dw]sim_card_slot=%d\n",sim_card_slot);

	return count;
};

static int hwid_info_show(struct seq_file *s, void *unused)
{
	unsigned short hw_preload = 0, project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00f;
	phase_id   = (fih_hwid >> 4) & 0x00f;
	module_id  = fih_hwid & 0x00f;

	hw_preload = project_id * 100 + phase_id * 10 + module_id;

	seq_printf(s, "%d\n", hw_preload);
}

static int bandinfo_show(struct seq_file *s,void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].bandinfo);

	seq_printf(s, "%s\n", model[project_id].bandinfo);

	return 0;
}

/* This function is called when the /proc file is open. */
static int root_status_info_show(struct seq_file *s, void *unused)
{
	mm_segment_t oldfs;
	struct file *rs_filp = NULL;

	printk("root_status_info_show open_already = %d\n", open_already);

	if(!open_already)
	{
		oldfs = get_fs();
		set_fs(KERNEL_DS);

		rs_filp = filp_open(STATUSROOT_LOCATION, O_RDONLY|O_NONBLOCK, 0);

		if(!IS_ERR(rs_filp))
		{
			rs_filp->f_op->read(rs_filp, root_status_load,
                                            sizeof(char)*root_status_len, &rs_filp->f_pos);
			filp_close(rs_filp, NULL);

			open_already = TRUE;
		}
		else
		{
			printk("Open %s Failed\n", STATUSROOT_LOCATION);
		}

		set_fs(oldfs);
	}

	seq_printf(s, "%s\n", root_status_load);

	return 0;
}

static int efuse_enabled_show(struct seq_file *s, void *unused)
{
	int sec_en = 0xff;

	sec_en = sec_schip_enabled();

	printk("efuse_enabled_show() sec_en = %d\n", sec_en);

	seq_printf(s, "%d\n", sec_en);
}

static int otg_last_flag_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "%d\n", otg_last_flag);
}

static int otg_last_flag_write(struct file *flip, const char __user *buf,
                               size_t count, loff_t *f_pos)
{
	char temp[32] = {'\0'};

	if(copy_from_user(temp, buf, count))
		return -EFAULT;

	otg_last_flag = simple_strtol(temp,NULL,0);
	//printk("[dw]sim_card_slot=%d\n",sim_card_slot);

	return count;
}

static int sim_number_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].sim_num);

	seq_printf(s, "%s\n", model[project_id].sim_num);

	return 0;
}

static int nfc_support_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;

	project_id = (fih_hwid >> 8) & 0x00F;
	phase_id   = (fih_hwid >> 4) & 0x00F;
	module_id  = fih_hwid & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].nfc_support);

	seq_printf(s, "%s\n", model[project_id].nfc_support);

	return 0;
}

static int fqc_xml_path_show(struct seq_file *s, void *unused)
{
	unsigned short project_id = 0, phase_id = 0, module_id = 0;
        char fqc_path[30] = FQC_DEFAULT_PATH;

	project_id = (fih_hwid >> 8) & 0x00F;

	printk("%s: %s\n", __func__, model[project_id].sim_num);
        printk("%s: %s\n", __func__, model[project_id].nfc_support);

	if(0 == strncmp(model[project_id].sim_num, "Single", strlen("Single")))
	{
                fqc_path[15] = 's';
        }

        if(0 == strncmp(model[project_id].nfc_support, "Support", strlen("Support")))
        {
                fqc_path[20] = 'N';
        }
        if(strchr(model[project_id].model_name, 'C') != NULL)
        {
                fqc_path[18] = 'C';
                fqc_path[19] = 'O';
        }
        printk("%s: %s\n", __func__, fqc_path);
        seq_printf(s, "%s\n", fqc_path);
	return 0;
}

static int cda_user_read_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", root_cda_user_flag);
	return 0;
}

static int cda_user_proc_write(struct file *file, const char  __user *buffer,
                                        size_t count, loff_t *data)
{
	struct manuf_data fih_proinfo_data;
	char tmp[16] = {0};
	int access = 0;

	if ( copy_from_user(tmp, buffer, count) ) {
		return -EFAULT;
	}

	memset(&fih_proinfo_data, 0, sizeof(struct manuf_data));

	access = read_ef(&fih_proinfo_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("cda_user_proc_writ: Read failed\n");
		return READ_MANUFACTURE_FAIL;
	}

	root_cda_user_flag = simple_strtoull(tmp, NULL, 0);

	switch (root_cda_user_flag) {
	case FIH_CDA_KERN_USER:
		fih_proinfo_data.rootflag.status = FIH_CDA_STAT_USER;
		break;
	case FIH_CDA_KERN_ROOT:
		fih_proinfo_data.rootflag.status = FIH_CDA_STAT_ROOT;
		break;
	default:
		fih_proinfo_data.rootflag.status = FIH_CDA_STAT_ROOT;
		break;
	}
	access = write_ef(&fih_proinfo_data);
	if ((FILE_NOT_FOUND == access) || (FILE_CORRUPTED == access))
	{
		printk("cda_user_proc_write: write_ef\n");
		return WRITE_MANUFACTURE_FAIL;
	}
	return count;
}

static int skuid_show(struct seq_file *s, void *unused)
{
	char fih_skuid[30] = {'\0'};
	char *p, *q;

	p = strstr(saved_command_line, "fih_skuid=");
	if (p == NULL) return 0;
	p = p+10;
	q = p;
	while(*q != ' ') q++;
	strncpy(fih_skuid, p, (int)(q-p));
	printk("%s: fih_skuid=%s\n", __func__, fih_skuid);

	seq_printf(s, "%s\n", fih_skuid);

	return 0;
}

static int wifi_show(struct seq_file *s, void *unused)
{
	struct file *wifimac_filp = NULL;
	mm_segment_t oldfs;
	loff_t pos = 0;
	int offset = 4;
	wifimac_preload = kmalloc(sizeof(char)*wifimac_len, GFP_KERNEL);

	printk("[%s] enter!!\n",__func__);
	oldfs = get_fs();
	set_fs(KERNEL_DS);
	wifimac_filp = filp_open(wifimac_BLOCK, O_RDONLY, 0);

	if(!IS_ERR(wifimac_filp))
	{
		if (wifimac_filp->f_pos != offset) {
			if (wifimac_filp->f_op->llseek) {
				if (wifimac_filp->f_op->llseek(wifimac_filp, offset, 0) != offset) {
						printk("[%s]nvram_read : failed to seek!!\n",__func__);
				}
			} else {
				wifimac_filp->f_pos = offset;
			}
		}

		pos = (loff_t)offset;
		vfs_read(wifimac_filp, (char __user *)wifimac_preload, sizeof(char)*wifimac_len, &pos);
		filp_close(wifimac_filp, NULL);
		seq_printf(s, "%x:",(int)wifimac_preload[0]);
		seq_printf(s, "%x:",(int)wifimac_preload[1]);
		seq_printf(s, "%x:",(int)wifimac_preload[2]);
		seq_printf(s, "%x:",(int)wifimac_preload[3]);
		seq_printf(s, "%x:",(int)wifimac_preload[4]);
		seq_printf(s, "%x",(int)wifimac_preload[5]);
	}
	else
	{
		printk("[%s]open %s fail\n", __func__,wifimac_BLOCK);
	}

	set_fs(oldfs);

	kfree(wifimac_preload);
	return 0;
}

static int bt_mac_show(struct seq_file *s, void *unused)
{
	struct file *btmac_filp = NULL;
	mm_segment_t oldfs;
	loff_t pos = 0;
	int offset = 0;
	btmac_preload = kmalloc(sizeof(char)*btmac_len, GFP_KERNEL);

	printk("[%s] enter\n",__func__);

	oldfs = get_fs();
	set_fs(KERNEL_DS);
	btmac_filp = filp_open(btmac_BLOCK, O_RDONLY, 0);

	if(!IS_ERR(btmac_filp))
	{
		if (btmac_filp->f_pos != offset) {
			if (btmac_filp->f_op->llseek) {
				if (btmac_filp->f_op->llseek(btmac_filp, offset, 0) != offset) {
						printk("[%s]nvram_read : failed to seek!!\n",__func__);
				}
			} else {
				btmac_filp->f_pos = offset;
			}
		}

		pos = (loff_t)offset;
		vfs_read(btmac_filp, (char __user *)btmac_preload, sizeof(char)*btmac_len, &pos);
		filp_close(btmac_filp, NULL);
		seq_printf(s,"%02x:%02x:%02x:%02x:%02x:%02x\n",
				btmac_preload[0], btmac_preload[1], btmac_preload[2],
				btmac_preload[3], btmac_preload[4], btmac_preload[5]);
	}
	else
	{
		printk("[%s]open %s fail\n", __func__,btmac_BLOCK);
	}

	set_fs(oldfs);

	kfree(btmac_preload);
	return 0;
}

static int baseband_open(struct inode *inode, struct file *file)
{
	return single_open(file, baseband_show, &inode->i_private);
}

static int baseband_settings_open(struct inode *inode, struct file *file)
{
	return single_open(file, baseband_settings_show, &inode->i_private);
}

static int pcba_description_open(struct inode *inode, struct file *file)
{
	return single_open(file, pcba_description_show, &inode->i_private);
}

static int hwidv_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwidv_show, &inode->i_private);
}

static int fver_open(struct inode *inode, struct file *file)
{
	return single_open(file, fver_show, &inode->i_private);
}

static int model_open(struct inode *inode, struct file *file)
{
	return single_open(file, model_show, &inode->i_private);
}

static int hwmodel_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwmodel_show, &inode->i_private);
}

static int dram_open(struct inode *inode, struct file *file)
{
	return single_open(file, dram_show, &inode->i_private);
}

static int emmc_open(struct inode *inode, struct file *file)
{
	return single_open(file, emmc_show, &inode->i_private);
}

static int poweroncause_open(struct inode *inode, struct file *file)
{
	return single_open(file, poweroncause_show, &inode->i_private);
}

static int cpu_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_show, &inode->i_private);
}

static int lcm_open(struct inode *inode, struct file *file)
{
	return single_open(file, lcm_show, &inode->i_private);
}

static int sim1_open(struct inode *inode, struct file *file)
{
	return single_open(file, sim1_show, &inode->i_private);
}

static int sim2_open(struct inode *inode, struct file *file)
{
	return single_open(file, sim2_show, &inode->i_private);
}

static int imei_open(struct inode *inode, struct file *file)
{
	return single_open(file, imei_show, &inode->i_private);
}

static int imei2_open(struct inode *inode, struct file *file)
{
	return single_open(file, imei2_show, &inode->i_private);
}

static int cavis_d_open(struct inode *inode, struct file *file)
{
	return single_open(file, cavis_d_show, &inode->i_private);
}

static int cavis_r_open(struct inode *inode, struct file *file)
{
	return single_open(file, cavis_r_show, &inode->i_private);
}

static int module_open(struct inode *inode, struct file *file)
{
	return single_open(file, module_show, &inode->i_private);
}

static int pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, pid_show, &inode->i_private);
}

static int touch_open(struct inode *inode, struct file *file)
{
	return single_open(file, touch_show, &inode->i_private);
}

static int lcm_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, lcm_info_show, &inode->i_private);
}



static int audio_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, audio_info_show, &inode->i_private);
}

static int hwid_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, hwid_info_show, &inode->i_private);
}

static int root_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, root_status_info_show, &inode->i_private);
}

static int cda_user_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, cda_user_read_show, &inode->i_private);
};

static int uicolor_open(struct inode *inode, struct file *file)
{
	return single_open(file, uicolor_show, &inode->i_private);
}

static int sim_card_slot_open(struct inode *inode, struct file *file)
{
	return single_open(file, sim_card_slot_show, &inode->i_private);
}

static int bandinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, bandinfo_show, &inode->i_private);
}

static int efuse_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, efuse_enabled_show, &inode->i_private);
}

static int otg_last_flag_open(struct inode *inode, struct file *file)
{
	return single_open(file, otg_last_flag_show, &inode->i_private);
}

static int sim_number_open(struct inode *inode, struct file *file)
{
	return single_open(file, sim_number_show, &inode->i_private);
}

static int nfc_support_open(struct inode *inode, struct file *file)
{
	return single_open(file, nfc_support_show, &inode->i_private);
}

static int fqc_xml_path_open(struct inode *inode, struct file *file)
{
	return single_open(file, fqc_xml_path_show, &inode->i_private);
}

static int skuid_open(struct inode *inode, struct file *file)
{
	return single_open(file, skuid_show, &inode->i_private);
}

static int wifi_open(struct inode *inode, struct file *file)
{
        return single_open(file, wifi_show, &inode->i_private);
}

static int bt_mac_open(struct inode *inode, struct file *file)
{
        return single_open(file, bt_mac_show, &inode->i_private);
}

static const struct file_operations fver_fops = {
        .open        = fver_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations model_fops = {
        .open        = model_open,
        .write       = model_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations hwmodel_fops = {
        .open        = hwmodel_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations emmc_fops = {
        .open        = emmc_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations poweroncause_fops = {
        .open        = poweroncause_open,
	.write		= poweroncause_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations dram_fops = {
        .open        = dram_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations cpu_fops = {
        .open        = cpu_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations baseband_fops = {
        .open        = baseband_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations baseband_settings_fops = {
        .open        = baseband_settings_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations pcba_description_fops = {
        .open        = pcba_description_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations hwidv_fops = {
        .open        = hwidv_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations lcm_fops = {
        .open        = lcm_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations sim1_fops = {
        .open        = sim1_open,
        .write	     = sim_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations imei_fops = {
        .open        = imei_open,
        .write	     = imei_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations imei2_fops = {
        .open        = imei2_open,
        .write	     = imei2_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations cavis_d_fops = {
        .open        = cavis_d_open,
        .write	     = cavis_d_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations cavis_r_fops = {
        .open        = cavis_r_open,
        .write	     = cavis_r_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations sim2_fops = {
        .open        = sim2_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations module_fops = {
        .open        = module_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations pid_fops = {
        .open        = pid_open,
	.write		 = pid_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations touch_fops = {
        .open        = touch_open,
	.write		 = touch_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations lcm_info_fops = {
        .open        = lcm_info_open,
	.write		= lcm_info_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations AUDIO_para_info_fops = {
        .open        = audio_info_open,
	.write= audio_info_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static struct file_operations draminfo_test_result_ops = {
	.owner   = THIS_MODULE,
	.open    = draminfo_test_result_open,
	.read    = seq_read,
	.write   = draminfo_test_result_wtite,
	.llseek  = seq_lseek,
	.release = single_release
};
#if 0
static const struct file_operations ramtest_result_fops = {
        .open        = ramtest_result_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};
#endif
static const struct file_operations root_status_fops = {
	.open		 = root_status_open,
	.read		 = seq_read,
	.llseek 	 = seq_lseek,
	.release	 = single_release,
};

static const struct file_operations cda_user_file_ops = {
	.open    = cda_user_proc_open,
	.read    = seq_read,
	.write   = cda_user_proc_write,
	.llseek  = seq_lseek,
	.release = single_release
};

static const struct file_operations uicolor_fops = {
	.open		 =  uicolor_open,
	.write		 =  uicolor_write,
	.read		 =  seq_read,
	.llseek		 =  seq_lseek,
	.release	 =  single_release,
};

static const struct file_operations hwid_info_fops = {
	.open		 =  hwid_info_open,
	.read		 =  seq_read,
	.llseek		 =  seq_lseek,
	.release	 =  single_release,
};

static const struct file_operations sim_card_slot_fops = {
	.open		 =  sim_card_slot_open,
	.write		 =  sim_card_slot_write,
	.read		 =  seq_read,
	.llseek		 =  seq_lseek,
	.release	 =  single_release,
};

static const struct file_operations bandinfo_fops = {
	.open		 =  bandinfo_open,
	.read		 =  seq_read,
	.llseek		 =  seq_lseek,
	.release	 =  single_release,
};

static const struct file_operations efuse_state_fops = {
        .open        = efuse_state_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations otg_last_flag_fops = {
	.open        = otg_last_flag_open,
	.write       = otg_last_flag_write,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations sim_number_fops = {
        .open        = sim_number_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations nfc_support_fops = {
        .open        = nfc_support_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations fqc_xml_path_fops = {
        .open        = fqc_xml_path_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations skuid_fops = {
        .open        = skuid_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations wifi_fops = {
        .open        = wifi_open,
        .read        = seq_read,
        .llseek      = seq_lseek,
        .release     = single_release,
};

static const struct file_operations bt_mac_fops = {
		.open            =  bt_mac_open,
		.read            =  seq_read,
		.llseek          =  seq_lseek,
		.release         =  single_release,
};

static int dec2hex_under100(int num_dec)
{
	int num_hex = 0;

	num_hex = num_dec / 16 * 10;
	num_hex = num_hex + num_dec % 16;

	return num_hex;
}

static int fs_get_hwid_num(void)
{
	mm_segment_t oldfs;
	struct file *fdata_filp = NULL;

	char hwid_num = 0;
	char hwid_tag[FIH_HWID_TAG_SIZE] = {0};
	int i;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	for(i=0;i<50;i++)
	{
		fdata_filp = filp_open(FIH_HWID_INFO, O_RDONLY, 0);

		if(!IS_ERR(fdata_filp))
		{
			fdata_filp->f_op->llseek(fdata_filp, 0, SEEK_SET);
			fdata_filp->f_op->read(fdata_filp, hwid_tag, FIH_HWID_TAG_SIZE, &fdata_filp->f_pos);

			if(0 == strncmp(hwid_tag, "FIH_HWID_INFO", strlen("FIH_HWID_INFO")))
			{
				fdata_filp->f_op->llseek(fdata_filp, FIH_HWID_TAG_SIZE, SEEK_SET);
				fdata_filp->f_op->read(fdata_filp, &hwid_num, 1, &fdata_filp->f_pos);

				hwid_num = dec2hex_under100(hwid_num);
			}
			else
			{
				hwid_num = 0;
				printk("fs_get_hwid_num() check hwid_info error!\n");
			}

			filp_close(fdata_filp, NULL);
			printk("fs_get_hwid_num() Read hwid_info Success!\n");
			break;
		}
		else
		{
			hwid_num = 0;
			printk("fs_get_hwid_num() Read hwid_info failed! %d\n", PTR_ERR(fdata_filp));
		}

		msleep(100);
	}

	set_fs(oldfs);
	return hwid_num;
}

static void fs_read_hwid_info(char *temp, int offset, int size)
{
	struct file *fdata_filp = NULL;
	mm_segment_t oldfs;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	fdata_filp = filp_open(FIH_HWID_INFO, O_RDONLY, 0);

	if(!IS_ERR(fdata_filp))
	{
		fdata_filp->f_op->llseek(fdata_filp, offset, SEEK_SET);
		fdata_filp->f_op->read(fdata_filp, temp, size, &fdata_filp->f_pos);

		filp_close(fdata_filp, NULL);
		printk("fs_read_hwid_info() Read hwid_info Success!\n");
	}
	else
	{
		printk("fs_read_hwid_info() Read hwid_info failed!\n");
	}

	set_fs(oldfs);
}

int fih_read_hwid_info(void *x)
{
	int i = 0;
	char fih_hwid_num = 0;

	struct fih_hwid_info *hwid_info_tabel = NULL;

	unsigned short hw_project = 0, hw_phase = 0, hw_module = 0;

	hw_project = (fih_hwid >> 8) & 0xF;
	hw_phase   = (fih_hwid >> 4) & 0xF;
	hw_module  = fih_hwid  & 0xF;

	fih_hwid_num = fs_get_hwid_num();
	printk("fih_read_hwid_info() fih_hwid_num = %d\n", fih_hwid_num);

	if(fih_hwid_num > 0)
	{
		hwid_info_tabel = kmalloc(sizeof(struct fih_hwid_info)*fih_hwid_num, GFP_KERNEL);

		if (!hwid_info_tabel)
		{
			memset(&model[hw_project], 0, sizeof(struct systeminfo));
			memcpy(&model[hw_project], &model[0], sizeof(struct systeminfo));

			printk("fih_read_hwid_info() kmalloc Failed!\n");

			return;
		}

		memset(hwid_info_tabel, 0, sizeof(struct fih_hwid_info)*fih_hwid_num);

		fs_read_hwid_info(hwid_info_tabel, FIH_HWID_TAG_SIZE*2,
                                  sizeof(struct fih_hwid_info)*fih_hwid_num);

		for(i = 0; i < fih_hwid_num; i++, hwid_info_tabel++)
		{
			if((hwid_info_tabel->project_id == hw_project) &&
				(hwid_info_tabel->phase_id == hw_phase) &&
                           (hwid_info_tabel->module_id == hw_module))
			{
				strcpy(model[hw_project].model_name, hwid_info_tabel->project_name);
				strcpy(model[hw_project].cpu_name, hwid_info_tabel->cpu_name);
				strcpy(model[hw_project].phase_settings, hwid_info_tabel->phase_sw);
				strcpy(model[hw_project].phase, hwid_info_tabel->phase_hw);
				strcpy(model[hw_project].module, hwid_info_tabel->module);
				strcpy(model[hw_project].bandinfo, hwid_info_tabel->bandinfo);
				strcpy(model[hw_project].pcba_description,
                                       hwid_info_tabel->pcba_description);
				strcpy(model[hw_project].hw_family, hwid_info_tabel->hw_family);
				strcpy(model[hw_project].hac, hwid_info_tabel->hac);
				strcpy(model[hw_project].sim_num, hwid_info_tabel->sim_num);
				strcpy(model[hw_project].nfc_support, hwid_info_tabel->nfc_support);
				break;
			}
		}

		kfree(hwid_info_tabel);

		if(i == fih_hwid_num)
		{
			memset(&model[hw_project], 0, sizeof(struct systeminfo));
			memcpy(&model[hw_project], &model[0], sizeof(struct systeminfo));
		}
		mutex_lock(&proc_read_mutex);
		strcpy(nfc_hw_support, model[hw_project].nfc_support);
		mutex_unlock(&proc_read_mutex);

	}
	else
	{
		memset(&model[hw_project], 0, sizeof(struct systeminfo));
		memcpy(&model[hw_project], &model[0], sizeof(struct systeminfo));
	}
	return 0;
}

static void init_poweroncause(void)
{
	enum boot_reason_t boot_reason = BR_UNKNOWN;
	boot_reason = get_boot_reason();
	switch(boot_reason)
	{
		case BR_UNKNOWN:
			strcpy(causeStr, FIH_PON_APR_UNKNOWN_RESET);break;
		default:
			     strcpy(causeStr, "0x00\n");
	}
#ifdef CONFIG_MTK_RAM_CONSOLE
	if (aee_rr_last_fiq_step() != 0)
		strcpy(causeStr, FIH_PON_APR_KERNEL_PANIC);
#endif
}

static int __init proc_info_module_init(void)
{
	struct proc_dir_entry *entry;
	struct proc_dir_entry *baseband_entry;
	struct proc_dir_entry *baseband_settings_entry;
	struct proc_dir_entry *pcba_description_entry;
	struct proc_dir_entry *hwidv_entry;
	struct proc_dir_entry *poweroncause_entry;
	struct proc_dir_entry *entry_C;

	mutex_init(&proc_read_mutex);
	kthread_run(fih_read_hwid_info, NULL, "fih_read_hwid_info");
	//fih_read_hwid_info();


	entry = proc_create(FVER_PROC, 0777, NULL, &fver_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", FVER_PROC);
	fver_preload = kmalloc(sizeof(char)*fver_len, GFP_KERNEL);

	entry = proc_create(MODEL_PROC, 0777, NULL, &model_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", MODEL_PROC);

	entry = proc_create(EMMC_PROC, S_IFREG | S_IRUGO, NULL, &emmc_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", EMMC_PROC);

	entry = proc_create(DRAM_PROC, S_IFREG | S_IRUGO, NULL, &dram_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", DRAM_PROC);

	entry = proc_create(CPU_PROC, S_IFREG | S_IRUGO, NULL, &cpu_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", CPU_PROC);

	baseband_entry = proc_create(BASEBAND_PROC, S_IFREG | S_IRUGO, NULL, &baseband_fops);
	if(baseband_entry == NULL)
		printk("[dw]creat proc %s fail\n", BASEBAND_PROC);

	baseband_settings_entry = proc_create(BASEBAND_SETTINGS_PROC, S_IFREG | S_IRUGO, NULL,
                                              &baseband_settings_fops);
	if(baseband_settings_entry==NULL)
		printk("[dw]creat proc %s fail\n", BASEBAND_SETTINGS_PROC);

	pcba_description_entry = proc_create(PCBA_DESCRIPTION_PROC, S_IFREG | S_IRUGO, NULL,
                                             &pcba_description_fops);
	if(pcba_description_entry==NULL)
		printk("[dw]creat proc %s fail\n", PCBA_DESCRIPTION_PROC);

	hwidv_entry = proc_create(HWIDV_PROC, S_IFREG | S_IRUGO, NULL, &hwidv_fops);
	if(hwidv_entry == NULL)
		printk("[dw]creat proc %s fail\n", HWIDV_PROC);

	poweroncause_entry = proc_create(POWERONCAUSE_PROC, S_IFREG | S_IRUGO | S_IWUGO, NULL,
                                         &poweroncause_fops);
	if(poweroncause_entry == NULL)
		printk("[dw]creat proc %s fail\n", POWERONCAUSE_PROC);
	init_poweroncause();
	//strcpy(causeStr, "0x00\n");

	entry = proc_create(LCM_PROC, S_IFREG | S_IRUGO, NULL, &lcm_fops);
	if(entry == NULL)
	printk("[dw]creat proc %s fail\n", LCM_PROC);

	entry = proc_create(AUDIO_PROC, 0777, NULL, &AUDIO_para_info_fops);
	if(entry == NULL)
		printk("creat audiopara_info proc %s fail\n", AUDIO_PROC);

	entry = proc_create(SIM1_proc, 0666, NULL, &sim1_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", SIM2_proc);

	entry = proc_create(SIM2_proc, 0666, NULL, &sim2_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", SIM2_proc);

	entry = proc_create(IMEI_PROC, 0777, NULL, &imei_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", IMEI_PROC);

	entry = proc_create(IMEI2_PROC, 0777, NULL, &imei2_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", IMEI2_PROC);

	entry = proc_create(MODULE_PROC, 0777, NULL, &module_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", MODULE_PROC);

	entry_C = proc_mkdir("CAVIS", NULL);
	if(entry_C == NULL)
		printk("[dw]creat proc fail\n");

	entry = proc_create(CAVIS_D, 0660, entry_C, &cavis_d_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", CAVIS_D);

	entry = proc_create(CAVIS_R, 0660, entry_C, &cavis_r_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", CAVIS_R);

	entry = proc_create(PRODUCTID_PROC, 0777, NULL, &pid_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", PRODUCTID_PROC);

	entry_C = proc_mkdir("AllHWList", NULL);
	if(entry_C == NULL)
		printk("creat AllHWList touch proc fail\n");

	entry = proc_create(TOUCH_INFO, 0777, entry_C, &touch_fops);
	if(entry == NULL)
		printk("creat touch proc %s fail\n", TOUCH_INFO);

	entry = proc_create(LCM_PROC, 0777, entry_C, &lcm_info_fops);
	if(entry == NULL)
		printk("creat AllHWList/lcm proc %s fail\n", LCM_PROC);

	entry = proc_create(HWMODEL_PROC, S_IFREG | S_IRUGO, NULL, &hwmodel_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", HWMODEL_PROC);

	entry = proc_create(IMEI_PROC, 0777, entry_C, &imei_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", IMEI_PROC);

	entry = proc_create(IMEI2_PROC, 0777, entry_C, &imei2_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", IMEI2_PROC);

	fih_dram_setup_MEM();
	entry = proc_create(FIH_PROC_TESTRESULT_PATH, 0777, entry_C, &draminfo_test_result_ops);

	if(entry == NULL)
		printk("creat AllHWList/dramtest_result proc %s fail\n", RAMRESULT_PROC);

	entry = proc_create(UICOLOR_PROC, S_IFREG | S_IRUGO, NULL, &uicolor_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", UICOLOR_PROC);

	entry = proc_create(PROC_STATUSROOT, 0777, NULL, &root_status_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", PROC_STATUSROOT);
	root_status_load = kmalloc(sizeof(char)*root_status_len, GFP_KERNEL);

	proc_mkdir(FIH_PROC_CDA_USER_DIR, NULL);
	entry = proc_create(FIH_PROC_CDA_USER_PATH, 0777, NULL, &cda_user_file_ops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", FIH_PROC_CDA_USER_PATH);

	entry = proc_create(HWINFO_PROC, S_IFREG | S_IRUGO, NULL, &hwid_info_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", HWINFO_PROC);

	entry = proc_create(SIM_CARD_SLOT_PROC, S_IFREG | S_IRUGO, NULL, &sim_card_slot_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", SIM_CARD_SLOT_PROC);

        entry = proc_create("SIMSlot", S_IFREG | S_IRUGO, NULL, &sim_card_slot_fops);
        if(entry == NULL)
                printk("creat proc %s fail\n", SIM_CARD_SLOT_PROC);

	entry = proc_create(BANDINFO, S_IFREG | S_IRUGO, NULL, &bandinfo_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", BANDINFO);

	entry = proc_create(EFUSE_INFO_PROC, S_IFREG | S_IRUGO, NULL, &efuse_state_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", EFUSE_INFO_PROC);

	entry = proc_create(OTG_LAST_FLAG, 0777, NULL, &otg_last_flag_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", OTG_LAST_FLAG);

	entry = proc_create(SIM_NUMBER, S_IFREG | S_IRUGO, NULL, &sim_number_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", SIM_NUMBER);

	entry = proc_create(NFC_SUPPORT, S_IFREG | S_IRUGO, NULL, &nfc_support_fops);
        if(entry == NULL)
                 printk("creat proc %s fail\n", NFC_SUPPORT);

	entry = proc_create(FQCXMLPATH, S_IFREG | S_IRUGO, NULL, &fqc_xml_path_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", FQCXMLPATH);

	entry = proc_create(SKUID_PROC, S_IFREG | S_IRUGO, NULL, &skuid_fops);
	if(entry == NULL)
		printk("creat proc %s fail\n", SKUID_PROC);

	entry = proc_create(WIFI_MAC, 0777, NULL, &wifi_fops);
	if(entry == NULL)
		printk("[dw]creat proc %s fail\n", WIFI_MAC);

	entry = proc_create(BT_MAC, 0777, NULL, &bt_mac_fops);
	if(entry == NULL)
		printk("create proc %s fail\n", BT_MAC);

	return 0;
}

static void __exit proc_info_module_exit(void)
{
	//return 0;
}

module_exit(proc_info_module_exit);
module_init(proc_info_module_init);


