#include <linux/interrupt.h>
#include <linux/i2c.h>
//#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/synclink.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/proc_fs.h>
#include "../../fs/proc/internal.h"

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif

#define DEBUG 1
#define DEBUG_SENSOR
#ifdef DEBUG_SENSOR
#define dbg(format, arg...) printk(KERN_ALERT format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define MMA7660_ADDR	0x4C

#define MMA7660_DATA_MASK		0x3F
//#define MMA7660_DATA_MASK		0x3E


#ifdef CONFIG_MMA7660_UT7GM

//parameters for UT7GM	(for froyo)
/*
//for gingerbread port mode
#define MMA7660_CHANNEL_X		0
#define MMA7660_CHANNEL_Y		1
#define MMA7660_CHANNEL_Z		2
#define MMA7660_DIR_X			1
#define MMA7660_DIR_Y			(-1)
#define MMA7660_DIR_Z			1
*/

//for gingerbread land mode ( standard)
#define MMA7660_CHANNEL_X		0
#define MMA7660_CHANNEL_Y		1
#define MMA7660_CHANNEL_Z		2
#define MMA7660_DIR_X			-1
#define MMA7660_DIR_Y			-1
#define MMA7660_DIR_Z			-1


#elif defined(CONFIG_MMA7660_UT10GM)

//parameters for UT10GM
#define MMA7660_CHANNEL_X		0
#define MMA7660_CHANNEL_Y		1
#define MMA7660_CHANNEL_Z		2
#define MMA7660_DIR_X			(-1)
#define MMA7660_DIR_Y			1
#define MMA7660_DIR_Z			(-1)

#else

//parameters for default
#define MMA7660_CHANNEL_X		0
#define MMA7660_CHANNEL_Y		1
#define MMA7660_CHANNEL_Z		2
#define MMA7660_DIR_X			(-1)
#define MMA7660_DIR_Y			(-1)
#define MMA7660_DIR_Z			(-1)

#endif


#define GSENSOR_PROC_NAME "mma7660"
static struct proc_dir_entry * s_proc = NULL;
static struct i2c_client * s_i2c_client = NULL;

static struct early_suspend s_mma7660_early_suspend;
static int s_mma7660_suspend = 0;

//static void mma7660_poll_callback(struct work_struct *work);
//static DECLARE_WORK(mma7660_poll_work, mma7660_poll_callback);
static int s_kthread_run = 0;

#define MIN_POLL_INTERVAL	70	//milliseconds

static int s_ts_model = 703;
static int mma7660_chip = 1;

static int __devinit mma7660_i2c_probe(struct i2c_client * client, const struct i2c_device_id * id);
static int mma7660_i2c_suspend(struct i2c_client * client, pm_message_t mesg);
static int mma7660_i2c_resume(struct i2c_client * client);


static void check_ts_model(void)
{
	/*
	int value = 703;
	extern char g_selected_utmodel[];
	if(sscanf(g_selected_utmodel, "%d", &value) == 1)
	{
		if(value > 0) 
			s_ts_model = value;
		printk("select ts model %d\n", s_ts_model);
	} 
	*/
	s_ts_model = 712; 
}

static void mma7660_read_thread(int run);

static inline unsigned int ms_to_jiffies(unsigned int ms)
{
        unsigned int j;
        j = (ms * HZ + 500) / 1000;
        return (j > 0) ? j : 1;
}

static volatile int s_x = 0;
static volatile int s_y = 0;
static volatile int s_z = 0;


/*double xyz_g_table[64] = {
0,0.047,0.094,0.141,0.188,0.234,0.281,0.328,
0.375,0.422,0.469,0.516,0.563,0.609,0.656,0.703,
0.750,0.797,0.844,0.891,0.938,0.984,1.031,1.078,
1.125,1.172,1.219,1.266,1.313,1.359,1.406,1.453,
-1.500,-1.453,-1.406,-1.359,-1.313,-1.266,-1.219,-1.172,
-1.125,-1.078,-1.031,-0.984,-0.938,-0.891,-0.844,-0.797,
-0.750,-0.703,-0.656,-0.609,-0.563,-0.516,-0.469,-0.422,
-0.375,-0.328,-0.281,-0.234,-0.188,-0.141,-0.094,-0.047
};*/

#if 1
/*
//712
int xyz_g_table_712[64] = {
0,47,94,141,188,234,281,328,
375,422,469,516,563,609,656,703,
750,797,844,891,938,984,1031,1078,
1125,1172,1219,1266,1313,1359,1406,1453,
-1500,-1453,-1406,-1359,-1313,-1266,-1219,-1172,
-1125,-1078,-1031,-984,-938,-891,-844,-797,
-750,-703,-656,-609,-563,-516,-469,-422,
-375,-328,-281,-234,-188,-141,-94,-47
};
*/
// 712
int xyz_g_table_712[64] = {
10, 20, 47,94,141,188,234,281,328,
375,422,469,516,563,609,656,703,
750,797,844,891,938,984,1031,1078,
1125,1172,1219,1266,1313,1359,1406,
-1406,-1359,-1313,-1266,-1219,-1172,
-1125,-1078,-1031,-984,-938,-891,-844,-797,
-750,-703,-656,-609,-563,-516,-469,-422,
-375,-328,-281,-234,-188,-141,-94,-47, -20, -10
};

int xyz_g_table_712_y[64] = {
0, 10, 20, 47,94,141,188,234,281,328,
375,422,469,516,563,609,656,703,
750,797,844,891,938,984,1031,1078,
1125,1172,1219,1266,1313,1359,
-1359,-1313,-1266,-1219,-1172,
-1125,-1078,-1031,-984,-938,-891,-844,-797,
-750,-703,-656,-609,-563,-516,-469,-422,
-375,-328,-281,-234,-188,-141,-94,-47, -20, -10, 0
};

// 703
int xyz_g_table[64] = {
0,40,80,120,160,200,240,280,  // 40
320,360,400,440,480,520,560,606, //40
670,734,798,862,926,990,1054,1118, //64
1198,1278,1358,1438,1518,1598,1678,1758, // 80
-1838,-1758,-1678,-1598,-1518,-1438,-1358,-1278,
-1198,-1118,-1054,-990,-926,-862,-798,-734,
-670,-600,-560,-520,-480,-440,-400,-360,
-320,-280,-240,-200,-160,-120,-80,-40
};


int xyz_g_table_8452[128] = {
0,30,60,90,120,150,180,210,
240,270,300,330,360,390,420,450,
480,510,540,570,600,630,660,690,
720,750,780,810,840,870,900,930,//
960,990,1020,1050,1080,1110,1140,1170,
1200,1230,1260,1290,1320,1350,1380,1410,
1440,1470,1500,1530,1560,1590,1620,1650,
1680,1710,1740,1770,1800,1830,1860,1890,

-1920,-1890,-1860,-1830,-1800,-1770,-1740,-1710,
-1680,-1650,-1620,-1590,-1560,-1530,-1500,-1470,
-1440,-1410,-1380,-1350,-1320,-1290,-1260,-1230,
-1200,-1170,-1140,-1110,-1080,-1050,-1020,-990,//
-960,-930,-900,-870,-840,-810,-780,-750,
-720,-690,-660,-630,-600,-570,-540,-510,
-480,-450,-420,-390,-360,-330,-300,-270,
-240,-210,-180,-150,-120,-90,-60,-30,
};

#else
 // for zero debounce.
int xyz_g_table[64] = {
0,0,0,141,188,234,281,328,
375,422,469,516,563,609,656,703,
750,797,844,891,938,984,1031,1078,
1125,1172,1219,1266,1313,1359,1406,1453,
-1500,-1453,-1406,-1359,-1313,-1266,-1219,-1172,
-1125,-1078,-1031,-984,-938,-891,-844,-797,
-750,-703,-656,-609,-563,-516,-469,-422,
-375,-328,-281,-234,-188,-141,0,0
};
#endif

#if 1 //did not use degree
int xyz_degree_table[64][2] = {
	[0] = {0, 9000},
	[1] = {269,8731},
	[2] = {538,8462},
	[3] = {808,8192},
	[4] = {1081,7919},
	[5] = {1355,7645},
	[6] = {1633,7367},
	[7] = {1916,7084},
	[8] = {2202,6798},
	[9] = {2495,6505},
	[10] = {2795,6205},
	[11] = {3104,5896},
	[12] = {3423,5577},
	[13] = {3754,5246},
	[14] = {4101,4899},
	[15] = {4468,4532},
	[16] = {4859,4141},
	[17] = {5283,3717},
	[18] = {5754,3246},
	[19] = {6295,2705},
	[20] = {6964,2036},
	[21] = {7986,1014},
#if 0 //urbetter
	/* These are invalaid from [22] to [42] invalaid */
	[22] = {0xFFFF,0xFFFF},
	[23] = {0xFFFF,0xFFFF},
	[24] = {0xFFFF,0xFFFF},
	[25] = {0xFFFF,0xFFFF},
	[26] = {0xFFFF,0xFFFF},
	[27] = {0xFFFF,0xFFFF},
	[28] = {0xFFFF,0xFFFF},
	[29] = {0xFFFF,0xFFFF},
	[30] = {0xFFFF,0xFFFF},
	[31] = {0xFFFF,0xFFFF},
	[32] = {0xFFFF,0xFFFF},
	[33] = {0xFFFF,0xFFFF},
	[34] = {0xFFFF,0xFFFF},
	[35] = {0xFFFF,0xFFFF},
	[36] = {0xFFFF,0xFFFF},
	[37] = {0xFFFF,0xFFFF},
	[38] = {0xFFFF,0xFFFF},
	[39] = {0xFFFF,0xFFFF},
	[40] = {0xFFFF,0xFFFF},
	[41] = {0xFFFF,0xFFFF},
	[42] = {0xFFFF,0xFFFF},
#else
	[22] = {9000,0},
	[23] = {9000,0},
	[24] = {9000,0},
	[25] = {9000,0},
	[26] = {9000,0},
	[27] = {9000,0},
	[28] = {9000,0},
	[29] = {9000,0},
	[30] = {9000,0},
	[31] = {9000,0},
	[32] = {9000,0},
	[33] = {-9000,0},
	[34] = {-9000,0},
	[35] = {-9000,0},
	[36] = {-9000,0},
	[37] = {-9000,0},
	[38] = {-9000,0},
	[39] = {-9000,0},
	[40] = {-9000,0},
	[41] = {-9000,0},
	[42] = {-9000,0},
#endif
	[43] = {-7986,-1014},
	[44] = {-6964,-2036},
	[45] = {-6295,-2705},
	[46] = {-5754,-3246},
	[47] = {-5283,-3717},
	[48] = {-4859,-4141},
	[49] = {-4468,-4532},
	[50] = {-4101,-4899},
	[51] = {-3754,-5246},
	[52] = {-3423,-5577},
	[53] = {-3104,-5896},
	[54] = {-2795,-6205},
	[55] = {-2495,-6505},
	[56] = {-2202,-6798},
	[57] = {-1916,-7084},
	[58] = {-1633,-7367},
	[59] = {-1355,-7645},
	[60] = {-1081,-7919},
	[61] = {-808,-8192},
	[62] = {-538,-8462},
	[63] = {-269,-8731},
};
#endif

#if 1 //did not use degree
int xyz_degree_table_8452[128][2] = {
	[0] = {0, 9000},
	[1] = {269,8731},
	[2] = {538,8462},
	[3] = {808,8192},
	[4] = {1081,7919},
	[5] = {1355,7645},
	[6] = {1633,7367},
	[7] = {1916,7084},
	[8] = {2202,6798},
	[9] = {2495,6505},
	[10] = {2795,6205},
	[11] = {3104,5896},
	[12] = {3423,5577},
	[13] = {3754,5246},
	[14] = {4101,4899},
	[15] = {4468,4532},
	[16] = {4859,4141},
	[17] = {5283,3717},
	[18] = {5754,3246},
	[19] = {6295,2705},
	[20] = {6964,2036},
	[21] = {7986,1014},	
	[22] = {9000,0},
	[23] = {9000,0},
	[24] = {9000,0},
	[25] = {9000,0},
	[26] = {9000,0},
	[27] = {9000,0},
	[28] = {9000,0},
	[29] = {9000,0},
	[30] = {9000,0},
	[31] = {9000,0},
	
	[32] = {9000,0},
	[33] = {-9000,0},
	[34] = {-9000,0},
	[35] = {-9000,0},
	[36] = {-9000,0},
	[37] = {-9000,0},
	[38] = {-9000,0},
	[39] = {-9000,0},
	[40] = {-9000,0},
	[41] = {-9000,0},
	[42] = {-9000,0},
	[43] = {9000,0},
	[44] = {9000,0},
	[45] = {9000,0},
	[46] = {9000,0},
	[47] = {9000,0},
	[48] = {9000,0},
	[49] = {9000,0},
	[50] = {9000,0},
	[51] = {9000,0},
	[52] = {9000,0},
	[53] = {9000,0},	
	[54] = {9000,0},
	[55] = {9000,0},
	[56] = {9000,0},
	[57] = {9000,0},
	[58] = {9000,0},
	[59] = {9000,0},
	[60] = {9000,0},
	[61] = {9000,0},
	[62] = {9000,0},
	[63] = {9000,0},
	
	[64] = {9000,0},
	[65] = {-9000,0},
	[66] = {-9000,0},
	[67] = {-9000,0},
	[68] = {-9000,0},
	[69] = {-9000,0},
	[70] = {-9000,0},
	[71] = {-9000,0},
	[72] = {-9000,0},
	[73] = {-9000,0},
	[74] = {-9000,0},
	[75] = {-9000,0},
	[76] = {-9000,0},
	[77] = {-9000,0},
	[78] = {-9000,0},
	[79] = {-9000,0},
	[80] = {-9000,0},
	[81] = {-9000,0},
	[82] = {-9000,0},
	[83] = {-9000,0},
	[84] = {-9000,0},
	[85] = {-9000,0},
	[86] = {-9000,0},
	[87] = {-9000,0},
	[88] = {-9000,0},
	[89] = {-9000,0},
	[90] = {-9000,0},
	[91] = {-9000,0},
	[92] = {-9000,0},
	[93] = {-9000,0},
	[94] = {-9000,0},
	[95] = {-9000,0},
	

	
	[96] = {-7986,-1014},
	[97] = {-6964,-2036},
	[98] = {-6295,-2705},
	[99] = {-5754,-3246},
	[100] = {-5283,-3717},
	[101] = {-4859,-4141},
	[102] = {-4468,-4532},
	[103] = {-4101,-4899},
	[104] = {-3754,-5246},
	[105] = {-3423,-5577},
	[106] = {-3104,-5896},
	[107] = {-2795,-6205},
	[108] = {-2495,-6505},
	[109] = {-2202,-6798},
	[110] = {-1916,-7084},
	[111] = {-1633,-7367},
	[112] = {-1355,-7645},
	[113] = {-1081,-7919},
	[114] = {-808,-8192},
	[115] = {-538,-8462},
	[116] = {-269,-8731},
	[117] = {-3104,-5896},
	[118] = {-2795,-6205},
	[119] = {-2495,-6505},
	[120] = {-2202,-6798},
	[121] = {-1916,-7084},
	[122] = {-1633,-7367},
	[123] = {-1355,-7645},
	[124] = {-1081,-7919},
	[125] = {-808,-8192},
	[126] = {-538,-8462},
	[127] = {-269,-8731},
	
};
#endif

unsigned char mma7660_i2c_read(unsigned char reg)
{
	if(!s_i2c_client)
	{
		printk("[%s]s_i2c_client is not ready\n", __FUNCTION__);
		return -1;
	}
	return i2c_smbus_read_byte_data(s_i2c_client, reg);
}

int mma7660_i2c_write(unsigned char reg, unsigned char data)
{
	if(!s_i2c_client)
	{
		printk("[%s]s_i2c_client is not ready\n", __FUNCTION__);
		return -1;
	}
	return i2c_smbus_write_byte_data(s_i2c_client, reg, data);
}

static int mma7660_i2c_write_bytes(struct i2c_client *client,uint8_t *data,int len)
{
        struct i2c_msg msg;
        int ret=-1;

        msg.flags=!I2C_M_RD;
        msg.addr=client->addr;
        msg.len=len;
        msg.buf=data;

        ret=i2c_transfer(client->adapter,&msg,1);
        return ret;
}

static void mma8452_chip_init(void)
{
	mma7660_i2c_write(0x2A, 0x3);	//set it standby to write.
//mma7660	
	mma7660_i2c_write(0x07, 0);	//set it standby to write.
	mma7660_i2c_write(0x05, 0);
	mma7660_i2c_write(0x06, 0);
	mma7660_i2c_write(0x08, 0xFB);
	mma7660_i2c_write(0x0A, 0xFF);
 	mma7660_i2c_write(0x07, 0xF9);	//write finished, resume.
//end		 	
	printk("mma8452_chip_init\n");
}
static void mma7660_chip_init(void)
{
	if(mma7660_chip)
		{
			mma7660_i2c_write(0x07, 0);	//set it standby to write.
			mma7660_i2c_write(0x05, 0);
			mma7660_i2c_write(0x06, 0);
			mma7660_i2c_write(0x08, 0xFB);
		//	mma7660_i2c_write(0x08, 0xF8);
		//	mma7660_i2c_write(0x09, 0x1F);
			mma7660_i2c_write(0x0A, 0xFF);
		 	mma7660_i2c_write(0x07, 0xF9);	//write finished, resume.

			printk("mma7660_chip_init\n");
		}
	else
		mma8452_chip_init();
}



static int mma8452_poll_thread(void * data)
{
	int x, y, z;
	while(1)
	{
/*	
		if(!s_mma7660_suspend) //raymanfeng for early syuspend
		{
			s_x = x = 0x7F - mma7660_i2c_read(1)/2;
			s_y = y = mma7660_i2c_read(3)/2;
			s_z = z = mma7660_i2c_read(5)/2;
		}
*/		
		if(!s_mma7660_suspend) //raymanfeng for early syuspend
		{
//			s_x = x = mma7660_i2c_read(0x1)/2;
//			s_y = y = mma7660_i2c_read(0x0)/2;
//			s_z = z = mma7660_i2c_read(0x2)/2 + 5;

			
			s_x = x = mma7660_i2c_read(1) / 2;
			s_y = y = mma7660_i2c_read(3)/2;
			s_z = z = mma7660_i2c_read(5)/2;

			//printk("Got gsensor data!\n");
		}
	
		//printk("(%d,%d,%d)\n",x,y,z);
/*		
		if (s_x == 63 || s_x == 1)
			s_x = 0;
		if (s_y == 63 || s_y == 1)
			s_y = 0;
		if (s_z == 63 || s_z == 1)
			s_z = 0;
*/		
		msleep(50);
		//msleep(100);
	}
	return 0;
}

static struct i2c_device_id mma7660_idtable[] = {
	{"mma7660", 0},
};

static struct i2c_driver mma7660_i2c_driver = 
{
	.driver = 
	{
		.name = "mma7660_i2c",
		.owner = THIS_MODULE,
	},
	.id_table = mma7660_idtable,
	.probe = mma7660_i2c_probe,
#if 0  //raymanfeng: moved to early suspend
	.suspend = mma7660_i2c_suspend,
	.resume = mma7660_i2c_resume,
#endif
};

static struct platform_device mma7660_platform_i2c_device = 
{
    .name           = "mma7660_i2c",
    .id             = -1,
};


static int __devinit mma7660_i2c_probe(struct i2c_client * client, const struct i2c_device_id * id)
{
	int i = -1;
	s_i2c_client = client;

	printk("mma7660_i2c_probe\n");
	printk("mma7660: s_i2c_client->addr = 0x%x\n",s_i2c_client->addr);
	
	i = mma7660_i2c_write(0x01,0x0);

//	i = mma7660_i2c_read(0x18);
	
	if(i < 0)
	{
		printk("mma8452\n");
//		printk("mc3230\n");
		s_i2c_client->addr = 0x1C;
		mma7660_chip = 0;
	}
	else
	{
		printk("mma7660\n");
		mma7660_chip = 1;
	}
	
	mma7660_chip_init();
	mma7660_read_thread(1);	
	
	return 0;
}

#if 0 //raymanfeng: moved to early suspend
static int mma7660_i2c_suspend(struct i2c_client * client, pm_message_t mesg)
{
	mma7660_i2c_write(0x07, 0);
	return 0;
}

static int mma7660_i2c_resume(struct i2c_client * client)
{
	mma7660_chip_init();
	return 0;
}
#endif

static int mma7660_open(struct inode *inode, struct file *file)
{
	//printk("[%s,%d]\n", __FUNCTION__, __LINE__);
	mma7660_chip_init();
	return nonseekable_open(inode, file);
}

static int mma7660_release(struct inode *inode, struct file *file)
{
	return 0;
}

#if 0
static void mma7660_work()
{
	int x, y, z;
	
		x = mma7660_i2c_read(MMA7660_CHANNEL_X);
		y = mma7660_i2c_read(MMA7660_CHANNEL_Y);
		z = mma7660_i2c_read(MMA7660_CHANNEL_Z);

		if(x < 0 || y < 0 || z < 0)
		{
			//printk("mma7660 read error, delay\n");
			//msleep(2000);
		}
		s_x = x & MMA7660_DATA_MASK;
		s_y = y & MMA7660_DATA_MASK;
		s_z = z & MMA7660_DATA_MASK;
}
#endif

//extern volatile int MmFlag;  // masked andydeng 20120731
static int mma7660_poll_thread(void * data)
{
	int x, y, z;
	int x_buf[15], y_buf[15], z_buf[15], i, tmp; 
	if(mma7660_chip)
		{
	while(1)
	{
#if 0
		for(i = 0; i < 15; i++) {
			x_buf[i] = mma7660_i2c_read(MMA7660_CHANNEL_X) & MMA7660_DATA_MASK;
		//for(i = 0; i < 5; i++)
			y_buf[i] = mma7660_i2c_read(MMA7660_CHANNEL_Y) & MMA7660_DATA_MASK;
		//for(i = 0; i < 5; i++)
			z_buf[i] = mma7660_i2c_read(MMA7660_CHANNEL_Z) & MMA7660_DATA_MASK;
		}

		for(i = 0; i < (15 - 1); i++)
		{
			if(x_buf[i] > x_buf[i + 1]) 
			{
				tmp = x_buf[i];
				x_buf[i] = x_buf[i + 1];
				x_buf[i + 1] = tmp;
			}
			if(y_buf[i] > y_buf[i + 1])
			{
				tmp = y_buf[i];
				y_buf[i] = y_buf[i + 1];
				y_buf[i + 1] = tmp;
			}
			if(z_buf[i] > z_buf[i + 1])
			{
				tmp = z_buf[i];
				z_buf[i] = z_buf[i + 1];
				z_buf[i + 1] = tmp;
			}
		}
		x = x_buf[7];
		y = y_buf[7];
		z = z_buf[7];
#else
     //   if(MmFlag)  //masked andydeng 20120731
        {
		x = mma7660_i2c_read(MMA7660_CHANNEL_X);
		y = mma7660_i2c_read(MMA7660_CHANNEL_Y);
		z = mma7660_i2c_read(MMA7660_CHANNEL_Z);
		//printk("[%s,%d]Got Gsensor Data:(%d,%d,%d)\n",x,y,z, __FUNCTION__, __LINE__);
       }
#endif

		if(x < 0 || y < 0 || z < 0)
		{
			//printk("mma7660 read error, delay\n");
			//msleep(2000);
		}
		s_x = x & MMA7660_DATA_MASK;
		s_y = y & MMA7660_DATA_MASK;
		s_z = z & MMA7660_DATA_MASK;

		//printk("[%s,%d](%d,%d,%d)\n",s_x,s_y,s_z,__FUNCTION__, __LINE__);
/*		
		if (s_x == 63 || s_x == 1)
			s_x = 0;
		if (s_y == 63 || s_y == 1)
			s_y = 0;
		if (s_z == 63 || s_z == 1)
			s_z = 0;
*/		
		msleep(50);
	}
		}
	else
		mma8452_poll_thread((void *)1);
	return 0;
}
/*
static void mma7660_poll_callback(struct work_struct *work)
{
	static volatile unsigned long s_last_jiffies = 0;
	if(jiffies - s_last_jiffies > ms_to_jiffies(MIN_POLL_INTERVAL))
	{
		//printk("*");
		s_x = mma7660_i2c_read(MMA7660_CHANNEL_X) & MMA7660_DATA_MASK;
		s_y = mma7660_i2c_read(MMA7660_CHANNEL_Y) & MMA7660_DATA_MASK;
		s_z = mma7660_i2c_read(MMA7660_CHANNEL_Z) & MMA7660_DATA_MASK;
		s_last_jiffies = jiffies;
	}
}
*/
static int mma7660_poll_date(int * x, int * y, int * z)
{
	*x = s_x;
	*y = s_y;
	*z = s_z;
	//flush_work(&mma7660_poll_work);
	//schedule_work(&mma7660_poll_work);
	return 0;
}

ssize_t mma7660_read(struct file *filp, char __user *buf,
			     size_t count, loff_t *ppos)
{
	int readBuf[3];
	int outBuf[8];
	mma7660_poll_date(&readBuf[0], &readBuf[1], &readBuf[2]);
	if (mma7660_chip)
		{
	if (712 == s_ts_model)
		{
			outBuf[0] = xyz_g_table_712[readBuf[0]] * MMA7660_DIR_X;
			//outBuf[1] = xyz_g_table_712[readBuf[1]] * MMA7660_DIR_Y;
			outBuf[1] = xyz_g_table_712_y[readBuf[1]] * MMA7660_DIR_Y;
			outBuf[2] = xyz_g_table_712[readBuf[2]] * MMA7660_DIR_Z;
			outBuf[3] = 65536 + 1;
		}
	else
		{
			outBuf[0] = xyz_g_table[readBuf[0]] * MMA7660_DIR_X;
			outBuf[1] = xyz_g_table[readBuf[1]] * MMA7660_DIR_Y;
			outBuf[2] = xyz_g_table[readBuf[2]] * MMA7660_DIR_Z;
			outBuf[3] = 65536 + 1;

				}
		}
	else
		{
			outBuf[0] = xyz_g_table_8452[readBuf[0]] * MMA7660_DIR_X;
			outBuf[1] = xyz_g_table_8452[readBuf[1]] * MMA7660_DIR_Y;
			outBuf[2] = xyz_g_table_8452[readBuf[2]] * MMA7660_DIR_Z;
			outBuf[3] = 65536 + 1;
		}

	outBuf[4] = xyz_degree_table[readBuf[0]][0] * MMA7660_DIR_X;
	outBuf[5] = xyz_degree_table[readBuf[1]][0] * MMA7660_DIR_Y;
	outBuf[6] = xyz_degree_table[readBuf[2]][0] * MMA7660_DIR_Z;
	outBuf[7] = 65536 + 1;

	if(copy_to_user(buf, outBuf, sizeof(outBuf)))
		return -EFAULT;
	return sizeof(outBuf);
}

static void mma7660_read_thread(int run)
{
	if(run && (!s_kthread_run)) 
	{	
		kthread_run(mma7660_poll_thread, s_i2c_client, "mma7660_poll");
		s_kthread_run = 1;
	}
	//else
}

static int mma7660_writeproc(struct file *file,const char *buffer,
                           unsigned long count, void *data)
{
	if (*buffer == 'a')
	{
		printk("s_i2c_client:%d\n", s_i2c_client);
	}
	
	return count;
}

static int mma7660_readproc(char *page, char **start, off_t off,
			  int count, int *eof, void *data)
{
	int len = 0;
	int x, y, z;
	mma7660_poll_date(&x, &y, &z);
	len = sprintf(page, "x=%d\ny=%d\nz=%d\nx_degree=%d\ny_degree=%d\nz_degree=%d\n", 
				  xyz_g_table[x] * MMA7660_DIR_X, 
				  xyz_g_table[y] * MMA7660_DIR_Y, 
				  xyz_g_table[z] * MMA7660_DIR_Z,
				  xyz_degree_table[x][0] * MMA7660_DIR_X,
				  xyz_degree_table[y][0] * MMA7660_DIR_Y,
				  xyz_degree_table[z][0] * MMA7660_DIR_Z);
	return len;
}

static struct file_operations mma7660_fops = {
	.owner = THIS_MODULE,
	.open = mma7660_open,
	.release = mma7660_release,
	.read = mma7660_read,
};

static struct miscdevice mma7660_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mma7660",
	.fops = &mma7660_fops,
};


void ut_mma7660_early_suspend(struct early_suspend *h)
{
	mma7660_i2c_write(0x07, 0);
	s_mma7660_suspend = 1;
}

void ut_mma7660_late_resume(struct early_suspend *h)
{
	//printk("[%s,%d]\n", __FUNCTION__, __LINE__);
	mma7660_chip_init();
	s_mma7660_suspend = 0;
}

static void ut_init_mma7660_early_suspend(void)
{
	s_mma7660_early_suspend.suspend = ut_mma7660_early_suspend;
	s_mma7660_early_suspend.resume = ut_mma7660_late_resume;
	s_mma7660_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN/* + 1*/;
	register_early_suspend(&s_mma7660_early_suspend);
}

static int __init mma7660_init(void)
{
    int ret;
	printk(KERN_INFO "mma7660 G-Sensor driver: init\n");

	check_ts_model();

	ret = platform_device_register(&mma7660_platform_i2c_device);
    if(ret)
        return ret;

	s_proc = create_proc_entry(GSENSOR_PROC_NAME, 0666, &proc_root);
	if (s_proc != NULL)
	{
		s_proc->write_proc = mma7660_writeproc;
		s_proc->read_proc = mma7660_readproc;
	}

	ret = misc_register(&mma7660_device);
	if (ret) {
		printk("mma7660_probe: mma7660_device register failed\n");
		return ret;
	}

	ut_init_mma7660_early_suspend(); //raymanfeng

	//mma7660_read_thread(1);
	return i2c_add_driver(&mma7660_i2c_driver); 
	 
}

static void __exit mma7660_exit(void)
{
	if (s_proc != NULL)
		remove_proc_entry(GSENSOR_PROC_NAME, &proc_root);
	misc_deregister(&mma7660_device);
	i2c_del_driver(&mma7660_i2c_driver);
    platform_device_unregister(&mma7660_platform_i2c_device);
}

//module_init(mma7660_init);
late_initcall(mma7660_init);
module_exit(mma7660_exit);
MODULE_AUTHOR("urbetter inc.");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("urbetter mma7660 G-sensor");

