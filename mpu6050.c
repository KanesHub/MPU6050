#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ide.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/of_gpio.h>
#include <linux/semaphore.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <asm/mach/map.h>
#include <asm/uaccess.h>
#include <asm/io.h>





#define OLED_CNT	1
#define OLED_NAME	"MPU6050"

#define	MPU6050_SMPLRT_DIV		0x19
#define	MPU6050_CONFIG			0x1A
#define	MPU6050_GYRO_CONFIG		0x1B
#define	MPU6050_ACCEL_CONFIG	0x1C

#define	MPU6050_ACCEL_XOUT_H	0x3B
#define	MPU6050_ACCEL_XOUT_L	0x3C
#define	MPU6050_ACCEL_YOUT_H	0x3D
#define	MPU6050_ACCEL_YOUT_L	0x3E
#define	MPU6050_ACCEL_ZOUT_H	0x3F
#define	MPU6050_ACCEL_ZOUT_L	0x40
#define	MPU6050_TEMP_OUT_H		0x41
#define	MPU6050_TEMP_OUT_L		0x42
#define	MPU6050_GYRO_XOUT_H		0x43
#define	MPU6050_GYRO_XOUT_L		0x44
#define	MPU6050_GYRO_YOUT_H		0x45
#define	MPU6050_GYRO_YOUT_L		0x46
#define	MPU6050_GYRO_ZOUT_H		0x47
#define	MPU6050_GYRO_ZOUT_L		0x48

#define	MPU6050_PWR_MGMT_1		0x6B
#define	MPU6050_PWR_MGMT_2		0x6C
#define	MPU6050_WHO_AM_I		0x75

struct mpu6050_dev {
	dev_t devid;			/* 设备号 	 */
	struct cdev cdev;		/* cdev 	*/
	struct class *class;	/* 类 		*/
	struct device *device;	/* 设备 	 */
	struct device_node	*nd; /* 设备节点 */
	int major;			/* 主设备号 */
	void *private_data;	/* 私有数据 */
	uint8_t MPU_ID;
};

static struct mpu6050_dev mpu6050dev;


static int mpu6050_read_regs(struct mpu6050_dev *dev, u8 reg, void *val, int len)
{
	int ret;
	struct i2c_msg msg[2];
	struct i2c_client *client = (struct i2c_client *)dev->private_data;

	/* msg[0]为发送要读取的首地址 */
	msg[0].addr = client->addr;			/* ap3216c地址 */
	msg[0].flags = 0;					/* 标记为发送数据 */
	msg[0].buf = &reg;					/* 读取的首地址 */
	msg[0].len = 1;						/* reg长度*/

	/* msg[1]读取数据 */
	msg[1].addr = client->addr;			/* ap3216c地址 */
	msg[1].flags = I2C_M_RD;			/* 标记为读取数据*/
	msg[1].buf = val;					/* 读取数据缓冲区 */
	msg[1].len = len;					/* 要读取的数据长度*/

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret == 2) {
		ret = 0;
	} else {
		printk("i2c rd failed=%d reg=%06x len=%d\n",ret, reg, len);
		ret = -EREMOTEIO;
	}
	return ret;
}

static s32 mpu6050_write_regs(struct mpu6050_dev *dev, u8 reg, u8 *buf, u8 len)
{
	u8 b[256];
	struct i2c_msg msg;
	struct i2c_client *client = (struct i2c_client *)dev->private_data;
	
	b[0] = reg;					/* 寄存器首地址 */
	memcpy(&b[1],buf,len);		/* 将要写入的数据拷贝到数组b里面 */
		
	msg.addr = client->addr;	/* ap3216c地址 */
	msg.flags = 0;				/* 标记为写数据 */

	msg.buf = b;				/* 要写入的数据缓冲区 */
	msg.len = len + 1;			/* 要写入的数据长度 */

	return i2c_transfer(client->adapter, &msg, 1);
}

static unsigned char mpu6050_read_reg(struct mpu6050_dev *dev, u8 reg)
{
	u8 data = 0;

	mpu6050_read_regs(dev, reg, &data, 1);
	return data;

}

static void mpu6050_write_reg(struct mpu6050_dev *dev, u8 reg, u8 data)
{
	u8 buf = 0;
	buf = data;
	mpu6050_write_regs(dev, reg, &buf, 1);
}

void mpu6050register_init(void)
{
	mpu6050_write_reg(&mpu6050dev, MPU6050_PWR_MGMT_1, 0x01);
	mpu6050_write_reg(&mpu6050dev, MPU6050_PWR_MGMT_2, 0x00);
	mpu6050_write_reg(&mpu6050dev, MPU6050_SMPLRT_DIV, 0x09);
	mpu6050_write_reg(&mpu6050dev, MPU6050_CONFIG, 0x06);
	mpu6050_write_reg(&mpu6050dev, MPU6050_GYRO_CONFIG, 0x18);
	mpu6050_write_reg(&mpu6050dev, MPU6050_ACCEL_CONFIG, 0x18);

}

void MPU6050_GetData(int16_t *AccX, int16_t *AccY, int16_t *AccZ, 
						int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ)
{
	uint8_t DataH, DataL;								//定义数据高8位和低8位的变量
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_XOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_XOUT_L);
	*AccX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_YOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_YOUT_L);
	*AccY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_ZOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_ACCEL_ZOUT_L);
	*AccZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_XOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_XOUT_L);
	*GyroX = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_YOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_YOUT_L);
	*GyroY = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
	DataH=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_ZOUT_H);
	DataL=mpu6050_read_reg(&mpu6050dev,MPU6050_GYRO_ZOUT_L);
	*GyroZ = (DataH << 8) | DataL;						//数据拼接，通过输出参数返回
}
/*
 * @description		: 打开设备
 * @param - inode 	: 传递给驱动的inode
 * @param - filp 	: 设备文件，file结构体有个叫做private_data的成员变量
 * 					  一般在open的时候将private_data指向设备结构体。
 * @return 			: 0 成功;其他 失败
 */
static int mpu6050_open(struct inode *inode, struct file *filp)
{

	mpu6050register_init();
	printk("MPUinit ... \n");
	mdelay(5);
	mpu6050dev.MPU_ID=mpu6050_read_reg(&mpu6050dev,MPU6050_WHO_AM_I);
	printk("MPU6050ID: %#.2x\n",mpu6050dev.MPU_ID);


	filp->private_data = &mpu6050dev;


	
	return 0;
}

/*
 * @description		: 从设备读取数据 
 * @param - filp 	: 要打开的设备文件(文件描述符)
 * @param - buf 	: 返回给用户空间的数据缓冲区
 * @param - cnt 	: 要读取的数据长度
 * @param - offt 	: 相对于文件首地址的偏移
 * @return 			: 读取的字节数，如果为负值，表示读取失败
 */


/*
 * @description		: 关闭/释放设备
 * @param - filp 	: 要关闭的设备文件(文件描述符)
 * @return 			: 0 成功;其他 失败
 */
static int mpu6050_release(struct inode *inode, struct file *filp)
{

	return 0;
}

static ssize_t mpu6050_write(struct file *filp, const char __user *buf, size_t cnt, loff_t *offt)
{
	return 0;
}

static ssize_t mpu6050_read(struct file *filp, char __user *buf, size_t cnt, loff_t *offt)
{
	struct mpu6050_dev *dev = (struct mpu6050_dev *)filp->private_data;
	int16_t data[6];
	long err = 0;
	MPU6050_GetData(&data[0],&data[1],&data[2],&data[3],&data[4],&data[5]);
	err = copy_to_user(buf, data, sizeof(data));
	return 0;
}

/* AP3216C操作函数 */
static const struct file_operations mpu6050_ops = {
	.owner = THIS_MODULE,
	.open = mpu6050_open,
	.release = mpu6050_release,
	.write = mpu6050_write,
	.read = mpu6050_read,
};



 /*
  * @description     : i2c驱动的probe函数，当驱动与
  *                    设备匹配以后此函数就会执行
  * @param - client  : i2c设备
  * @param - id      : i2c设备ID
  * @return          : 0，成功;其他负值,失败
  */
static int mpu6050_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("beep driver and device was matched!\r\n");
	/* 1、构建设备号 */
	if (mpu6050dev.major) {
		mpu6050dev.devid = MKDEV(mpu6050dev.major, 0);
		register_chrdev_region(mpu6050dev.devid, OLED_CNT, OLED_NAME);
	} else {
		alloc_chrdev_region(&mpu6050dev.devid, 0, OLED_CNT, OLED_NAME);
		mpu6050dev.major = MAJOR(mpu6050dev.devid);
	}

	/* 2、注册设备 */
	cdev_init(&mpu6050dev.cdev, &mpu6050_ops);
	cdev_add(&mpu6050dev.cdev, mpu6050dev.devid, OLED_CNT);

	/* 3、创建类 */
	mpu6050dev.class = class_create(THIS_MODULE, OLED_NAME);
	if (IS_ERR(mpu6050dev.class)) {
		return PTR_ERR(mpu6050dev.class);
	}

	/* 4、创建设备 */
	mpu6050dev.device = device_create(mpu6050dev.class, NULL, mpu6050dev.devid, NULL, OLED_NAME);
	if (IS_ERR(mpu6050dev.device)) {
		return PTR_ERR(mpu6050dev.device);
	}

	mpu6050dev.private_data = client;


	printk("Addr:%#.2x\n",client->addr);


	
	return 0;


}

/*
 * @description     : i2c驱动的remove函数，移除i2c驱动的时候此函数会执行
 * @param - client 	: i2c设备
 * @return          : 0，成功;其他负值,失败
 */
static int mpu6050_remove(struct i2c_client *client)
{
	/* 删除设备 */
	cdev_del(&mpu6050dev.cdev);
	unregister_chrdev_region(mpu6050dev.devid, OLED_CNT);

	/* 注销掉类和设备 */
	device_destroy(mpu6050dev.class, mpu6050dev.devid);
	class_destroy(mpu6050dev.class);
	return 0;
}

/* 传统匹配方式ID列表 */
static const struct i2c_device_id oled_id[] = {
	{"alientek,mpu6050", 0},  
	{}
};

/* 设备树匹配列表 */
static const struct of_device_id oled_of_match[] = {
	{ .compatible = "alientek,mpu6050" },
	{ /* Sentinel */ }
};

/* i2c驱动结构体 */	
static struct i2c_driver mpu6050_driver = {
	.probe = mpu6050_probe,
	.remove = mpu6050_remove,
	.driver = {
			.owner = THIS_MODULE,
		   	.name = "mpu6050",
		   	.of_match_table = oled_of_match, 
		   },
	.id_table = oled_id,
};
		   
/*
 * @description	: 驱动入口函数
 * @param 		: 无
 * @return 		: 无
 */
static int __init mpu6050_init(void)
{
	int ret = 0;

	ret = i2c_add_driver(&mpu6050_driver);
	return ret;
}

/*
 * @description	: 驱动出口函数
 * @param 		: 无
 * @return 		: 无
 */
static void __exit mpu6050_exit(void)
{
	i2c_del_driver(&mpu6050_driver);
}

/* module_i2c_driver(ap3216c_driver) */

module_init(mpu6050_init);
module_exit(mpu6050_exit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zuozhongkai");



