#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>
#include <math.h>
/***************************************************************
Copyright © ALIENTEK Co., Ltd. 1998-2029. All rights reserved.
文件名		: ap3216cApp.c
作者	  	: 左忠凯
版本	   	: V1.0
描述	   	: ap3216c设备测试APP。
其他	   	: 无
使用方法	 ：./ap3216cApp /dev/ap3216c
论坛 	   	: www.openedv.com
日志	   	: 初版V1.0 2019/9/20 左忠凯创建
***************************************************************/

/*
 * @description		: main主程序
 * @param - argc 	: argv数组元素个数
 * @param - argv 	: 具体参数
 * @return 			: 0 成功;其他 失败
 */

float roll,pitch,yaw;

int main(int argc, char *argv[])
{
	float Ax,Ay,Az=0;
	float Gx,Gy,Gz=0;
	static float Gyroscope_roll=0;
	static float Gyroscope_pitch=0;
    const static float dt=0.005;
    const static float weight = 0.95;//权重
	int fd;
	char *filename;
	short databuf[6];
	int ret = 0;

	if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

	filename = argv[1];
	fd = open(filename, O_RDWR);

	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}
	printf("Sucessfully open file %s\r\n", filename);
	while (1) {
		ret = read(fd, databuf, sizeof(databuf));
		if(ret == 0) { 			/* 数据读取成功 */

			Ax = databuf[0]* 0.00048828125;
			Ay = databuf[1]* 0.00048828125;
			Az = databuf[2]* 0.00048828125;
			Gx = databuf[3]* dt*0.0174533;
			Gy = databuf[4]* dt*0.0174533;
			Gz = databuf[5]* dt*0.0174533;

			Gyroscope_roll+=Gy;
			Gyroscope_pitch+=Gx;

			roll=weight * atan2(Ay,Az)*57.295779513 + (1-weight) * Gyroscope_roll;

			pitch=(weight * atan2(Ax,Az)*57.295779513 + (1-weight) * Gyroscope_pitch);
			
			yaw += Gz*20+0.0157;


			printf("Roll:%.2f,pitch=%.2f,yaw=%.2f\n",roll,pitch,yaw);

		}
	}
	close(fd);	/* 关闭文件 */	
	return 0;
}

