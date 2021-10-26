#include <can_imu.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <zlgcan/zlgcan.h>


extern sensor_msgs::Imu imu_data;

int main(int argc, char* argv[]) { 
    ros::init(argc, argv, "serial_imu_node"); 
    ros::NodeHandle nh; 
    ros::Publisher IMU_read_pub = nh.advertise<sensor_msgs::Imu>("imu_data", 1); 
    DEVICE_HANDLE  dHandle;
    CHANNEL_HANDLE cHandle;
    ZCAN_Receive_Data zrd;
    ZCAN_DEVICE_INFO  dinfo;
    ZCAN_CHANNEL_ERR_INFO einfo;
    IProperty *property;
    UINT  ch;
    int ret;
    char path[128];
    int i;
    if (argc < 2)
        ch = 1;//can channel number
    else 
        ch = atoi(argv[1]);

    dHandle = ZCAN_OpenDevice(ZCAN_USBCAN_4E_U, 0x0, 0);

    if (dHandle == INVALID_DEVICE_HANDLE) {
        printf("ZCAN_OpenDevice failed\n");
        return -1;
    }
    ret = ZCAN_GetDeviceInf(dHandle, &dinfo);
    if (ret < 0) {
        printf("ZCAN_GetDeviceInf failed\n");
        ZCAN_CloseDevice(dHandle);
        return -1;
    } else
        printf("ZCAN_GetDeviceInf Version:(%d:%d),Serial: %s\n",  dinfo.hw_Version, dinfo.fw_Version ,dinfo.str_Serial_Num);
    	property = GetIProperty(dHandle);


    cHandle = ZCAN_InitCAN(dHandle, ch, NULL);
    if (cHandle == INVALID_CHANNEL_HANDLE) {
        printf("ZCAN_InitCAN failed\n");
        ZCAN_CloseDevice(dHandle);
        return -1;
    }
    ZCAN_ResetCAN(cHandle);
    snprintf(path, 128, "info/channel/channel_%d/baud_rate", ch);
    property->SetValue(path, "500000");//set boud_rate

    snprintf(path, 128, "info/channel/channel_%d/work_mode", ch);
    property->SetValue(path, "0");


    ret = ZCAN_StartCAN(cHandle);
    if (ret < 0) {
        printf("ZCAN_StartCAN failed\n");
        ZCAN_CloseDevice(dHandle);
        return -1;
    }


    while (1) {
		
        memset(&zrd, 0x0, sizeof(ZCAN_Receive_Data));
        ret = ZCAN_Receive(cHandle, &zrd, 1, 10000);
        if (ret == 0) {
            printf("ZCAN_Receive Timeout\n");
            break;      
        } else if (ret < 0) {
            printf("ZCAN_Receive failed\n");
            break;
        } else  {
			can2imu(zrd.frame);
			if(data_ready()&&ros::ok())
			{
			    imu_data.header.stamp = ros::Time::now();
			    imu_data.header.frame_id = "imu";
	            IMU_read_pub.publish(imu_data);
	            clear_flag();
			}
        }
        if (ZCAN_ReadChannelErrInfo(cHandle, &einfo) == 1) 
            printf("ZCAN_ReadChannelErrInfo Read a Error(error_code: 0x%x)\n", einfo.error_code);
    }

    ZCAN_ResetCAN(cHandle);
    ReleaseIProperty(property);
    ZCAN_CloseDevice(dHandle);
    return 0;
}









