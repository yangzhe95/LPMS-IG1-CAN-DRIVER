#ifndef _CAN_IMU_H
#define _CAN_IMU_H

#include <vector>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <sys/time.h>
#include <signal.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <sensor_msgs/Imu.h>


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <zlgcan/zlgcan.h>
#include <iomanip>

#define dec 32767.0f
#define cDegToRad 3.1415926f/180.0f
#define cEarthG 9.81f
//#define base_id  181h
unsigned int all_id[5]={0x181,0x281,0x381,0x481,0x701};
bool data_flag[10]{};
sensor_msgs::Imu imu_data;


float unsign2float1(can_frame frame)
{
    unsigned char s1[2];
    s1[0]=frame.data[0];
    float s10 = (float)s1[0];
    s1[1]=frame.data[1];
    float s11 = (float)s1[1];
    float s12 = s11*256+s10; 
    if (s12>dec)
    {
        s12 = s12 - 65536;
    }
    return s12;
}
float unsign2float2(can_frame frame)
{
    unsigned char s2[2];
    s2[0]=frame.data[2];
    float s20 = (float)s2[0];
    s2[1]=frame.data[3];
    float s21 = (float)s2[1];
    float s22 = s21*256+s20; 
    if (s22>dec)
    {
        s22 = s22 - 65536;
    }

    return s22;
}
float unsign2float3(can_frame frame)
{
    unsigned char s3[2];
    s3[0]=frame.data[4];
    float s30 = (float)s3[0];
    s3[1]=frame.data[5];
    float s31 = (float)s3[1];
    float s32 = s31*256+s30; 
    if (s32>dec)
    {
        s32 = s32 - 65536;
    }

    return s32;
}
float unsign2float4(can_frame frame)
{
    unsigned char s4[2];
    s4[0]=frame.data[6];
    float s40 = (float)s4[0];
    s4[1]=frame.data[7];
    float s41 = (float)s4[1];
    float s42 = s41*256+s40; 
    if (s42>dec)
    {
        s42 = s42 - 65536;
    }

    return s42;
}

void can2imu(can_frame frame)
{
    unsigned int canid=frame.can_id;
    int id=5;
    for(int i=0;i<5;i++)
        if(canid==all_id[i])
        {
            id=i;break;
        }
    switch(id){
    float ax, ay, az, avx, avy, avz, ox, oy, oz, ow;
        case 0:
            ax = unsign2float1(frame);
            ax = -ax/1000.0*cEarthG;
            imu_data.linear_acceleration.x = ax;
            data_flag[4] = true;
            ay = unsign2float2(frame);
            ay = -ay/1000.0*cEarthG;
            imu_data.linear_acceleration.y = ay;
            data_flag[5] = true;
            az = unsign2float3(frame);
            az = -az/1000.0*cEarthG;
            imu_data.linear_acceleration.z = az;
            data_flag[6] = true;

            break;
	    case 1:
            break;
        case 2:
            avx = unsign2float2(frame);
            avx = avx/10*cDegToRad;
            imu_data.angular_velocity.x = avx;
            data_flag[7] = true;
            avy = unsign2float3(frame);
            avy = avy/10*cDegToRad;
            imu_data.angular_velocity.y = avy;
            data_flag[8] = true;
            avz = unsign2float4(frame);
            avz = avz/10*cDegToRad;
            imu_data.angular_velocity.z = avz;
            data_flag[9] = true;

            break;
        case 3:
            ox = unsign2float2(frame);
            ox = -ox/10000.0;
            imu_data.orientation.x = ox;
            data_flag[0] = true;
            oy = unsign2float3(frame);
            oy = -oy/10000.0;
            imu_data.orientation.y = oy;
            data_flag[1] = true;
            oz = unsign2float4(frame);
            oz = -oz/10000.0;
            imu_data.orientation.z = oz;
            data_flag[2] = true;
            ow = unsign2float1(frame);
            ow = ow/10000.0;
            imu_data.orientation.w = ow;
            data_flag[3] = true;
            break;
	    case 4:
	        break;
        default:
            std::cout<<"There is no ID matching :"<< canid <<std::endl;
            break;
    }
}

bool data_ready()
{
    for(int i=0;i<10;i++)
        if(data_flag[i]==false)
            return 0;
    Eigen::Quaterniond q;

    return 1;
}
bool clear_flag()
{
    for(int i=0;i<10;i++)
        data_flag[i]=false;
}

#endif
