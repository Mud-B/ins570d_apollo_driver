#ifndef OFFSET_DEFINE_H
#define OFFSET_DEFINE_H

#endif // OFFSET_DEFINE_H

#pragma once        //这个要有，否则提示重复定义

#include <iostream>
#include <values.h>

///offset Def
#define HEADER_1 0
#define HEADER_2 1
#define HEADER_3 2
#define ROLL_OFFSET 3
#define PITCH_OFFSET 5
#define YAW_OFFSET 7
#define GYRO_X_OFFSET 9
#define GYRO_Y_OFFSET 11
#define GYRO_Z_OFFSET 13
#define ACC_X_OFFSET 15
#define ACC_Y_OFFSET 17
#define ACC_Z_OFFSET 19
#define LATITUDE_OFFSET 21
#define LONGITUDE_OFFSET 25
#define ALTITUDE_OFFSET 29
#define VEL_N_OFFSET 33
#define VEL_E_OFFSET 35
#define VEL_D_OFFSET 37
#define STATUS_OFFSET 39
#define DATA_1_OFFSET 46
#define DATA_2_OFFSET 48
#define DATA_3_OFFSET 50
#define TIME_OFFSET 52
#define TYPE_OFFSET 56
#define CHECKSUM1_OFFSET 57
#define GPSWEEK_OFFSET 58
#define CHECKSUM2_OFFSET 62

///GPS STATUS Def
// #define GPS_NONE            0   //无解
// #define FIXEDPOS            1
// #define FIXEDHEIGHT         2
// #define DOPPLER_VELOCITY    8   //速度由即时多普勒信息导出
// #define SINGLE              16  //单点定位
// #define PSRDIFF             17  //伪距差分定位(Pseudorange differential solution)
// #define SBAS                18  //SBAS 定位(Solution calculated using corrections from an SBAS)
// #define L1_FLOAT            32  //L1 浮动解（Floating L1 ambiguity solution）
// #define IONOFREE_FLOAT      33  //消电离层浮点解(Floating ionospheric-free ambiguity solution)
// #define NARROW_FLOAT        34  //窄巷浮点解(Floating naeeow-lane ambiguity solution)
// #define L1_INT              48  //L1 固定解(Integer L1 ambiguity solution)
// #define WIDE_INT            49  //Integer wide-lane ambiguity solution
// #define NARROW_INT          50  //RTK FIX

/// Resolution Def
#define ANGLE_RESOLUTION    0.010986328125
#define GYRO_RESOLUTION     0.0091552734375
#define ACC_RESOLUTION      0.0003662109375
#define WGS_RESOLUTION      1.00E-07
#define ALTITUDE_RESOLUTION 1.00E-03
#define VEL_RESOLUTION      0.0030517578125
#define TIMESTAMP_RESOLUTION    2.50E-04
#define TEMPETURE_RESOLUTION    0.006103515625

extern int config_[62][2];

enum aligen_status{
    NOT_ALIGEN=0,
    ALIGNED=1};

enum data_type{
    POSI_PRECISION=0,
    VEL_PRECISION=1,
    POSE_PRECISION=2,
    TEMPERATURE=22,
    GPS_STATUS=32,
    WHEEL_SPEED_STATUS=33
};


//echo -e -n "\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xBD\xDB\x0B\x4A\xFF\xAA\x00\x00" >> /dev/pts/7
