//
// Created by ShiF on 2024/1/10.
//

#include "IMUC.hpp"

cimu IMU;

float cimu::cAngle(cimu::eImuType imu_type, cimu::eImuFun imu_fun)
{
    switch (imu_type)
    {
        case C_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return C_IMU_Angle(0);
                case Pitch:
                    return C_IMU_Angle(2);
                case Roll:
                    return C_IMU_Angle(1);
            }
            break;
    }
    return 0;
}

float cimu::cSpeed(cimu::eImuType imu_type, cimu::eImuFun imu_fun)
{
    switch (imu_type)
    {
        case C_imu:
            switch (imu_fun)
            {
                case Yaw:
                    return C_IMU_Speed(0);
                case Pitch:
                    return C_IMU_Speed(2);
                case Roll:
                    return C_IMU_Speed(1);
            }
            break;
    }
    return 0;
}
