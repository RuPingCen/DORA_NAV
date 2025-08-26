#ifndef IMU_MSG_H
#define IMU_MSG_H

namespace canslam
{
    struct Vector3
    {
        float x;
        float y;
        float z;
    };

    struct imu_msg_h
    {
        double stamp;
        Vector3 linear_acceleration;
        Vector3 angular_velocity;
    };
}

#endif