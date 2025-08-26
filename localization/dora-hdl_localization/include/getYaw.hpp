#ifndef GET_YAW_H
#define GET_YAW_H

#include <Eigen/Dense>  
#include <cmath>        


float getYaw(const Eigen::Matrix3d &rotationMatrix)
{
    struct Euler
    {
        float yaw;
        float pitch;
        float roll;
    };

    Euler euler_out;
    if(std::fabs(rotationMatrix(2, 0)) >= 1.0)
    {
        euler_out.yaw = 0;
    }
    else
    {
        euler_out.pitch = -std::asin(rotationMatrix(2, 0));

        euler_out.yaw = std::atan2(rotationMatrix(1, 0) / std::cos(euler_out.pitch), 
                                rotationMatrix(0, 0) / std::cos(euler_out.pitch));
    }



    return euler_out.yaw;  
}

#endif // GET_YAW_H