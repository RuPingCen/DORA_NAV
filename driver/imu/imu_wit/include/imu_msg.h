#ifndef IMU_MSG_H
#define IMU_MSG_H

#include "Vector3.h"

namespace cslam {
struct ImuMsg {
    double stamp;
	struct Vector3 linear_acceleration;
	struct Vector3 angular_velocity;
};
}

#endif