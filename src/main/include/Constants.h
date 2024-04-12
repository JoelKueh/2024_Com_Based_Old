// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

#include <units/length.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/voltage.h>

namespace Constants {

struct pid_constants {
	double kp;
	double ki;
	double kd;
	units::volt_t ks;
	units:: kv;
	double ka;
};

namespace Operator {

constexpr int kDriverControllerPort = 0;

}  // namespace OperatorConstants

namespace Drive {

constexpr int k_front_left_id	= 1;
constexpr int k_front_right_id	= 2;
constexpr int k_back_left_id 	= 3;
constexpr int k_back_right_id	= 4;

constexpr double k_rot_to_m	= 0.00797964534 / 9.13;
constexpr double k_rpm_to_mps	= 0.4787787204 / 9.13;

const struct pid_constants k_fl_pid = {
	0.70234,
	0.0,
	0.0,
	0.30923,

}

namespace Shooter {

constexpr int k_left_id	 = 5;
constexpr int k_right_id = 6;

}

namespace Elevator {

constexpr int k_intake_upper_id = 7;
constexpr int k_intake_lower_id = 8;
constexpr int k_motor_id	= 9;

}

namespace Hanger {

constexpr int k_pneumatics_hub_id = 10;
constexpr int k_forward_id	  =  1;
constexpr int k_reverse_id	  =  0;

}

}
