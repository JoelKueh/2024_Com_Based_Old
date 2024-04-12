// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Constants.h"

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>

// Rev Includes
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxRelativeEncoder.h>

class Drive : public frc2::SubsystemBase {
public:
	Drive();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

private:
	rev::CANSparkMax m_fl_motor {
		Constants::Drive::k_front_left_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkMaxRelativeEncoder m_fl_encoder = m_fl_motor.GetEncoder();

	rev::CANSparkMax m_fr_motor {
		Constants::Drive::k_front_right_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkMaxRelativeEncoder m_fr_encoder = m_fr_motor.GetEncoder();

	rev::CANSparkMax m_bl_motor {
		Constants::Drive::k_back_left_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkMaxRelativeEncoder m_bl_encoder = m_bl_motor.GetEncoder();

	rev::CANSparkMax m_br_motor {
		Constants::Drive::k_back_right_id,
		rev::CANSparkMax::MotorType::kBrushless
	};
	rev::SparkMaxRelativeEncoder m_br_encoder = m_br_motor.GetEncoder();

	frc::MecanumDrive *m_mecanums;
};
