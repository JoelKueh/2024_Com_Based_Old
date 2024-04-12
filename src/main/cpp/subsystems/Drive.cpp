// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"

Drive::Drive()
{
	m_fl_motor.SetInverted(false);
	m_fr_motor.SetInverted(true);
	m_bl_motor.SetInverted(false);
	m_br_motor.SetInverted(true);

	m_fl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_fr_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_bl_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_br_motor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

	m_fl_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_fr_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_bl_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);
	m_br_encoder.SetPositionConversionFactor(Constants::Drive::k_rot_to_m);

	m_fl_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_fr_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_bl_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
	m_br_encoder.SetVelocityConversionFactor(Constants::Drive::k_rpm_to_mps);
}

// This method will be called once per scheduler run
void Drive::Periodic() {}
