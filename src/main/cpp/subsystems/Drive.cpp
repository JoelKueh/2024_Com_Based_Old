// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drive.h"
#include "frc2/command/Command.h"
#include "frc2/command/Commands.h"

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

	m_gyro.Calibrate();
	m_fl_encoder.SetPosition(0);
	m_fr_encoder.SetPosition(0);
	m_bl_encoder.SetPosition(0);
	m_br_encoder.SetPosition(0);
}

// This method will be called once per scheduler run
void Drive::Periodic()
{

}

/**
 * This design pattern is called a "command factory" we use arguments to
 * create a commmand that we will use later.
 *
 * This function creates a new command that binds an initial
 * position to a lambda expression that resets the pose estimator.
 * We could then schedule this command to reset our position.
 */
frc2::CommandPtr Drive::init_lps_command(frc::Pose2d init_pos)
{
	return this->RunOnce([this, init_pos]() {
		m_estimator.ResetPosition(
			m_gyro.GetAngle(),
			get_wheel_pos(),
			init_pos
		);
		m_drive_timer.Restart();
	});
}

/**
 * This command factory returns a command that drives our robot.
 * While we are driving, we want to be constantly updating our position
 * estimator, so we run both drive_subcommand and estimate_position_subcommand
 * in parallel.
 */
frc2::CommandPtr Drive::drive_command(
		std::function<double(void)> drive_power,
		std::function<double(void)> strafe_power,
		std::function<double(void)> rot_power)
{
	return frc2::cmd::Parallel(
		drive_subcommand(drive_power, strafe_power, rot_power),
		estimate_position_subcommand()
	);
}

/**
 * Another example of a command factory. This time we pass in functions
 * to get our directions before we drive.
 *
 * These three lambda expressions are expected to return doubles that we
 * pass into DriveCartesian. We bind these these functions to our command
 * so that we get an updated value on each execution of this command. This
 * is different from the init_lps_command becuase we bind a "constant" value
 * there, that value is then the same for every invocation of the command.
 *
 * The RunEnd command factory is simply defined. While the command runs,
 * the first function is executed every 20ms. When the command ends
 */
frc2::CommandPtr Drive::drive_subcommand(
		std::function<double(void)> drive_power,
		std::function<double(void)> strafe_power,
		std::function<double(void)> rot_power)
{
	return this->RunEnd(
		[this, drive_power, strafe_power, rot_power]() {
			m_mecanums->DriveCartesian(
				drive_power(), strafe_power(), rot_power()
			);
		},
		[this]() { m_mecanums->DriveCartesian(0.0, 0.0, 0.0); }
	);
}

/**
 * Another command factory that returns a command that updates our limelight
 * position estimator.
 */
frc2::CommandPtr Drive::estimate_position_subcommand()
{
	return this->Run(
		[this]() {
			frc::Pose2d pos = get_vision_estimate();
			frc::Rotation2d rot = m_gyro.GetAngle(frc::ADIS16470_IMU::IMUAxis::kYaw);
			m_estimator.UpdateWithTime(
				m_drive_timer.Get(),
				rot,
				get_wheel_pos()
			);
			
			if (pos.X().value() == 0.0
			    && pos.Y().value() == 0.0
			    && pos.Rotation().Radians().value() == 0.0) {
				return;
			}

			m_estimator.AddVisionMeasurement(
				pos, m_drive_timer.Get()
			);
		}
	);
}
