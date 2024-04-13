// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShotFactory.h"

ShotFactory::ShotFactory(Drive *m_drive, Shooter *m_shooter)
{

}

frc2::CommandPtr ShotFactory::teleop_shoot_command()
{
	
}

frc2::CommandPtr ShotFactory::auto_shoot_command(frc::Pose2d shot_pos)
{
	return frc2::cmd::Sequence(
		frc2::cmd::Parallel(
			m_drive->
}
