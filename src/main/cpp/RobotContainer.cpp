// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() {
	// Initialize all of your commands and subsystems here

	// Configure the button bindings
	ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
	// Essentially this line tells the drive to drive with values from the
	// joysticks if you aren't doing anything else.
	m_drive.SetDefaultCommand(m_drive.drive_command(
		[this]() { return m_driverController.GetLeftX(); },
		[this]() { return -m_driverController.GetLeftY(); },
		[this]() { return m_driverController.GetRightY(); }
	));

	// Schedule `ExampleCommand` when `exampleCondition` changes to `true`
	frc2::Trigger([this] {
			return m_subsystem.ExampleCondition();
			}).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

	// Attempt to pick up one note when the X button is pressed.
	m_driverController.X().WhileTrue(m_shooter.pickup_command().ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
	// An example command will be run in autonomous
	return autos::ExampleAuto(&m_subsystem);
}
