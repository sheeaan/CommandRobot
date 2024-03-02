// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveStopCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;

	public DriveStopCommand(DriveSubsystem driveSubsystem) {
		m_driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);
	}

	@Override
	public void initialize() {
		m_driveSubsystem.resetMotors();
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {}

	// One time command.
	@Override
	public boolean isFinished() {
		return true;
	}
}
