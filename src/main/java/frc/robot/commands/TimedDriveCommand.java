// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TimedDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private final double m_speed, m_time;
	private double startTime;

	public TimedDriveCommand(DriveSubsystem driveSubsystem, double speed, double time) {
		m_driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);

		m_speed = speed;
		m_time = time;
	}

	@Override
	public void initialize() {
		startTime = Timer.getFPGATimestamp();
		System.out.println("Start timed drive.");
	}

	@Override
	public void execute() {
		m_driveSubsystem.setArcadeDrive(m_speed, 0);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.resetMotors();
		System.out.println("End timed drive.");
	}

	// One time command.
	@Override
	public boolean isFinished() {
		return (Timer.getFPGATimestamp() - startTime >= m_time);
	}
}
