// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArcadeDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private final Supplier<Double> m_speedSupplier, m_turnSupplier;

	public ArcadeDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedSupplier, Supplier<Double> turnSupplier) {
		m_driveSubsystem = driveSubsystem;
		m_speedSupplier = speedSupplier;
		m_turnSupplier = turnSupplier;
		addRequirements(driveSubsystem);
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		double speed = m_speedSupplier.get();
		double turn = m_turnSupplier.get();

		m_driveSubsystem.setArcadeDrive(speed, turn);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.resetMotors();
	}

	// Command always runs during teleop.
	@Override
	public boolean isFinished() {
		return false;
	}
}
