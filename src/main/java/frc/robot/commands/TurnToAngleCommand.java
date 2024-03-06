// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TurnToAngleCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private final double m_target;
	private final PIDController m_pidController = new PIDController(0.1, 0, 0);

	public TurnToAngleCommand(DriveSubsystem driveSubsystem, double angle) {
		m_driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);
		m_target = angle;

		m_pidController.setTolerance(0.2);
		m_pidController.enableContinuousInput(-180, 180);
	}

	@Override
	public void initialize() {
		System.out.println(String.format("Turn to angle %.0f", m_target));
		m_pidController.reset();
	}

	@Override
	public void execute() {
		System.out.println("Angle " + m_driveSubsystem.getGyroAngle());
		double rot = m_pidController.calculate(m_driveSubsystem.getGyroAngle(), m_target);
		m_driveSubsystem.setArcadeDrive(0, rot);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.resetMotors();
		System.out.println("Finished PID");
	}

	// One time command.
	@Override
	public boolean isFinished() {
		return m_pidController.atSetpoint();
	}
}
