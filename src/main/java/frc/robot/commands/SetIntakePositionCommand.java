// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SetIntakePositionCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;
	private final IntakePosition m_intakePosition;

	public SetIntakePositionCommand(IntakeSubsystem intakeSubsystem, IntakePosition intakePosition) {
		m_intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);

		m_intakePosition = intakePosition;
	}

	@Override
	public void initialize() {
		m_intakeSubsystem.setMotorPosition(m_intakePosition);
		System.out.println("Setting intake position.");
	}

	@Override
	public void execute() {}

	@Override
	public void end(boolean interrupted) {
		if (!interrupted) {
			System.out.println("Intake in position!");
		}
	}

	// Command finishes when intake is in correct position.
	@Override
	public boolean isFinished() {
		return m_intakeSubsystem.intakeInPosition();
	}
}
