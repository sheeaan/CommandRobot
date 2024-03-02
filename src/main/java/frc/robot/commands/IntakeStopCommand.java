// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeStopCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;

	public IntakeStopCommand(IntakeSubsystem intakeSubsystem) {
		m_intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
	}

	@Override
	public void initialize() {
		System.out.println("Intake roller stopped!");
		m_intakeSubsystem.stopRoller();
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
