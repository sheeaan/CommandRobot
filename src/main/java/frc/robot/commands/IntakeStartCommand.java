// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class IntakeStartCommand extends Command {
	private final IntakeSubsystem m_intakeSubsystem;
	private final boolean m_reversed;

	public IntakeStartCommand(IntakeSubsystem intakeSubsystem, boolean reversed) {
		m_intakeSubsystem = intakeSubsystem;
		addRequirements(intakeSubsystem);
		m_reversed = reversed;
	}

	@Override
	public void initialize() {
		if (m_reversed) {
			System.out.println("Intake roller started (in)!");
		} else {
			System.out.println("Intake roller started (out)!");
		}
		m_intakeSubsystem.startRoller(m_reversed);
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
