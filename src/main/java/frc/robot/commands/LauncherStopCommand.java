// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class LauncherStopCommand extends Command {
	private final LauncherSubsystem m_LauncherSubsystem;

	public LauncherStopCommand(LauncherSubsystem launcherSubsystem) {
		m_LauncherSubsystem = launcherSubsystem;
		addRequirements(launcherSubsystem);
	}

	@Override
	public void initialize() {
		System.out.println("Launcher stopped!");
		m_LauncherSubsystem.stopLauncher();
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
