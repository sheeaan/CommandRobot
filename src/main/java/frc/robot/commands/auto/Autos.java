// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import frc.robot.commands.DriveStopCommand;
import frc.robot.commands.IntakeStartCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.LauncherStartCommand;
import frc.robot.commands.LauncherStopCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class Autos {

	public static Command resetSubsystems(DriveSubsystem driveSubsystem, 
			IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
		return Commands.parallel(
			new DriveStopCommand(driveSubsystem),
			new IntakeStopCommand(intakeSubsystem),
			new LauncherStopCommand(launcherSubsystem)
		);
	}

	public static Command launchSequence(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
		return Commands.sequence(
			// Set position of intake arm.
			new SetIntakePositionCommand(intakeSubsystem, IntakePosition.RETRACTED),

			// Start and wait for launcher to rev up for 2.5 s
			new LauncherStartCommand(launcherSubsystem),
			Commands.waitSeconds(2.5),

			// Intake out for 0.5 s
			new IntakeStartCommand(intakeSubsystem, true),
			Commands.waitSeconds(0.5),

			// Stop after sequence finished
			Commands.parallel(
				new LauncherStopCommand(launcherSubsystem),
				new IntakeStopCommand(intakeSubsystem)
			)
		);
	}

	private Autos() {
		throw new UnsupportedOperationException("This is a utility class!");
	}
}
