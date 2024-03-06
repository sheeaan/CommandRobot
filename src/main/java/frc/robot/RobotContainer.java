// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants.*;
import frc.robot.commands.IntakeStartCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.LauncherStartCommand;
import frc.robot.commands.LauncherStopCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.TurnToAngleCommand;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.teleop.TankDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// Defining robot subsystems.
	private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
	private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
	private final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem();

	// Defining controllers for raw inputs. TODO add to constants
	private Joystick driveController = new Joystick(0);
	private XboxController mechController = new XboxController(1);

	// Defining buttons on mech controller.
	private JoystickButton intakeAmpButton = new JoystickButton(mechController, Xbox.AMP_BUTTON);
	private JoystickButton intakeDeployedButton = new JoystickButton(mechController, Xbox.DEPLOYED_BUTTON);
	private JoystickButton intakeRetractedButton = new JoystickButton(mechController, Xbox.RETRACTED_BUTTON);
	private JoystickButton intakeSourceButton = new JoystickButton(mechController, Xbox.SOURCE_BUTTON);

	private JoystickButton intakeInButton = new JoystickButton(mechController, Xbox.ROLLER_IN_BUTTON);
	private JoystickButton intakeOutButton = new JoystickButton(mechController, Xbox.ROLLER_OUT_BUTTON);

	private Trigger launcherTrigger = new Trigger(() -> mechController.getRawAxis(Xbox.LAUNCHER_AXIS) == 1);

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		m_driveSubsystem.setDefaultCommand(
			new TankDriveCommand(m_driveSubsystem, 
				() -> driveController.getRawAxis(iFlight.LEFT_Y_AXIS), 
				() -> driveController.getRawAxis(iFlight.RIGHT_Y_AXIS), 
				() -> (int)(driveController.getRawAxis(iFlight.REVERSE_AXIS))
		));

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings.
	 */
	private void configureBindings() {		
		// Set intake position with mech controller buttons.
		intakeAmpButton.onTrue(new SetIntakePositionCommand(m_intakeSubsystem, IntakePosition.AMP));
		intakeDeployedButton.onTrue(new SetIntakePositionCommand(m_intakeSubsystem, IntakePosition.DEPLOYED));
		intakeRetractedButton.onTrue(new SetIntakePositionCommand(m_intakeSubsystem, IntakePosition.RETRACTED));
		// intakeSourceButton.onTrue(new SetIntakePositionCommand(m_intakeSubsystem, IntakePosition.SOURCE));

		intakeSourceButton.onTrue(new TurnToAngleCommand(m_driveSubsystem, 180));

		// Set intake.
		intakeInButton
			.onTrue(new IntakeStartCommand(m_intakeSubsystem, true))
			.onFalse(new IntakeStopCommand(m_intakeSubsystem));
		intakeOutButton
			.onTrue(new IntakeStartCommand(m_intakeSubsystem, false))
			.onFalse(new IntakeStopCommand(m_intakeSubsystem));
		
		// Set launcher.
		launcherTrigger
			.onTrue(new LauncherStartCommand(m_launcherSubsystem))
			.onFalse(new LauncherStopCommand(m_launcherSubsystem));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An example command will be run in autonomous
		return Autos.launchSequence(m_intakeSubsystem, m_launcherSubsystem);
	}
}
