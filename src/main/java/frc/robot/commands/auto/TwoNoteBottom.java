package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.DriveDistanceCommand;
import frc.robot.commands.IntakeStartCommand;
import frc.robot.commands.IntakeStopCommand;
import frc.robot.commands.LauncherStartCommand;
import frc.robot.commands.LauncherStopCommand;
import frc.robot.commands.SetIntakePositionCommand;
import frc.robot.commands.TimedDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakePosition;

public class TwoNoteBottom {
    public static Command autoSequence(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        return Commands.sequence(

            Commands.parallel(
                // Drive back for better shot - 0 to 2 s
                new TimedDriveCommand(driveSubsystem, 0.3, 2),
                
                // Start launch sequence - 0 to 3 s
                Autos.launchSequence(intakeSubsystem, launcherSubsystem)
            ),

            Commands.parallel(
                // Drive to next note TODO

                // While prepping intake
                Commands.sequence(
                    Commands.waitSeconds(2),
                    new SetIntakePositionCommand(intakeSubsystem, IntakePosition.DEPLOYED),

                    // Intake
                    new IntakeStartCommand(intakeSubsystem, false)
                )
            ),

            Commands.parallel(
                // Retract intake arm
                Commands.sequence(
                    new IntakeStopCommand(intakeSubsystem),
                    new SetIntakePositionCommand(intakeSubsystem, IntakePosition.RETRACTED)
                )
                
                // Drive back to speaker TODO
            ),

            Autos.launchSequence(intakeSubsystem, launcherSubsystem)
        );
    }
}
