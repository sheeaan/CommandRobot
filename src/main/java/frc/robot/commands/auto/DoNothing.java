package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class DoNothing {
    public static Command autoSequence(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem) {
        return Autos.resetSubsystems(driveSubsystem, intakeSubsystem, launcherSubsystem);
    }
}
