// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DrivetrainConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivePathCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
    private final Pose2d m_endPoint;
    private Timer m_timer = new Timer();
    private RamseteController m_ramsete = new RamseteController();
    
    private Trajectory m_trajectory;
    TrajectoryConfig config = new TrajectoryConfig(4, 4)
        .setKinematics(DrivetrainConstants.KINEMATICS);

	public DrivePathCommand(DriveSubsystem driveSubsystem, Pose2d endpoint) {
		m_driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);

        m_endPoint = endpoint;
	}

	@Override
	public void initialize() {
        Pose2d startPoint = m_driveSubsystem.getPose();
        var interiorWaypoints = new ArrayList<Translation2d>(); // none

        m_trajectory = TrajectoryGenerator.generateTrajectory(startPoint,
                interiorWaypoints, m_endPoint, config);
        
        m_timer.restart();
	}

	@Override
	public void execute() {
        double elapsed = m_timer.get();
        Trajectory.State reference = m_trajectory.sample(elapsed);

        // Get desired speeds
        ChassisSpeeds speeds = m_ramsete.calculate(m_driveSubsystem.getPose(), reference);
        m_driveSubsystem.setArcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

	@Override
	public void end(boolean interrupted) {
        m_driveSubsystem.resetMotors();
    }

	// One time command.
	@Override
	public boolean isFinished() {
		return m_ramsete.atReference();
	}
}
