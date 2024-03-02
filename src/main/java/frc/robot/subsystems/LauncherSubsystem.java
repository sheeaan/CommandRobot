// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LauncherConstants;
import frc.robot.Constants.PIDConstants;

public class LauncherSubsystem extends SubsystemBase {

	// Launcher hardware.
	private final CANSparkMax topRoller;
    private final CANSparkMax bottomRoller;
    private final SparkPIDController topLauncherPid, bottomLauncherPid;
    private final RelativeEncoder launcherEncoder;

    // TODO: Launcher PID

	/** Creates a new drive subsystem. */
	public LauncherSubsystem() {

		// Motor controllers
        topRoller = new CANSparkMax(Constants.HardwareMap.LTC_TOP, CANSparkLowLevel.MotorType.kBrushless);
        bottomRoller = new CANSparkMax(Constants.HardwareMap.LTC_BOTTOM, CANSparkLowLevel.MotorType.kBrushless);

        topRoller.restoreFactoryDefaults();
        bottomRoller.restoreFactoryDefaults();
 
        topRoller.setIdleMode(CANSparkBase.IdleMode.kCoast);
        bottomRoller.setIdleMode(CANSparkBase.IdleMode.kCoast);

        bottomRoller.setInverted(true);

        // PID controller for launcher
        topLauncherPid = topRoller.getPIDController();
        topLauncherPid.setP(PIDConstants.Launcher.P);
        topLauncherPid.setI(PIDConstants.Launcher.I);
        topLauncherPid.setD(PIDConstants.Launcher.D);

        bottomLauncherPid = bottomRoller.getPIDController();
        bottomLauncherPid.setP(PIDConstants.Launcher.P);
        bottomLauncherPid.setI(PIDConstants.Launcher.I);
        bottomLauncherPid.setD(PIDConstants.Launcher.D);

        SmartDashboard.putNumber("Launcher P", topLauncherPid.getP());
        SmartDashboard.putNumber("Launcher I", topLauncherPid.getI());
        SmartDashboard.putNumber("Launcher D", topLauncherPid.getD());

        // Encoder
        launcherEncoder = topRoller.getEncoder();
	}

	@Override
	public void periodic() {        
        // PID Values are changeable using ShuffleBoard
        double p = SmartDashboard.getNumber("Launcher P", 0);
        double i = SmartDashboard.getNumber("Launcher I", 0);
        double d = SmartDashboard.getNumber("Launcher D", 0);
        if (p != topLauncherPid.getP()) topLauncherPid.setP(p);
        if (i != topLauncherPid.getI()) topLauncherPid.setI(i);
        if (d != topLauncherPid.getD()) topLauncherPid.setD(d);

        // Motor controller output
        SmartDashboard.putNumber("Launcher Output", topRoller.getAppliedOutput());
        SmartDashboard.putNumber("Launcher Pos", launcherEncoder.getPosition());
        SmartDashboard.putNumber("Launcher Vel", launcherEncoder.getVelocity());
	}

    public void startLauncher() {
        topLauncherPid.setReference(LauncherConstants.MAX_PERCENT_OUTPUT * topRoller.getBusVoltage(), 
            ControlType.kVoltage);
        bottomLauncherPid.setReference(LauncherConstants.MAX_PERCENT_OUTPUT * topRoller.getBusVoltage(), 
            ControlType.kVoltage);
    }
    public void stopLauncher() {
        topRoller.set(0);
        bottomRoller.set(0);
    }
}