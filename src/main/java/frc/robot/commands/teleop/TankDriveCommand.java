// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.DriveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class TankDriveCommand extends Command {
	private final DriveSubsystem m_driveSubsystem;
	private final Supplier<Double> m_axis1Supplier, m_axis2Supplier;
	private final Supplier<Integer> m_reversedSupplier;

	// Slew rate limiters to make joystick inputs more gentle.
	private final SlewRateLimiter leftSpeedLimiter = new SlewRateLimiter(2);
	private final SlewRateLimiter rightSpeedLimiter = new SlewRateLimiter(2);

	public TankDriveCommand(DriveSubsystem driveSubsystem, 
			Supplier<Double> axis1Supplier, Supplier<Double> axis2Supplier, Supplier<Integer> reversedSupplier) {

		m_driveSubsystem = driveSubsystem;
		addRequirements(driveSubsystem);

		m_axis1Supplier = axis1Supplier;
		m_axis2Supplier = axis2Supplier;
		m_reversedSupplier = reversedSupplier;
	}

	@Override
	public void initialize() {}

	@Override
	public void execute() {
		// Get inputs from suppliers.
		double axis1 = m_axis1Supplier.get();
		double axis2 = m_axis2Supplier.get();
		int reversed = m_reversedSupplier.get();

		// Reverse axis inputs if drivetrain is reversed.
		double leftSpeed, rightSpeed;
        if (reversed == 1) {
            leftSpeed = axis1;
			rightSpeed = axis2;
        } else {
			leftSpeed = axis2;
			rightSpeed = axis1;
        }

		// Apply factors.
		leftSpeed = leftSpeed * reversed;
		rightSpeed = rightSpeed * reversed;

		// Apply deadbands.
        leftSpeed = MathUtil.applyDeadband(leftSpeed, ControllerConstants.DEADBAND);
        rightSpeed = MathUtil.applyDeadband(rightSpeed, ControllerConstants.DEADBAND);

        // Apply slew rate limiters.
        // leftSpeed = leftSpeedLimiter.calculate(leftSpeed);
        // rightSpeed = rightSpeedLimiter.calculate(rightSpeed);

		// System.out.println("Driving at " + leftSpeed + " " + rightSpeed);
		m_driveSubsystem.setTankDrive(leftSpeed, rightSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		m_driveSubsystem.resetMotors();
	}

	// Command always runs during teleop.
	@Override
	public boolean isFinished() {
		return false;
	}
}
