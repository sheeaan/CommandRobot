// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

public class DriveSubsystem extends SubsystemBase {
	
	// Motors, PID controllers, and encoders.
	private CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;

	// Differential drive.
	private DifferentialDrive drivetrain;

	// Other hardware.
	private RelativeEncoder leftEncoder, rightEncoder;
	private final AHRS gyro = new AHRS(SPI.Port.kMXP);

	// Odometry and pose estimator for keeping track of position on field
	private final Pose2d startPosition = new Pose2d(); // TODO update
	private final DifferentialDriveOdometry m_odometry;

	/** Creates a new drive subsystem. */
	public DriveSubsystem() {
		
		// Initialize brushless motors with IDs from 1 to 4.
        leftLeader = new CANSparkMax(Constants.HardwareMap.DT_LEFT_LEADER_ID, MotorType.kBrushless);
        leftFollower = new CANSparkMax(Constants.HardwareMap.DT_LEFT_FOLLOWER_ID, MotorType.kBrushless);
        rightLeader = new CANSparkMax(Constants.HardwareMap.DT_RIGHT_LEADER_ID, MotorType.kBrushless);
        rightFollower = new CANSparkMax(Constants.HardwareMap.DT_RIGHT_FOLLOWER_ID, MotorType.kBrushless);

        // Set motors to default states.
        leftLeader.restoreFactoryDefaults();
        leftFollower.restoreFactoryDefaults();
        rightLeader.restoreFactoryDefaults();
        rightFollower.restoreFactoryDefaults();

        // Set leaders and followers.
        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);

        // Invert one side of the drivetrain.
        leftLeader.setInverted(false);
        rightLeader.setInverted(true);

		// Get motor encoders and gyro, and initialise them.
        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        leftEncoder.setPositionConversionFactor(2 * DrivetrainConstants.WHEEL_RADIUS_M * Math.PI / 8.5);
		// gyro.reset();

        gyro.enableBoardlevelYawReset(true);
        // gyro.zeroYaw()

		// Create differential drivetrain.
		drivetrain = new DifferentialDrive(leftLeader, rightLeader);

		// TODO rotation 2D not correct
		m_odometry = new DifferentialDriveOdometry(
                new Rotation2d(gyro.getYaw()), leftEncoder.getPosition(), rightEncoder.getPosition(), 
                startPosition);     // Initial pose of robot: default to x = 0, y = 0, theta = 0
	}

	/** 
     * Controls the robot drivetrain using arcade drive. (Used only for auto)
     * 
     * @param speed - The robot's speed along the X axis (-1.0 to 1.0). Forward is positive.
     * @param rot - The robot's rotation rate around the Z axis (-1.0 to 1.0). Counterclockwise is positive.
     */
	public void setArcadeDrive(double speed, double rot) {
        drivetrain.arcadeDrive(speed * DrivetrainConstants.MAX_LINEAR_SPEED,
                               rot * DrivetrainConstants.MAX_ANGULAR_SPEED);
    }

	/** 
     * Controls the robot drivetrain using tank drive.
     * 
     * @param leftSpeed - The robot's left side speed (-1.0 to 1.0).
     * @param rightSpeed - The robot's right side speed (-1.0 to 1.0).
     */
	public void setTankDrive(double leftSpeed, double rightSpeed) {
        drivetrain.tankDrive(leftSpeed * DrivetrainConstants.MAX_LINEAR_SPEED, 
                             rightSpeed * DrivetrainConstants.MAX_LINEAR_SPEED);
    }

	/**
     * Resets all motor voltages to zero.
     */
    public void resetMotors() {
		leftLeader.setVoltage(0);
		rightLeader.setVoltage(0);
    }

    /**
     * Get encoder positions. 
     */
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }
    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }
    public double getGyroAngle() {
        return gyro.getAngle();
    }

	/**
     * Get pose of robot from pose estimator.
     * @return the estimated field-relative robot pose, in meters from bottom left.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the translation of the robot.
     * @return The translation of the robot, in meters from bottom left.
     */
    public Translation2d getTranslation() {
        return m_odometry.getPoseMeters().getTranslation();
    }

    /**
     * Returns the rotation of the robot.
     * @return The rotation of the robot, in degrees.
     */
    public Rotation2d getRotation() {
        return m_odometry.getPoseMeters().getRotation();
    }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
        System.out.println(//getLeftEncoderPosition() + " " + getRightEncoderPosition() +
                        gyro.getQuaternionW() + 
                           " (" + gyro.getPitch() + " " + gyro.getYaw() + " " + gyro.getRoll() + " " +  gyro.getAngle() + ")");
	}
}
