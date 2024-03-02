// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PIDConstants;

public class IntakeSubsystem extends SubsystemBase {

	// Intake hardware.
	private final PIDController pidPivot;
    private final CANSparkMax leadPivotMotor;    
    private final CANSparkMax rollerMotor;
    private final DutyCycleEncoder pivotEncoder;

    private double pivotSetpoint;
    private IntakePosition intakePosition;
	public static enum IntakePosition {
        DEPLOYED, RETRACTED, AMP, SOURCE
    }

	/** Creates a new drive subsystem. */
	public IntakeSubsystem() {

		// Pivot controls
        leadPivotMotor = new CANSparkMax(HardwareMap.IT_PIVOT_LEAD, MotorType.kBrushless);
        leadPivotMotor.restoreFactoryDefaults();
        leadPivotMotor.setInverted(true);
        setMotorToBrake();

        pidPivot = new PIDController(PIDConstants.Intake.P, PIDConstants.Intake.I, PIDConstants.Intake.D);

		// Roller controls
        rollerMotor = new CANSparkMax(HardwareMap.IT_ROLLER_LEAD, MotorType.kBrushless);
        rollerMotor.restoreFactoryDefaults();

        // the intake better be all the way down (retracted)
        pivotEncoder = new DutyCycleEncoder(0);
        pivotEncoder.setDistancePerRotation(360);
        pivotEncoder.reset();

        setMotorPosition(IntakePosition.RETRACTED);

        SmartDashboard.putNumber("Intake Pivot P", pidPivot.getP());
        SmartDashboard.putNumber("Intake Pivot I", pidPivot.getI());
        SmartDashboard.putNumber("Intake Pivot D", pidPivot.getD());
	}

	@Override
	public void periodic() {
		double encoderPos = pivotEncoder.getDistance();

		// Set motor to go to setpoint
        double pivotSpeed = pidPivot.calculate(encoderPos, pivotSetpoint) * IntakeConstants.MAX_PERCENT_OUTPUT;
        double pivotOutput = pivotSpeed * leadPivotMotor.getBusVoltage();
        pivotOutput = MathUtil.clamp(pivotOutput, -12, 12);
        leadPivotMotor.setVoltage(pivotOutput);
        
        // PID Values: changeable using ShuffleBoard
        double p = SmartDashboard.getNumber("Intake Pivot P", 0);
        double i = SmartDashboard.getNumber("Intake Pivot I", 0);
        double d = SmartDashboard.getNumber("Intake Pivot D", 0);
        if (p != pidPivot.getP()) pidPivot.setP(p);
        if (i != pidPivot.getI()) pidPivot.setI(i);
        if (d != pidPivot.getD()) pidPivot.setD(d);

        // Output all variables to SmartDashboard
        SmartDashboard.putNumber("Roller Output", rollerMotor.getAppliedOutput());
        SmartDashboard.putNumber("Pivot Position", encoderPos);
        SmartDashboard.putNumber("Pivot Setpoint", pivotSetpoint);
        SmartDashboard.putNumber("Pivot Speed", pivotSpeed);
        SmartDashboard.putNumber("Pivot Output", pivotOutput);
	}

	/**
	 * Set position of intake arm.
	 */
	public void setMotorPosition(IntakePosition pos) {
        intakePosition = pos;
        pivotSetpoint = switch (pos) {
            case DEPLOYED -> IntakeConstants.DEPLOYED_POS; // positive (~200)
            case RETRACTED -> IntakeConstants.RETRACTED_POS; // 0
            case AMP -> IntakeConstants.AMP_POS;
            case SOURCE -> IntakeConstants.SOURCE_POS;
        };
    }
    public IntakePosition getMotorPosition() {
        return intakePosition;
    }

    public boolean intakeInPosition() {
        return MathUtil.isNear(pivotSetpoint, pivotEncoder.getDistance(), 5);
    }

	/** Roller Code */
	public void startRoller(boolean reverse) {
        if (reverse) {
            rollerMotor.set(IntakeConstants.Roller.MAX_PERCENT_OUTPUT);
        } else {
            rollerMotor.set(-IntakeConstants.Roller.MAX_PERCENT_OUTPUT);
        }
    }
    public void stopRoller() {
        rollerMotor.set(0);
    }

	/*
     * Set idle mode of arm motor to brake or coast.
     */
    public void setMotorToBrake() {
        leadPivotMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    }
    public void setMotorToCoast() {
        leadPivotMotor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

	/*
     * Set PID values of intake motor pivot.
     */
    public void setPivotP(double p) {
        pidPivot.setP(p);
    }
    public void setPivotI(double i) {
        pidPivot.setI(i);
    }
    public void setPivotD(double d) {
        pidPivot.setD(d);
    }

    /*
     * Get PID values of intake motor pivot.
     */
    public double getPivotP() {
        return pidPivot.getP();
    }
    public double getPivotI() {
        return pidPivot.getI();
    }
    public double getPivotD() {
        return pidPivot.getD();
    }
}
