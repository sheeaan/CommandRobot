// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {

    /**
     * CAN and RIO IDs for hardware throughout the robot.
     */
    public static class HardwareMap {
        // CAN IDs for Drivetrain motors. Leader are FRONT motors, followers are BACK motors.
        public static final int DT_LEFT_LEADER_ID = 1;
        public static final int DT_LEFT_FOLLOWER_ID = 2;
        public static final int DT_RIGHT_LEADER_ID = 3;
        public static final int DT_RIGHT_FOLLOWER_ID = 4;

        public static final int IT_PIVOT_LEAD = 5;
        public static final int IT_ROLLER_LEAD = 6;

        public static final int LTC_BOTTOM = 10;
        public static final int LTC_TOP = 11;
    }

    /**
     * Driver options for mode and speed modifiers.
     */
    public static class DrivetrainConstants {
        public static final double MAX_LINEAR_SPEED = 0.9;
        public static final double MAX_ANGULAR_SPEED = 0.9;

        public static final double TRACK_WIDTH_M = 0.554;
        public static final DifferentialDriveKinematics KINEMATICS = 
            new DifferentialDriveKinematics(TRACK_WIDTH_M);
        public static final double WHEEL_RADIUS_M = 7.62 / 100;
    }

    /** 
     * Names, buttons, and axes for different controllers. 
     */
    public static class ControllerConstants {
        public static final double DEADBAND = 0.1;

        public class ControllerName {
            public static final String IFLIGHT = "iFlight Commando 8 Joystick";
            public static final String XBOX = "Xbox Controller";
        }
    
        public static class iFlight {
            public static final int LEFT_Y_AXIS = 2;
            public static final int RIGHT_X_AXIS = 0;
            public static final int RIGHT_Y_AXIS = 1;
            public static final int REVERSE_AXIS = 4;
        }
    
        public static class Xbox {
            public static final int AMP_BUTTON = 1; // A
            public static final int DEPLOYED_BUTTON = 2; // B
            public static final int RETRACTED_BUTTON = 3; // X
            public static final int SOURCE_BUTTON = 4; // Y
            
            public static final int ROLLER_IN_BUTTON = 5; // LB
            public static final int ROLLER_OUT_BUTTON = 6; // RB

            public static final int LAUNCHER_AXIS = 2; // LT
        }
    }
    
    /**
     * PID controller constants for drivetrain.
     */
    public static class PIDConstants {

        // For motor controllers in manual drive.
        public static class Drive {
            public static final double kP = 6e-5; 
            public static final double kI = 0;
            public static final double kD = 0; 
            public static final double kIz = 0; 
            public static final double kFF = 0.000015;  // feed forward
        }

        public static class Intake {
            public static final double P = 0.01;
            public static final double I = 0;
            public static final double D = 0;
        }

        public static class Launcher {
            public static final double P = 0.2;
            public static final double I = 0;
            public static final double D = 0;
        }

        // For vision alignment with AprilTags.
        public static class Vision {
            public static final double LINEAR_P = 0.5;
            public static final double LINEAR_D = 0.1;
            
            public static final double ANGULAR_P = 0.03;
            public static final double ANGULAR_D = 0.003;
        }
    }

    public static class FF {
        public static class Intake {
            public static final double kS = 0;
            public static final double kG = 1.29;
            public static final double kV = 0.97;
            public static final double kA = 0.07;
        }
    }

    public static class IntakeConstants {
        public static final double MAX_PERCENT_OUTPUT = 1;
        public static final double DEPLOYED_POS = 215;
        public static final double AMP_POS = 100;
        public static final double RETRACTED_POS = 0;
        public static final double SOURCE_POS = 75;

        public static class Roller {
            public static final double MAX_PERCENT_OUTPUT = 0.3;
        }
    }

    public static class LauncherConstants {
        public static final double MAX_PERCENT_OUTPUT = 0.5;
    }

	public static class AutoConstants {
		public static final double INTAKE_ROLLER_DURATION = 1.0;
		public static final double LAUNCHER_DURATION = 3.0;
	}
}