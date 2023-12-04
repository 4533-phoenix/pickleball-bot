package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * Contains the constants classes.
 */
public final class Constants {
    /**
     * Contains the drive constants.
     */
    public static final class DriveConstants {
        // Motor ids.
        public static final int LEFT_LEADER_ID = 5;
        public static final int RIGHT_LEADER_ID = 2;
        public static final int LEFT_FOLLOWER_ID = 3;
        public static final int RIGHT_FOLLOWER_ID = 4;

        // Feedforward constants.
        public static final double KS = 1.0;
        public static final double KV = 1.0;

        // PID constants.
        public static final double KP = 1.0;
        public static final double KI = 1.0;
        public static final double KD = 1.0;

        // Math constants.
        public static final double MAX_VELOCITY = 5.0; // m/s
        public static final double WHEEL_CIRCUMFERENCE = 0.31919; // m
        public static final double RPM_TO_METERS_PER_SECOND = WHEEL_CIRCUMFERENCE / 60.0;
        public static final double GEAR_REDUCTION = 6.0;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(1.0);
    }

    /**
     * Contains the feeder constants.
     */
    public static final class FeederConstants {
        // Servo ids.
        public static final int PRELIMINARY_SERVO_ID = 0;
        public static final int FIRING_SERVO_ID = 1;
    }

    /**
     * Contains the shooter constants.
     */
    public static final class ShooterConstants {
        // Motor ids.
        public static final int FLYWHEEL_MOTOR_ID = 1;

        // Feedforward constants.
        public static final double KS = -0.00024616;
        public static final double KV = 0.12589 / 60.0; // kv is in rot/s, so divide by 60s to get from rpm to rot/s

        // Math constants.
        public static final double FLYWHEEL_VELOCITY = 3500.0; // rpm
    }

    /**
     * Contains the controller constants.
     */
    public static final class ControllerConstants {
        // Controller ids.
        public static final int CONTROLLER_ID = 0;

        // Button ids.
        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;

        public static final int BUTTON_LB = 5;
        public static final int BUTTON_RB = 6;

        public static final int BUTTON_BACK = 7;
        public static final int BUTTON_START = 8;

        public static final int BUTTON_LEFT_STICK = 9;
        public static final int BUTTON_RIGHT_STICK = 10;
    }
}
