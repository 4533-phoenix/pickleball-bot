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
        public static final int LEFT_LEADER_ID = 0;
        public static final int RIGHT_LEADER_ID = 1;
        public static final int LEFT_FOLLOWER_ID = 2;
        public static final int RIGHT_FOLLOWER_ID = 3;

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
     * Contains the controller constants.
     */
    public static final class ControllerConstants {
        // Controller ids.
        public static final int CONTROLLER_ID = 0;
    }
}
