package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.helpers.PID;

/** Contains the constants classes. */
public final class Constants {
  /** Contains the drive constants. */
  public static final class DriveConstants {
    // Motor ids.
    public static final int LEFT_ID = 2;
    public static final int RIGHT_ID = 3;

    // PID constants.
    public static final PID PID = new PID(0.1, 0.0, 0.0);

    // Math constants.
    public static final LinearVelocity FAST_VELOCITY = MetersPerSecond.of(1.0); // m/s
    public static final LinearVelocity SLOW_VELOCITY = MetersPerSecond.of(0.5); // m/s
    public static final Distance WHEEL_CIRCUMFERENCE = Meters.of(0.31919); // m
    public static final double GEAR_REDUCTION = 6.0;
    
    // Odometry and kinematics constants
    public static final Distance TRACK_WIDTH = Meters.of(0.57); // meters - distance between left and right wheels
  }

  /** Contains the shooter constants. */
  public static final class ShooterConstants {
    // Motor ids.
    public static final int FLYWHEEL_MOTOR_ID = 1;

    // PID constants
    public static final PID PID = new PID(0.0005, 0.0, 0.0);

    // Math constants.
    public static final AngularVelocity FLYWHEEL_VELOCITY = RotationsPerSecond.of(2500.0); // rpm
  }

  /** Contains the feeder constants. */
  public static final class FeederConstants {
    // Servo ids.
    public static final int PRELIMINARY_SERVO_ID = 0;
    public static final int FIRING_SERVO_ID = 1;

    // Angle (in degrees) for the closed and open positions of the servos
    public static final Angle CLOSED_ANGLE = Degrees.of(180.0);
    public static final Angle SEMI_CLOSED_ANGLE = Degrees.of(145.0);
    public static final Angle OPEN_ANGLE = Degrees.of(110.0);

    // Delay (in s) between open/close operations
    public static final Time DROP_DELAY = Seconds.of(0.250);
  }

  /** Contains the limelight constants. */
  public static final class LimelightConstants {
    public static final String LIMELIGHT_NAME = "limelight";
  }

  /** Contains the controller constants. */
  public static final class ControllerConstants {
    // Controller ids.
    public static final int CONTROLLER_ID = 0;
  }
}
