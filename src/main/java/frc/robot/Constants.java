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
    /** The CAN ID for the left drive motor */
    public static final int LEFT_ID = 2;

    /** The CAN ID for the right drive motor */
    public static final int RIGHT_ID = 3;

    /** PID control constants for the drive motors */
    public static final PID PID = new PID(0.1, 0.0, 0.0);

    /** Maximum drive velocity in fast mode */
    public static final LinearVelocity FAST_VELOCITY = MetersPerSecond.of(1.0);

    /** Maximum drive velocity in slow mode */
    public static final LinearVelocity SLOW_VELOCITY = MetersPerSecond.of(0.5);

    /** Drive wheel circumference */
    public static final Distance WHEEL_CIRCUMFERENCE = Meters.of(0.31919);

    /** Drive gearing reduction ratio */
    public static final double GEAR_REDUCTION = 6.0;

    /** Distance between the left and right wheels */
    public static final Distance TRACK_WIDTH = Meters.of(0.57);
  }

  /** Contains the shooter constants. */
  public static final class ShooterConstants {
    /** The CAN ID for the flywheel motor */
    public static final int FLYWHEEL_MOTOR_ID = 1;

    /** PID control constants for the flywheel */
    public static final PID PID = new PID(0.0005, 0.0, 0.0);

    /** Target velocity for the flywheel */
    public static final AngularVelocity FLYWHEEL_VELOCITY = RotationsPerSecond.of(2500.0);
  }

  /** Contains the feeder constants. */
  public static final class FeederConstants {
    /** The PWM port for the preliminary servo */
    public static final int PRELIMINARY_SERVO_ID = 0;

    /** The PWM port for the firing servo */
    public static final int FIRING_SERVO_ID = 1;

    /** Angle for fully closed position of servos */
    public static final Angle CLOSED_ANGLE = Degrees.of(180.0);

    /** Angle for semi-closed position of servos */
    public static final Angle SEMI_CLOSED_ANGLE = Degrees.of(145.0);

    /** Angle for fully open position of servos */
    public static final Angle OPEN_ANGLE = Degrees.of(110.0);

    /** Delay between servo operations */
    public static final Time DROP_DELAY = Seconds.of(0.250);
  }

  /** Contains the limelight constants. */
  public static final class LimelightConstants {
    /** The network tables name of the Limelight camera */
    public static final String LIMELIGHT_NAME = "limelight";
  }

  /** Contains the controller constants. */
  public static final class ControllerConstants {
    /** The driver station port number for the controller */
    public static final int CONTROLLER_ID = 0;
  }
}
