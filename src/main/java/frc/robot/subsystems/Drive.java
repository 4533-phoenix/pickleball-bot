package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/** Contains the code for the drive subsystem. */
public final class Drive extends SubsystemBase {
  /** The drive subsystem instance. */
  private static Drive drive = null;

  /** The left motor. */
  private final SparkMax leftMotor = new SparkMax(DriveConstants.LEFT_ID, MotorType.kBrushless);

  /** The right motor. */
  private final SparkMax rightMotor = new SparkMax(DriveConstants.RIGHT_ID, MotorType.kBrushless);

  /** The gyro used for odometry and rotation measurement. */
  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  /** The kinematics object for converting between wheel and chassis speeds. */
  private final DifferentialDriveKinematics kinematics =
      new DifferentialDriveKinematics(DriveConstants.TRACK_WIDTH);

  /** The pose estimator that fuses wheel odometry and gyro data. */
  private final DifferentialDrivePoseEstimator poseEstimator =
      new DifferentialDrivePoseEstimator(
          kinematics,
          gyro.getRotation2d(),
          getLeftDistance(),
          getRightDistance(),
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Degrees.of(5).in(Radians)),
          VecBuilder.fill(0.5, 0.5, Degrees.of(30).in(Radians)));

  /** Field visualization object for the dashboard. */
  private final Field2d field = new Field2d();

  /**
   * Gets the drive subsystem instance.
   *
   * @return The drive subsystem instance.
   */
  public static Drive getInstance() {
    /*
     * Constructs the drive subsystem instance if it
     * has not already been constructed.
     */
    if (drive == null) {
      drive = new Drive();
    }

    return drive;
  }

  /** Constructs the drive subsystem instance. */
  private Drive() {
    // Create a single configuration for both motors
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);
    config.openLoopRampRate(0.5);
    config.closedLoopRampRate(0.5);
    config.smartCurrentLimit(30);

    // Create a single encoder configuration
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters) / DriveConstants.GEAR_REDUCTION);
    encoderConfig.velocityConversionFactor(
        (DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters) / 60.0) / DriveConstants.GEAR_REDUCTION);
    config.apply(encoderConfig);

    // Create a single PID controller for both motors
    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    DriveConstants.PID.applyToConfig(pidConfig);

    // Configure left motor (not inverted)
    config.inverted(false);
    leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Configure right motor (inverted)
    config.inverted(true);
    rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Reset the gyro and encoders
    resetSensors();

    // Put the field visualization on the dashboard
    SmartDashboard.putData("Field", field);
  }

  /**
   * Creates a command to drive the robot using joystick inputs.
   * 
   * @param x The X-axis (turning) input from -1.0 to 1.0
   * @param y The Y-axis (forward/reverse) input from -1.0 to 1.0
   * @param slowDrive True enables slow mode, false enables fast mode
   * @return Command for driving with joystick control
   */
  public Command getDriveCommand(DoubleSupplier x, DoubleSupplier y, BooleanSupplier slowDrive) {
    return run(
        () -> {
          double left = y.getAsDouble() - x.getAsDouble();
          double right = y.getAsDouble() + x.getAsDouble();

          LinearVelocity maxVelocity =
              slowDrive.getAsBoolean()
                  ? DriveConstants.SLOW_VELOCITY
                  : DriveConstants.FAST_VELOCITY;

          double leftVelocity = left * maxVelocity.in(MetersPerSecond);
          double rightVelocity = right * maxVelocity.in(MetersPerSecond);

          leftMotor.getClosedLoopController().setReference(leftVelocity, ControlType.kVelocity);
          rightMotor.getClosedLoopController().setReference(rightVelocity, ControlType.kVelocity);
        });
  }

  /**
   * Gets the current estimated pose of the robot.
   *
   * @return The current estimated pose
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Gets the left encoder distance in meters.
   *
   * @return The left encoder distance
   */
  private double getLeftDistance() {
    return leftMotor.getEncoder().getPosition();
  }

  /**
   * Gets the right encoder distance in meters.
   *
   * @return The right encoder distance
   */
  private double getRightDistance() {
    return rightMotor.getEncoder().getPosition();
  }

  /**
   * Resets the pose estimator to the specified pose.
   *
   * @param pose The pose to reset to
   */
  public void resetPose(Pose2d pose) {
    resetSensors();
    poseEstimator.resetPosition(gyro.getRotation2d(), getLeftDistance(), getRightDistance(), pose);
  }

  /** Resets the gyro and encoders. */
  public void resetSensors() {
    gyro.reset();
    leftMotor.getEncoder().setPosition(0);
    rightMotor.getEncoder().setPosition(0);
  }

  @Override
  public void periodic() {
    // Periodic method for the drive subsystem.
    // Update the pose estimator with the latest encoder and gyro readings
    poseEstimator.update(gyro.getRotation2d(), getLeftDistance(), getRightDistance());

    // Update the field visualization
    field.setRobotPose(getPose());
  }
}
