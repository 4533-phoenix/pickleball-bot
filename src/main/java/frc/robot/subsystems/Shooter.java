package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ShooterConstants;

/** Contains the code for the shooter subsystem. */
public final class Shooter extends SubsystemBase {
  /** The shooter subsystem instance. */
  private static Shooter shooter = null;

  /** The flywheel motor. */
  private final SparkMax flywheelMotor =
      new SparkMax(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

  /** Trigger that is active when the shooter is up to speed. */
  public final Trigger isReady =
      new Trigger(
          () ->
              flywheelMotor.getEncoder().getVelocity()
                  >= ShooterConstants.FLYWHEEL_VELOCITY.in(RotationsPerSecond)
                      - RotationsPerSecond.of(100).in(RotationsPerSecond));

  /** Trigger that is active when the shooter is running. */
  public final Trigger isRunning = new Trigger(() -> flywheelMotor.getEncoder().getVelocity() > 0);

  /**
   * Gets the shooter subsystem instance.
   *
   * @return The shooter subsystem instance.
   */
  public static Shooter getInstance() {
    /*
     * Constructs the shooter subsystem instance if it
     * has not already been constructed.
     */
    if (shooter == null) {
      shooter = new Shooter();
    }

    return shooter;
  }

  /** Constructs the shooter subsystem instance. */
  private Shooter() {
    SparkMaxConfig flywheelConfig = new SparkMaxConfig();
    flywheelConfig.idleMode(IdleMode.kCoast);
    flywheelConfig.inverted(true);
    flywheelConfig.openLoopRampRate(0.2);
    flywheelConfig.closedLoopRampRate(0.2);
    flywheelConfig.smartCurrentLimit(30);

    // Configure encoder
    EncoderConfig encoderConfig = new EncoderConfig();
    encoderConfig.positionConversionFactor(1.0);
    encoderConfig.velocityConversionFactor(1.0);
    flywheelConfig.apply(encoderConfig);

    // Configure PID controller
    ClosedLoopConfig pidConfig = new ClosedLoopConfig();
    ShooterConstants.PID.applyToConfig(pidConfig);
    flywheelConfig.apply(pidConfig);

    flywheelMotor.configure(
        flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Set default command to stop the flywheel when no other command is running
    setDefaultCommand(stop());
  }

  /**
   * Creates a command that runs the flywheel forward at the preset velocity.
   *
   * @return A command that runs the flywheel forward.
   */
  public Command runForward() {
    return run(() ->
            flywheelMotor
                .getClosedLoopController()
                .setReference(ShooterConstants.FLYWHEEL_VELOCITY.in(RotationsPerSecond), ControlType.kVelocity))
        .withName("Shooter-RunForward");
  }

  /**
   * Creates a command that stops the flywheel.
   *
   * @return A command that stops the flywheel.
   */
  public Command stop() {
    return run(() -> flywheelMotor.getClosedLoopController().setReference(0, ControlType.kVelocity))
        .withName("Shooter-Stop");
  }

  /**
   * Creates a command that instantly stops the flywheel (one-time execution).
   *
   * @return A command that stops the flywheel.
   */
  public Command instantStop() {
    return runOnce(
            () -> flywheelMotor.getClosedLoopController().setReference(0, ControlType.kVelocity))
        .withName("Shooter-InstantStop");
  }

  /**
   * Creates a command that waits until the shooter is up to speed.
   *
   * @return A command that waits until the shooter is ready.
   */
  public Command waitUntilReady() {
    return Commands.waitUntil(isReady).withName("Shooter-WaitUntilReady");
  }
}
