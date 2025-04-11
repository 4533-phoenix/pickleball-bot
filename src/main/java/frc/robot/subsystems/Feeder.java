package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/** Contains the code for the feeder subsystem. */
public final class Feeder extends SubsystemBase {
  /** The feeder subsystem instance. */
  private static Feeder feeder = null;

  /** Servo for preliminary feeding. */
  private final Servo preliminaryServo = new Servo(FeederConstants.PRELIMINARY_SERVO_ID);

  /** Servo for firing feeding. */
  private final Servo firingServo = new Servo(FeederConstants.FIRING_SERVO_ID);

  /**
   * Gets the feeder subsystem instance.
   *
   * @return The feeder subsystem instance.
   */
  public static Feeder getInstance() {
    /*
     * Constructs the feeder subsystem instance if it
     * has not already been constructed.
     */
    if (feeder == null) {
      feeder = new Feeder();
    }

    return feeder;
  }

  /** Constructs the feeder subsystem instance. */
  private Feeder() {
    setDefaultCommand(closeAll());
  }

  /**
   * Creates a command that closes both servos.
   *
   * @return A command that closes both servos.
   */
  public Command closeAll() {
    return run(() -> {
          setPreliminary(false);
          setFiring(false);
        })
        .withName("Feeder-CloseAll");
  }

  /**
   * Creates a command that executes the rapid fire sequence.
   *
   * @return A command that executes the rapid fire sequence.
   */
  public Command rapidFire() {
    return run(() -> {
          setPreliminary(true);
          setFiring(true);
        })
        .withName("Feeder-RapidFire");
  }

  /**
   * Creates a command that opens only the preliminary servo.
   *
   * @return A command that opens only the preliminary servo.
   */
  public Command openPreliminaryOnly() {
    return run(() -> {
          setPreliminary(true);
          setFiring(false);
        })
        .withName("Feeder-OpenPreliminaryOnly");
  }

  /**
   * Creates a command that opens only the firing servo.
   *
   * @return A command that opens only the firing servo.
   */
  public Command openFiringOnly() {
    return run(() -> {
          setPreliminary(false);
          setFiring(true);
        })
        .withName("Feeder-OpenFiringOnly");
  }

  /**
   * Creates a command that executes a single feed cycle. Opens preliminary servo, waits for ball to
   * drop, closes preliminary, opens firing, waits for ball to drop, then closes firing.
   *
   * @return A command that executes a single feed cycle.
   */
  public Command singleFeedCycle() {
    return Commands.sequence(
            runOnce(() -> setPreliminary(true)),
            Commands.waitSeconds(FeederConstants.DROP_DELAY.in(Seconds)),
            runOnce(() -> setPreliminary(false)),
            Commands.waitSeconds(0.1),
            runOnce(() -> setFiring(true)),
            Commands.waitSeconds(FeederConstants.DROP_DELAY.in(Seconds)),
            runOnce(() -> setFiring(false)))
        .withName("Feeder-SingleFeedCycle");
  }

  /**
   * Open/close the preliminary servo.
   *
   * @param open True to open the servo, false to close it
   */
  private void setPreliminary(boolean open) {
    if (open) {
      preliminaryServo.setAngle(FeederConstants.OPEN_ANGLE.in(Degrees));
    } else {
      preliminaryServo.setAngle(FeederConstants.SEMI_CLOSED_ANGLE.in(Degrees));
    }
  }

  /**
   * Open/close the firing servo.
   *
   * @param open True to open the servo, false to close it
   */
  private void setFiring(boolean open) {
    if (open) {
      firingServo.setAngle(FeederConstants.OPEN_ANGLE.in(Degrees));
    } else {
      firingServo.setAngle(FeederConstants.SEMI_CLOSED_ANGLE.in(Degrees));
    }
  }
}
