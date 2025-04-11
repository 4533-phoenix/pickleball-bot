package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.helpers.LimelightHelper;

/** Subsystem that controls the Limelight vision camera. */
public class Vision extends SubsystemBase {
  /** The vision subsystem instance. */
  private static Vision instance = null;

  /**
   * Gets the vision subsystem instance.
   *
   * @return The vision subsystem instance.
   */
  public static Vision getInstance() {
    if (instance == null) {
      instance = new Vision();
    }
    return instance;
  }

  /** Constructs the vision subsystem instance. */
  private Vision() {
    // Initialize Limelight with LEDs off
    LimelightHelper.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_NAME);
  }

  /**
   * Creates a command that sets the Limelight LEDs to blink. LEDs will be turned off when the
   * command ends.
   *
   * @return A command that sets the Limelight LEDs to blink.
   */
  public Command blinkLEDs() {
    return runOnce(() -> LimelightHelper.setLEDMode_ForceBlink(LimelightConstants.LIMELIGHT_NAME))
        .withName("Vision-BlinkLEDs");
  }

  /**
   * Creates a command that turns off the Limelight LEDs.
   *
   * @return A command that turns off the Limelight LEDs.
   */
  public Command turnOffLEDs() {
    return runOnce(() -> LimelightHelper.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_NAME))
        .withName("Vision-TurnOffLEDs");
  }

  /**
   * Creates a command that turns on the Limelight LEDs. LEDs will be turned off when the command
   * ends.
   *
   * @return A command that turns on the Limelight LEDs.
   */
  public Command turnOnLEDs() {
    return runOnce(() -> LimelightHelper.setLEDMode_ForceOn(LimelightConstants.LIMELIGHT_NAME))
        .withName("Vision-TurnOnLEDs");
  }

  /**
   * Creates a command that makes the Limelight LEDs blink while the command is active, and turns
   * them off when the command ends.
   *
   * @return A command that blinks the Limelight LEDs while active.
   */
  public Command blinkWhileActive() {
    return startEnd(
            () -> LimelightHelper.setLEDMode_ForceBlink(LimelightConstants.LIMELIGHT_NAME),
            () -> LimelightHelper.setLEDMode_ForceOff(LimelightConstants.LIMELIGHT_NAME))
        .withName("Vision-BlinkWhileActive");
  }
}
