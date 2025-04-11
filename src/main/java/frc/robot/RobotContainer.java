package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.*;

/**
 * Contains the code for registering subsystems with the {@link CommandScheduler}, the robot
 * controller, the hash map for the autonomous commands, and the hash map for the initial positions
 * of the autonomous commands.
 */
public final class RobotContainer {
  /** The controller used to control the robot. */
  private static final CommandXboxController controller =
      new CommandXboxController(ControllerConstants.CONTROLLER_ID);

  private static final Drive drive = Drive.getInstance();
  private static final Shooter shooter = Shooter.getInstance();
  private static final Feeder feeder = Feeder.getInstance();
  private static final Vision vision = Vision.getInstance();

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);

    registerButtons();
  }

  /** Registers the button commands with their respective buttons. */
  public static void registerButtons() {
    // Registers the drive command to the left stick.
    drive.setDefaultCommand(
        drive.getDriveCommand(
            () -> controller.getLeftY(),
            () -> controller.getRightX(),
            () -> controller.a().getAsBoolean()));

    /*
     * Registers the run flywheel forward command to the right trigger.
     * When the trigger is released, the stop flywheel command is run.
     */
    controller.rightTrigger().whileTrue(shooter.runForward());

    // Registers the normal fire command to the left bumper.
    controller.leftBumper().onTrue(feeder.singleFeedCycle());

    // Registers the rapid fire command to the right bumper.
    controller.rightBumper().whileTrue(feeder.rapidFire());

    // Registers the Limelight blink command to the B button
    controller.b().whileTrue(vision.blinkWhileActive());
  }
}
