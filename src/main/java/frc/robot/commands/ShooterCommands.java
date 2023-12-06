package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter;

/**
 * Contains the methods that return the shooter commands.
 */
public final class ShooterCommands {
    /**
     * Gets the run flywheel forward command for the
     * shooter subsytem.
     * 
     * @return The run flywheel forward command.
     */
    public static RunCommand runFlywheelForwardCommand() {
        /*
         * Returns a run command that runs the run flywheel
         * forward method.
         */
        return new RunCommand(
            () -> Shooter.getInstance().runFlywheelForward(), 
            Shooter.getInstance()
        );
    }

    /**
     * Gets the run flywheel backward command for the
     * shooter subsystem.
     * 
     * @return The run flywheel backward command.
     */
    public static RunCommand runFlywheelBackwardCommand() {
        /*
         * Returns a run command that runs the run flywheel
         * backward method.
         */
        return new RunCommand(
            () -> Shooter.getInstance().runFlywheelBackward(), 
            Shooter.getInstance()
        );
    }

    /**
     * Gets the stop flywheel command for the shooter subsystem.
     * 
     * @return The stop flywheel command.
     */
    public static InstantCommand stopFlywheelCommand() {
        /*
         * Returns an instant command that runs the stop
         * flywheel method.
         */
        return new InstantCommand(
            () -> Shooter.getInstance().stopFlywheel(),
            Shooter.getInstance()
        );
    }
}
