package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * Contains the methods that return the feeder commands.
 */
public final class FeederCommands {
    /**
     * Gets the normal fire command for the
     * feeder subsystem.
     * 
     * @return The normal fire command.
     */
    public static InstantCommand normalFireCommand() {
        /*
         * Returns an instant command that runs the normal
         * fire command.
         */
        return new InstantCommand(
            () -> Feeder.getInstance().normalFire(),
            Feeder.getInstance()
        );
    }

    /**
     * Gets the rapid fire command for the
     * feeder subsystem.
     * 
     * @return The rapid fire command.
     */
    public static InstantCommand rapidFireCommand() {
        /*
         * Returns an instant command that runs the rapid
         * fire command.
         */
        return new InstantCommand(
            () -> Feeder.getInstance().rapidFire(),
            Feeder.getInstance()
        );
    }
}