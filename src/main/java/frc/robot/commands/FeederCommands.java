package frc.robot.commands;

import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Contains the methods that return the feeder commands.
 */
public final class FeederCommands {
    /**
     * Gets the rapid fire command for the
     * feeder subsystem.
     * 
     * @return The rapid fire command.
     */
    public static Command stopFireCommand() {
        /*
         * Returns an instant command that runs the rapid
         * fire command.
         */
        return new InstantCommand(
                () -> Feeder.getInstance().noFire(),
                Feeder.getInstance());
    }

    /**
     * Gets the normal fire command for the
     * feeder subsystem.
     * 
     * @return The normal fire command.
     */
    public static Command normalFireCommand() {
        /*
         * Returns an instant command that runs the normal
         * fire command.
         */
        return new SequentialCommandGroup(
                new InstantCommand(
                        () -> Feeder.getInstance().setPreliminary(true),
                        Feeder.getInstance()),
                new WaitCommand(FeederConstants.DROP_DELAY),
                new InstantCommand(
                        () -> Feeder.getInstance().setPreliminary(false),
                        Feeder.getInstance()),
                new InstantCommand(
                        () -> Feeder.getInstance().setFiring(true),
                        Feeder.getInstance()),
                new WaitCommand(FeederConstants.DROP_DELAY),
                new InstantCommand(
                        () -> Feeder.getInstance().setFiring(false),
                        Feeder.getInstance()));
    }

    /**
     * Gets the rapid fire command for the
     * feeder subsystem.
     * 
     * @return The rapid fire command.
     */
    public static Command rapidFireCommand() {
        /*
         * Returns an instant command that runs the rapid
         * fire command.
         */
        return new InstantCommand(
                () -> Feeder.getInstance().rapidFire(),
                Feeder.getInstance());
    }
}