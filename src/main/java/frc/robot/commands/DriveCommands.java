package frc.robot.commands;

import frc.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Contains the methods that return the drive commands.
 */
public final class DriveCommands {
    /**
     * Gets the default drive command for the drive subsystem.
     * 
     * @return The default drive command.
     */
    public static RunCommand getDefaultDriveCommand() {
        // Returns a run command that runs the teleop drive method.
        return new RunCommand(
            () -> Drive.getInstance().teleopDrive(),
            Drive.getInstance()
        );
    }

    /**
     * Gets the toggle slow drive mode command from the 
     * drive subsystem.
     *  
     * @return The toggle slow drive mode command.
     */
    public static InstantCommand getToggleSlowDriveModeCommand() {
        /*
         * Returns an instant command that runs the toggle slow
         * drive mode method.
         */
        return new InstantCommand(
            () -> Drive.getInstance().toggleSlowDriveMode(),
            Drive.getInstance()
        );
    }
}


