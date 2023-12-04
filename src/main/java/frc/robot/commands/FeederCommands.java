package frc.robot.commands;

import frc.robot.subsystems.Feeder;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public final class FeederCommands {
    public static InstantCommand normalFireCommand() {
        return new InstantCommand(
            () -> Feeder.getInstance().normalFire(),
            Feeder.getInstance()
        );
    }
    public static InstantCommand rapidFireCommand() {
        return new InstantCommand(
            () -> Feeder.getInstance().rapidFire(),
            Feeder.getInstance()
        );
    }
}