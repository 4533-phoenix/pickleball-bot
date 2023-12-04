package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Shooter;

public final class ShooterCommands {
    public static RunCommand runFlywheelForwardCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runFlywheelForward(), 
            Shooter.getInstance()
        );
    }

    public static RunCommand runFlywheelBackwardCommand() {
        return new RunCommand(
            () -> Shooter.getInstance().runFlywheelBackward(), 
            Shooter.getInstance()
        );
    }

    public static InstantCommand stopFlywheelCommand() {
        return new InstantCommand(
            () -> Shooter.getInstance().stopFlywheel(),
            Shooter.getInstance()
        );
    }
}
