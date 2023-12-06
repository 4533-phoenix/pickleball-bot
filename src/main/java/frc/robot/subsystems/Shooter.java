package frc.robot.subsystems;

import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * Contains the code for the shooter subsystem.
 */
public final class Shooter extends SubsystemBase {
    /**
     * The shooter subsystem instance.
     */
    private static Shooter shooter = null;

    /**
     * The flywheel motor.
     */
    private final CANSparkMax flywheelMotor = new CANSparkMax(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    /**
     * The encoder for {@link #flywheelMotor}.
     */
    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    /**
     * The feedforward controller for {@link #flywheelMotor}.
     */
    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV);

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

    /**
     * Constructs the shooter subsystem instance.
     */
    private Shooter() {
        flywheelMotor.setInverted(true);

        flywheelEncoder.setPositionConversionFactor(1.0);
        flywheelEncoder.setVelocityConversionFactor(1.0);
    }

    /**
     * Sets the flywheel motor to run forward at the
     * flywheel velocity.
     */
    public void runFlywheelForward() {
        flywheelMotor.setVoltage(flywheelFeedforward.calculate(ShooterConstants.FLYWHEEL_VELOCITY));
    }

    /**
     * Sets the flywheel motor to run backward at the
     * flywheel velocity.
     */
    public void runFlywheelBackward() {
        flywheelMotor.setVoltage(flywheelFeedforward.calculate(-ShooterConstants.FLYWHEEL_VELOCITY));
    }

    /**
     * Stops the flywheel motor.
     */
    public void stopFlywheel() {
        flywheelMotor.setVoltage(0.0);
    }

    /**
     * The {@link CommandScheduler} runs this method
     * every 20 ms.
     */
    @Override
    public void periodic() {}
}
