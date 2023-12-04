package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public final class Shooter extends SubsystemBase {
    private static Shooter shooter = null;

    private final CANSparkMax flywheelMotor = new CANSparkMax(ShooterConstants.FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

    private final RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

    private final SimpleMotorFeedforward flywheelFeedforward = new SimpleMotorFeedforward(ShooterConstants.KS, ShooterConstants.KV);

    public static Shooter getInstance() {
        if (shooter == null) {
            shooter = new Shooter();
        }

        return shooter;
    }

    private Shooter() {
        flywheelMotor.setInverted(true);

        flywheelEncoder.setPositionConversionFactor(1.0);
        flywheelEncoder.setVelocityConversionFactor(1.0);
    }

    public void runFlywheelForward() {
        flywheelMotor.setVoltage(flywheelFeedforward.calculate(ShooterConstants.FLYWHEEL_VELOCITY));
    }

    public void runFlywheelBackward() {
        flywheelMotor.setVoltage(flywheelFeedforward.calculate(-ShooterConstants.FLYWHEEL_VELOCITY));
    }

    public void stopFlywheel() {
        flywheelMotor.setVoltage(0.0);
    }

    @Override
    public void periodic() {}
}
