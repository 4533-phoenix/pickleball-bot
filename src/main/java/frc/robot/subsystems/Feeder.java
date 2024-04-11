package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;

/*
            The feeder
    - ` , WITH RAPID FIRE ` , -

    The feeder has a two-servo system.
    
    In normal mode:
     1. The "preliminary" servo is set to the open position
     2. A ball drops down to the second barrier
     3. The "preliminary" servo is set to the closed position
     4. The "firing" servo is set to the open position
     5. The ball drops into the shooter
     6. The "firing" servo is set to the closed position to
        prepare for the next trigger
    
    In rapid fire mode, both servos are set to the open
    position and all balls drop into the shooter.
*/
public final class Feeder extends SubsystemBase {
    private static Feeder feeder = null;

    // Initialize the servo objects
    private final Servo preliminaryServo = new Servo(FeederConstants.PRELIMINARY_SERVO_ID);
    private final Servo firingServo = new Servo(FeederConstants.FIRING_SERVO_ID);

    public static Feeder getInstance() {
        if (feeder == null) {
            feeder = new Feeder();
        }

        return feeder;
    }

    private Feeder() {
    }

    public void noFire() {
        this.setPreliminary(false);
        this.setFiring(false);
    }

    // public void normalFire() {
    //     this.setPreliminary(true);
    //     Timer.delay(FeederConstants.DROP_DELAY * 0.001);
    //     this.setPreliminary(false);
    //     this.setFiring(true);
    //     Timer.delay(FeederConstants.DROP_DELAY * 0.001);
    //     this.setFiring(false);
    // }

    // Rapid fire!!1!1!
    public void rapidFire() {
        // Set both servos to the open position
        this.setFiring(true);
        this.setPreliminary(true);
    }

    // Open/close the preliminary servo
    public void setPreliminary(boolean open) {
        if (open)
            this.preliminaryServo.setAngle(FeederConstants.OPEN_ANGLE);
        else
            this.preliminaryServo.setAngle(FeederConstants.SEMI_CLOSED_ANGLE);
    }

    // Open/close the firing servo
    public void setFiring(boolean open) {
        if (open)
            this.firingServo.setAngle(FeederConstants.OPEN_ANGLE);
        else
            this.firingServo.setAngle(FeederConstants.SEMI_CLOSED_ANGLE);
    }
}
