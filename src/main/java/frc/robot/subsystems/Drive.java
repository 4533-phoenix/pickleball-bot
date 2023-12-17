package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.kauailabs.navx.frc.AHRS;

/**
 * Contains the code for the drive subsystem.
 */
public final class Drive extends SubsystemBase {
    /**
     * The drive subsystem instance.
     */
    private static Drive drive = null;

    /**
     * The left leader motor.
     */
    private final CANSparkMax leftLeader = new CANSparkMax(DriveConstants.LEFT_LEADER_ID, MotorType.kBrushless);

    /**
     * The right leader motor.
     */
    private final CANSparkMax rightLeader = new CANSparkMax(DriveConstants.RIGHT_LEADER_ID, MotorType.kBrushless);

    /**
     * The encoder for {@link #leftLeader}.
     */
    private final RelativeEncoder leftEncoder = leftLeader.getEncoder();

    /**
     * The encoder for {@link #rightLeader}.
     */
    private final RelativeEncoder rightEncoder = rightLeader.getEncoder();

    /**
     * The feedforward controller for {@link #leftLeader}.
     */
    private final SimpleMotorFeedforward leftFeedforward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV);

    /**
     * The feedforward controller for {@link #rightLeader}.
     */
    private final SimpleMotorFeedforward rightFeedforward = new SimpleMotorFeedforward(DriveConstants.KS, DriveConstants.KV);

    /**
     * The PID controller for {@link #leftLeader}.
     */
    private final PIDController leftPID = new PIDController(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD);

    /**
     * The PID controller for {@link #rightLeader}.
     */
    private final PIDController rightPID = new PIDController(DriveConstants.KP, DriveConstants.KI, DriveConstants.KD);

    /**
     * The robot gyro.
     */
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    /**
     * The drive position estimator.
     */
    private DifferentialDrivePoseEstimator poseEstimator = null;

    /**
     * The state of slow drive mode.
     */
    private boolean willDriveSlow = false;

    /**
     * Gets the drive subsystem instance.
     * 
     * @return The drive subsystem instance. 
     */
    public static Drive getInstance() {
        /*
         * Constructs the drive subsystem instance if it
         * has not already been constructed.
         */
        if (drive == null) {
            drive = new Drive();
        }

        return drive;
    }

    /**
     * Constructs the drive subsystem instance.
     */
    private Drive() {
        /**
         * Sets the left and right encoders to convert position from motor
         * rotations to distance traveled by the wheel. This involves first
         * converting motor rotations to meters, and then dividing by the
         * gear reduction in order to get the distance traveled by the wheel
         * rather than the motor shaft.
         */
        leftEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.GEAR_REDUCTION);
        rightEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE / DriveConstants.GEAR_REDUCTION);

        /*
         * Sets the left and right encoders to convert velocity from motor rpm 
         * to translational velocity of the wheel. This involves first converting
         * motor rpm to meters per second, and then dividing by the gear reduction
         * in order to get the translational velocity of the wheel rather than
         * the motor shaft.
         */
        leftEncoder.setVelocityConversionFactor(DriveConstants.RPM_TO_METERS_PER_SECOND / DriveConstants.GEAR_REDUCTION);
        rightEncoder.setVelocityConversionFactor(DriveConstants.RPM_TO_METERS_PER_SECOND / DriveConstants.GEAR_REDUCTION);
    }

    /**
     * Registers the drive position estimator.
     * 
     * @param initialPose The initial position of the
     * selected autonomous command.
     */
    public void registerPoseEstimator(Pose2d initialPose) {
        if (poseEstimator == null) {
            poseEstimator = new DifferentialDrivePoseEstimator(
                DriveConstants.DRIVE_KINEMATICS, 
                getAngle(), 
                getLeftDistance(), 
                getRightDistance(), 
                initialPose
            );
        }
    }

    /**
     * Sets the robot to drive according to controller stick input.
     * This method is only used during teleop.
     */
    public void teleopDrive() {
        /*
         * Factor to multiply the left and right velocities by. If slow
         * drive mode is on, it will half them; otherwise, it will do
         * nothing to them.
         */
        double velocityFactor = willDriveSlow ? 0.5 : 1.0;

        /*
         * Sets the left and right velocities of the left and right sides of the
         * robot to be a fraction of the max velocity according to the 
         * -1.0 to 1.0 factor of the left and right controller sticks, as well
         * as the velocity factor.
         */
        double leftVelocity = RobotContainer.getController().getLeftY() * DriveConstants.MAX_VELOCITY * velocityFactor;
        double rightVelocity = RobotContainer.getController().getRightY() * DriveConstants.MAX_VELOCITY * velocityFactor;

        /*
         * Gets the current left and right velocities from the left and 
         * right encoders.
         */
        double currentLeftVelocity = leftEncoder.getVelocity();
        double currentRightVelocity = rightEncoder.getVelocity();

        /*
         * Sets the voltage of the left and right leader motors to the sum
         * of the voltage calculated by the left and right feedforward
         * and PID controllers.
         */
        leftLeader.setVoltage(leftFeedforward.calculate(leftVelocity) + leftPID.calculate(currentLeftVelocity, leftVelocity));
        rightLeader.setVoltage(rightFeedforward.calculate(rightVelocity) + rightPID.calculate(currentRightVelocity, rightVelocity));
    }

    /**
     * Toggles slow drive mode.
     */
    public void toggleSlowDriveMode() {
        willDriveSlow = !willDriveSlow;
    }

    /**
     * Gets the accumulated yaw of the robot.
     * 
     * @return The accumulated yaw of the robot.
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    /**
     * Gets the accumulated distance of the left 
     * side of the robot.
     * 
     * @return The accumulated distance of the
     * left side of the robot.
     */
    public double getLeftDistance() {
        return leftEncoder.getPosition();
    }

    /**
     * Gets the accumulated distance of the right
     * side of the robot.
     * 
     * @return The accumulated distance of the
     * right side of the robot.
     */
    public double getRightDistance() {
        return rightEncoder.getPosition();
    }

    /**
     * Gets the position estimated by the drive
     * position estimator.
     * 
     * @return The position estimated by the
     * drive position estimator.
     */
    public Pose2d getEstimatedPose() {
        return poseEstimator != null ? poseEstimator.getEstimatedPosition() : new Pose2d();
    }

    /**
     * Resets the yaw of the gyro to zero.
     */
    public void resetGyro() {
        gyro.reset();
    }

    /**
     * Gets the left PID controller.
     * 
     * @return The left PID controller.
     */
    public PIDController getLeftPID() {
        return leftPID;
    }

    /**
     * Gets the right PID controller.
     * 
     * @return The right PID controller.
     */
    public PIDController getRightPID() {
        return rightPID;
    }

    /**
     * The {@link CommandScheduler} runs this method
     * every 20 ms.
     */
    @Override
    public void periodic() {
        /*
         * Updates the drive position estimator if it 
         * has been created.
         */
        if (poseEstimator != null) {
            poseEstimator.update(getAngle(), getLeftDistance(), getRightDistance());
        }
    }
}
