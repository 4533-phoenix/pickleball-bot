// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Drive;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public final class Robot extends TimedRobot {
  /**
   * The key for the selected autonomous command.
   */
  private String autoSelected = "Default Auto";

  /**
   * The selected autonomous command.
   */
  private Command autoCommand = null;

  /**
   * The initial position of the selected autonomous
   * command.
   */
  private Pose2d autoPosition = null;
  
  /**
   * The autonomous chooser for the autonomous commands.
   */
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Resets the yaw of the gyro to zero.
    Drive.getInstance().resetGyro();

    // Registers the subsystems with the command scheduler.
    RobotContainer.registerSubsystems();

    // Registers the button commands with their respective buttons.
    RobotContainer.registerButtons();

    // Sets the options and default option for the autonomous chooser.
    autoChooser.setDefaultOption("Default Auto", "Default Auto");
    autoChooser.addOption("First Auto", "First Auto");

    // Sends the autonomous chooser to SmartDashboard.
    SmartDashboard.putData("Auto Commands", autoChooser);

    SmartDashboard.putNumber("Kp", 0.0);
    SmartDashboard.putNumber("Ki", 0.0);
    SmartDashboard.putNumber("Kd", 0.0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the command scheduler.
    CommandScheduler.getInstance().run();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    /*
     * Gets the key for the autonomous command selected from the 
     * autonomous chooser in SmartDashboard.
     */
    autoSelected = autoChooser.getSelected();

    /*
     * Gets the selected autonomous command using the key retrieved
     * from the autonomous chooser.
     */
    autoCommand = RobotContainer.getAutoCommand(autoSelected);

    /*
     * Gets the initial position of the selected autonomous command
     * using the key retrieved from the autonomous chooser.
     */
    autoPosition = RobotContainer.getAutoPosition(autoSelected);

    /*
     * Registers the drive position estimator using the initial
     * position of the selected autonomous command.
     */
    Drive.getInstance().registerPoseEstimator(autoPosition);
    
    // Schedules the selected autonomous command.
    CommandScheduler.getInstance().schedule(autoCommand);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /*
     * Cancels the selected autonomous command at the start
     * of teleop. This ensures that the autonomous command
     * stops if it continues running in teleop.
     */
    if (autoCommand != null) {
      CommandScheduler.getInstance().cancel(autoCommand);
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // double p = SmartDashboard.getNumber("Kp", 0.0);
    // double i = SmartDashboard.getNumber("Ki", 0.0);
    // double d = SmartDashboard.getNumber("Kd", 0.0);

    // Drive.getInstance().getLeftPID().setPID(p, i, d);
    // Drive.getInstance().getRightPID().setPID(p, i, d);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
