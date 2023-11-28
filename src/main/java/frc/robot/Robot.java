// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  XboxController controller;

  SwerveDrive frontLeft, frontRight, backLeft, backRight;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    controller = new XboxController(Constants.IO.controller);

    frontLeft = new SwerveDrive(Constants.IO.frontLeftDrive, Constants.IO.frontLeftTurn, Constants.Turn.angleOffsets[1]);
    frontRight = new SwerveDrive(Constants.IO.frontRightDrive, Constants.IO.frontRightTurn, Constants.Turn.angleOffsets[0]);
    backLeft = new SwerveDrive(Constants.IO.backLeftDrive, Constants.IO.backLeftTurn, Constants.Turn.angleOffsets[3]);
    backRight = new SwerveDrive(Constants.IO.backRightDrive, Constants.IO.backRightTurn, Constants.Turn.angleOffsets[2]);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

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
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //reset all of our variables
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(controller.getRawButtonReleased(1)){
      double angle = 0;
      frontLeft.setAngle(angle);
      frontRight.setAngle(angle);
      backRight.setAngle(angle);
      backLeft.setAngle(angle);
    }
    if(controller.getRawButtonReleased(2)){
      double angle = Math.PI * 1.5;
      frontLeft.setAngle(angle);
      frontRight.setAngle(angle);
      backRight.setAngle(angle);
      backLeft.setAngle(angle);
    }
    if(controller.getRawButtonReleased(3)){
      double angle = Math.PI * 0.5;
      frontLeft.setAngle(angle);
      frontRight.setAngle(angle);
      backRight.setAngle(angle);
      backLeft.setAngle(angle);
    }
    if(controller.getRawButtonReleased(4)){
      double angle = Math.PI;
      frontLeft.setAngle(angle);
      frontRight.setAngle(angle);
      backRight.setAngle(angle);
      backLeft.setAngle(angle);
    }
    frontLeft.setSpeed(1);
    frontRight.setSpeed(1); 
    backRight.setSpeed(1);
    backLeft.setSpeed(1);
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
