// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IO;
import frc.robot.commands.Autos;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private SwerveDrive m_robotDrive;
  private Claw claw = new Claw();
  private Arm arm = new Arm();
  private boolean armExtended = false;

  private List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("Blue" + Constants.Autonomous.autoPath,
      new PathConstraints(3, 2.5)); // 3, 2.5

  SwerveAutoBuilder autoBuilder;

  Limelight limelight = new Limelight(m_robotDrive);
  Autos autonomous;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  Joystick m_driverController = new Joystick(IO.driveController);
  XboxController m_armController = new XboxController(IO.armController);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    // LL port forwarding
    for (int port = 5800; port <= 5805; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }

    SmartDashboard.putNumber("translation-pid", Constants.Autonomous.TRANSLATION_PID);
    SmartDashboard.putNumber("rotation-pid", Constants.Autonomous.ROTATION_PID);

    // Read initial pose
    // REMINDER: get initial pose
    // Pose2d initialPose = readInitialPose("/");
    m_robotDrive = new SwerveDrive(limelight, pathGroup.get(0).getInitialHolonomicPose());
    autonomous = new Autos(m_robotDrive, limelight, arm, claw, pathGroup);
    autoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose, // Pose2d supplier
        m_robotDrive::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
        Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
        new PIDConstants(Constants.Autonomous.TRANSLATION_PID, 0, 0), // between 10.9 and 11
        // PID controllers)
        new PIDConstants(Constants.Autonomous.ROTATION_PID, 0, 0.2), // PID constants to correct for rotation error (used to create the rotation
                                       // controller)
        m_robotDrive::setModuleStates, // Module states consumer used to output to the drive subsystem
        autonomous.getEventMap(),
        true, // Should the path be automatically mirrored depending on alliance color.
              // Optional, defaults to true
        m_robotDrive // The drive subsystem. Used to properly set the requirements of path following
                     // commands
    );

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getY(), IO.kDriveDeadband), // add cube here
                -MathUtil.applyDeadband(m_driverController.getX(), IO.kDriveDeadband),
                m_driverController.getRawButton(2) ? -MathUtil.applyDeadband(m_driverController.getZ(), 0.4)
                    : ((m_driverController.getPOV() == 45 || m_driverController.getPOV() == 90
                        || m_driverController.getPOV() == 135)
                            ? -0.5
                            : (m_driverController.getPOV() == 225 || m_driverController.getPOV() == 270
                                || m_driverController.getPOV() == 315) ? 0.5 : 0)),
            m_robotDrive));

    arm.setDefaultCommand(
        new RunCommand(
            () -> arm.goToLocation(arm.shoulderSetpoint, arm.elbowSetpoint),
            arm));
  }

  // controllers for person with the Xbox
  private void configureBindings() {

    // intake the object with CLAW when A is pressed
    Trigger leftTrigger = new Trigger(() -> m_armController.getLeftTriggerAxis() >= 0.75);
    Trigger rightTrigger = new Trigger(() -> m_armController.getRightTriggerAxis() >= 0.75);

    //Right trigger starts intake until it picks up an object
    rightTrigger
        .onTrue(
            new FunctionalCommand(
                () -> System.out.println("Initiating Claw Intake"),
                () -> claw.intakeObject(),
                interrupted -> {
                  new WaitCommand(0.1);
                  claw.stop();
                },
                () -> claw.isStuck(), // ||!claw.hasCone()||!claw.hasCube(),
                claw));

    //Left trigger releases the object or can be used to stop intake
    leftTrigger
        .whileTrue(new RunCommand(() -> {
          claw.releaseObject();
        })).onFalse(new InstantCommand(() -> claw.stop(), claw));
    Trigger leftBumper = new Trigger(() -> m_armController.getLeftBumper());
    Trigger rightBumper = new Trigger(() -> m_armController.getRightBumper());
    
    //Left bumper to log arm values for debugging
    leftBumper
        .onTrue(new InstantCommand(() -> {
          Constants.ArmConstants.ELBOW_SUBSTATION_ANGLE+=0.8;
          arm.substation();
        }));
         //Left bumper to log arm values for debugging
    rightBumper
    .onTrue(new InstantCommand(() -> {
      Constants.ArmConstants.ELBOW_SUBSTATION_ANGLE-=0.8;
      arm.substation();
    }));

    //Debugging
    /*rightBumper.onTrue(new InstantCommand(()->{
      
    }));*/
      /*new RunCommand(() -> {
      m_robotDrive.autoBalance(true);
    })*/

    Trigger dPadDown = new Trigger(
        () -> m_armController.getPOV() == 135 || m_armController.getPOV() == 180 || m_armController.getPOV() == 225);
    Trigger dPadUp = new Trigger(
        () -> m_armController.getPOV() == 315 || m_armController.getPOV() == 0 || m_armController.getPOV() == 45);

    //D Pad Down to move arm to low pole cone
    dPadDown
        .onTrue(new InstantCommand(() -> {
          arm.lowPoleCone();
        }));

    //D Pad Up to move arm to high pole cone
    dPadUp.onTrue(new InstantCommand(() -> {
      armExtended = true;

      arm.highPoleCone();
    }));

    //A to go to low platform cube
    new JoystickButton(m_armController, XboxController.Button.kA.value)
        .onTrue(new InstantCommand(() -> {
          arm.lowPlatformCube();
        }));

    //Y to go to high platform cube
    new JoystickButton(m_armController, XboxController.Button.kY.value)
        .onTrue(new InstantCommand(() -> {
          armExtended = true;

          arm.highPlatformCube();
        }));

    //B to go to substation
    new JoystickButton(m_armController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(() -> {
          armExtended = true;
          arm.substation();
        }));

    //move arm home
    (new JoystickButton(m_armController, XboxController.Button.kBack.value))
        .onTrue(new ConditionalCommand(returnHomeElbowFirst(), returnHome(), new Trigger(() -> armExtended)));

    //move the arm to floor
    (new JoystickButton(m_armController, XboxController.Button.kStart.value)).onTrue(new InstantCommand(() -> {
      arm.moveToFloor();
    }));

    //move shoulder up manually
    (new JoystickButton(m_armController, XboxController.Button.kLeftStick.value)).whileTrue(new RunCommand(() -> {
      arm.setShoulderOutput(-m_armController.getLeftY() * 0.3);
    })).onFalse(new InstantCommand(() -> {
      arm.setSetpoints();
    }));

   

    //move elbow up manually
    (new JoystickButton(m_armController, XboxController.Button.kRightStick.value)).whileTrue(new RunCommand(() -> {
      arm.setElbowOutput(m_armController.getRightY() * 0.3);
    })).onFalse(new InstantCommand(() -> {
      arm.setSetpoints();
    }));

    //reset the gyro to reset field orientation
    (new JoystickButton(m_driverController, 1)).onTrue(new InstantCommand(() -> {
      resetGyro();
    }));

    // Change Gears to low gear
    (new JoystickButton(m_driverController, 3)).onTrue(new InstantCommand(() -> {
      m_robotDrive.switchGear(Constants.Drive.lowGear);
    }));

    // Change gears to high gear
    (new JoystickButton(m_driverController, 5)).onTrue(new InstantCommand(() -> {
      m_robotDrive.switchGear(Constants.Drive.highGear);
    }));
     //move shoulder up manually
     (new JoystickButton(m_driverController, 12)).whileTrue(new RunCommand(() -> {
      m_robotDrive.lockWheels();
    }));
  }

  //reset the gyrometer to zero deg
  public void resetGyro() {
    m_robotDrive.zeroHeading();
  }

  //return home but with the elbow moving in first
  public Command returnHomeElbowFirst() {
    return (new InstantCommand(() -> {
      armExtended = false;
      arm.setElbowBase();
    }).andThen((new WaitCommand(0.5))).andThen(new InstantCommand(() -> arm.setShoulderBase()))); // Was .7
  }

  //return home normally
  public Command returnHome() {
    return new InstantCommand(() -> {
      arm.setElbowBase();
      arm.setShoulderBase();
    });
  }

  //blue 1 and blue 3
  public Command getAutonomousCommand() {
    //Changed: new SwerveAutoBuilder here, path at beginning, smart dashboard, undo all
    
    SequentialCommandGroup seq = new SequentialCommandGroup();
    seq.addCommands(
        new InstantCommand(() -> {
          m_robotDrive.resetOdometry(pathGroup.get(0).getInitialHolonomicPose());
        }),
        // Move arm down to read limelight, then move arm up
        new InstantCommand(() -> arm.quickCubeAngle()),
        new WaitUntilCommand(() -> arm.reached()),
        new InstantCommand(() -> claw.releaseObjectAuto()), 
        new WaitCommand(1),
        new InstantCommand(() -> claw.stop()),
        // Go to Initial Point
        new ProxyCommand(() -> new SequentialCommandGroup(
            autonomous.toPosition(new Pose2d(m_robotDrive.getPose().getX() + 0.75, m_robotDrive.getPose().getY() + 0.25,
                Rotation2d.fromDegrees(180)), false),
            new ParallelDeadlineGroup(new SequentialCommandGroup(
                new InstantCommand(() -> arm.moveToFloor()),
                new WaitUntilCommand(() -> arm.reached()),
                returnHome()), new RunCommand(() -> m_robotDrive.addVision())),
            new ProxyCommand(() -> new SequentialCommandGroup(
                autonomous.toPosition(pathGroup.get(0).getInitialPose(), true),
                autoBuilder.fullAuto(pathGroup)
               // returnHomeElbowFirst()
                )))));
    return seq;
  }

  //blue 2: middle path with the autobalance path.
  public Command getAutoBalance() {
    SequentialCommandGroup seq = new SequentialCommandGroup();
    seq.addCommands(
        new InstantCommand(() -> m_robotDrive.zeroHeading()),
        // release cube
        /*new InstantCommand(() -> arm.quickCubeAngle()),
        new WaitUntilCommand(() -> arm.reached()),
        new InstantCommand(() -> claw.releaseObject()),
        new WaitCommand(1),
        new InstantCommand(() -> claw.stop()),
        returnHome(),*/
        // move backwards 5 seconds
        new RunCommand(() -> m_robotDrive.drive(-0.3, 0, 0)).until(() -> m_robotDrive.overTheThaang()),
        // move forward until angle BangBang
        new ProxyCommand(()->new SequentialCommandGroup(
            new RunCommand(() -> m_robotDrive.drive(0.3, 0, 0)).until(()->{
              System.out.println("Angle on the way back: " + m_robotDrive.getAngle());
              return Math.abs(m_robotDrive.getAngle()) > 5;
            }),
            // autobalance
            new RunCommand(() -> m_robotDrive.autoBalance(false), m_robotDrive)
        ))
      );
    return seq;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
   * public Command getAutonomousCommand() {
   * return new RunCommand(null, null);
   * }
   */
  //supplier for rotation in drive function
  public double getRotation() {
    if (m_driverController.getPOV() == 90) {
      return -0.4;
    } else if (m_driverController.getPOV() == 270) {
      return 0.4;
    }
    return 0.0;
  }

  public void getArmValues(){
    arm.logPotentiometerValues();
  }
}
