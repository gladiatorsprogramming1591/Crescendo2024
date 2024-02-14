
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.VisionDriveAligned;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.ShootNote;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.armCommands.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    addAutoOptions();
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, false),
            m_robotDrive));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() { 
    m_driverController.start().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_driverController.a().onTrue(new InstantCommand(() -> m_ShooterSubsystem.shooterOn(), m_ShooterSubsystem));
    m_driverController.b().onTrue(new InstantCommand(() -> m_ShooterSubsystem.shooterOff(), m_ShooterSubsystem));
    m_driverController.x().onTrue(new RunCommand(() -> m_ShooterSubsystem.transferOn(true), m_ShooterSubsystem));
    m_driverController.y().onTrue(new InstantCommand(() -> m_ShooterSubsystem.transferOff(), m_ShooterSubsystem));
    m_driverController.back().onTrue(new InstantCommand(() -> m_ShooterSubsystem.transferOn(false), m_ShooterSubsystem));
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_IntakeSubsystem.intakeOn(), m_IntakeSubsystem));
    m_driverController.rightBumper().onTrue(new InstantCommand(() -> m_IntakeSubsystem.intakeOff(), m_IntakeSubsystem));
    m_driverController.rightTrigger(OIConstants.kArmDeadband).whileTrue(new RunCommand(() -> 
      m_ArmSubsystem.ArmBackward(m_driverController.getRightTriggerAxis()), m_ArmSubsystem));
    m_driverController.rightTrigger(OIConstants.kArmDeadband).onFalse(new InstantCommand(() -> m_ArmSubsystem.ArmOff(), m_ArmSubsystem));
    m_driverController.leftTrigger(OIConstants.kArmDeadband).whileTrue(new RunCommand(() -> 
      m_ArmSubsystem.ArmForward(m_driverController.getLeftTriggerAxis()), m_ArmSubsystem));
    m_driverController.leftTrigger(OIConstants.kArmDeadband).onFalse(new InstantCommand(() -> m_ArmSubsystem.ArmOff(), m_ArmSubsystem));
    m_driverController.povDown().onTrue(new TurnToAngleProfiled(180, m_robotDrive)); 
    m_driverController.povLeft().onTrue(new TurnToAngleProfiled(270, m_robotDrive)); 
    m_driverController.povUp().onTrue(new TurnToAngleProfiled(0, m_robotDrive)); 
    m_driverController.povRight().onTrue(new TurnToAngleProfiled(90,m_robotDrive)); 
    m_driverController.leftStick().onTrue(new InstantCommand(() -> m_ShooterSubsystem.transferReverse(), m_ShooterSubsystem)); 
    m_operatorController.rightTrigger().onTrue(new ShootNote(m_ShooterSubsystem, m_ArmSubsystem, armPositions.SUBWOOFER)); 
    m_operatorController.leftTrigger().onTrue(new ShootNote(m_ShooterSubsystem, m_ArmSubsystem, armPositions.PODIUM)); 
    m_operatorController.a().onTrue(new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem)); 
    m_operatorController.povDown().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.TRANSFER)); 
    m_operatorController.povLeft().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.PODIUM)); 
    m_operatorController.povUp().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.SUBWOOFER)); 
    m_operatorController.povRight().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.AMP)); 
    //TODO change the rotation to be the letter buttons 
  }
  public Command getPathplannerCommand(){
    // Create config for trajectory
    // TrajectoryConfig config = new TrajectoryConfig(
    //      AutoConstants.kMaxSpeedMetersPerSecond,
    //     AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    //     // Add kinematics to ensure max speed is actually obeyed
    //     .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 2, Rotation2d.fromDegrees(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config); 

    // var thetaController = new ProfiledPIDController(
    //     AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    // thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
    //     exampleTrajectory,
    //     m_robotDrive::getPose, // Functional interface to feed supplier
    //     DriveConstants.kDriveKinematics,

    //     // Position controllers
    //     new PIDController(AutoConstants.kPXController, 0, 0),
    //     new PIDController(AutoConstants.kPYController, 0, 0),
    //     thetaController,
    //     m_robotDrive::setModuleStates,
    //     m_robotDrive);

        // PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");
        // Pose2d initialpose = new Pose2d(2, 7, Rotation2d.fromDegrees(0)); 

        PathPlannerPath path = PathPlannerPath.fromPathFile("Copy of Example Path");
        Pose2d initialpose = new Pose2d(0, 1, Rotation2d.fromDegrees(0)); 
        // Reset odometry to the starting pose of the trajectory.
         m_robotDrive.resetOdometry(initialpose);

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        
        return AutoBuilder.followPath(path);

  }


  private void addAutoOptions() {
    m_autoChooser.setDefaultOption("Copy Of Example Path", getPathplannerCommand());
    m_autoChooser.addOption("Vision Drive Aligned", new VisionDriveAligned(5, 0, 0, m_robotDrive));
    SmartDashboard.putData("Auto Mode", m_autoChooser);
  }
  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
   
  public Command getAutonomousCommand() { //TODO use pathplanner auto 
    return m_autoChooser.getSelected();
    }
}   
