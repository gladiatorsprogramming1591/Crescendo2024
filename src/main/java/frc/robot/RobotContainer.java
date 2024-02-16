
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
import frc.robot.commands.TransferOnWithBeamBreak;
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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {  
  
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private SendableChooser<Command> m_autoChooser; 

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Register Named Commands 
    registerNamedCommands();

    m_autoChooser = AutoBuilder.buildAutoChooser(); 
    SmartDashboard.putData("Auto Mode", m_autoChooser);
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * DriveConstants.kTeleopPercentLimit, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * DriveConstants.kTeleopPercentLimit, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * DriveConstants.kTeleopPercentLimit, OIConstants.kDriveDeadband),
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
    m_driverController.x().onTrue(new TransferOnWithBeamBreak(m_ShooterSubsystem));
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

  //  * Use this to pass the autonomous command to the main {@link Robot} class.
  //  *
  //  * @return the command to run in autonomous
   
  public Command getAutonomousCommand() { //TODO use pathplanner auto 
    return m_autoChooser.getSelected(); 
  }

  public void registerNamedCommands(){
    NamedCommands.registerCommand("ShootSubwoofer", new ShootNote(m_ShooterSubsystem, m_ArmSubsystem, armPositions.SUBWOOFER)); 
    NamedCommands.registerCommand("ShootPodium", new ShootNote(m_ShooterSubsystem, m_ArmSubsystem, armPositions.PODIUM)); 
    NamedCommands.registerCommand("Intake", new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem)); 
  
  }
}   
