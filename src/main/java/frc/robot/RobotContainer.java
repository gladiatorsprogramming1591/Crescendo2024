
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.VisionDriveAligned;
import frc.robot.commands.WarmUpAndAutoShoot;
import frc.robot.commands.WarmUpShooter;
import frc.robot.commands.AlignAndIntake;
import frc.robot.commands.AlignAndShootNote;
import frc.robot.commands.AlignAndMoonShoot;
import frc.robot.commands.AlignShooterOnly;
import frc.robot.commands.AmpScore;
import frc.robot.commands.IntakeNote;
import frc.robot.commands.IntakeSourcePartOne;
import frc.robot.commands.ShootFast;
import frc.robot.commands.ShootNote;
import frc.robot.commands.TransferOnWithBeamBreak;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.armCommands.ArmToPosition;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.time.Instant;
import java.util.List;
import java.util.function.DoubleSupplier;

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

        // The driver's controller
        CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
        CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

        // The robot's subsystems
        public static CANdleSubsystem m_CANdleSubsystem;
        public final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
        private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
        private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
        
        private SendableChooser<Command> m_autoChooser;

        

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                m_CANdleSubsystem = new CANdleSubsystem();

                // Register Named Commands
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
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftY()
                                                                                                * DriveConstants.kTeleopPercentLimit,
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getLeftX()
                                                                                                * DriveConstants.kTeleopPercentLimit,
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(
                                                                                m_driverController.getRightX()
                                                                                                * DriveConstants.kTeleopPercentLimit,
                                                                                OIConstants.kDriveDeadband),
                                                                true, true),
                                                m_robotDrive));
                m_IntakeSubsystem.setDefaultCommand(
                                new RunCommand(() -> m_IntakeSubsystem.intakeOff(), m_IntakeSubsystem));
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
                m_driverController.back().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
                // m_driverController.a().onTrue(new RunCommand(() ->
                // m_ShooterSubsystem.shooterOn(), m_ShooterSubsystem));
                m_driverController.a().whileTrue(
                                new RunCommand(() -> m_ShooterSubsystem.transferOn(false), m_ShooterSubsystem)
                                                .finallyDo(() -> m_ShooterSubsystem.transferOff()));
                m_driverController.b()
                                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.shooterOff(), m_ShooterSubsystem));
                m_driverController.x().onTrue(new TransferOnWithBeamBreak(m_ShooterSubsystem));
                m_driverController.y()
                                .onTrue(new InstantCommand(() -> m_ShooterSubsystem.transferOff(), m_ShooterSubsystem));
                m_driverController.start().whileTrue(new AlignAndShootNote(m_ShooterSubsystem, m_ArmSubsystem,
                                () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), m_robotDrive,
                                m_IntakeSubsystem));
                m_driverController.rightStick().whileTrue(new AlignAndMoonShoot(m_ShooterSubsystem, m_ArmSubsystem,
                                () -> m_driverController.getLeftX(), () -> m_driverController.getLeftY(), m_robotDrive,
                                m_IntakeSubsystem));
                m_driverController.rightTrigger()
                                .whileTrue(new AlignAndIntake(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem,
                                                m_robotDrive)
                                                .andThen(new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem,
                                                                m_IntakeSubsystem)));
                m_driverController.leftTrigger()
                                .onTrue(new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem));
                m_driverController.leftBumper()
                                .onTrue(new InstantCommand(() -> m_IntakeSubsystem.intakeOn(), m_IntakeSubsystem));
                // m_driverController.rightTrigger(OIConstants.kArmDeadband).whileTrue(new
                // RunCommand(() ->
                // m_ArmSubsystem.ArmBackward(m_driverController.getRightTriggerAxis()),
                // m_ArmSubsystem));
                // m_driverController.rightTrigger(OIConstants.kArmDeadband).onFalse(new
                // InstantCommand(() -> m_ArmSubsystem.ArmOff(), m_ArmSubsystem));
                // m_driverController.leftTrigger(OIConstants.kArmDeadband).whileTrue(new
                // RunCommand(() ->
                // m_ArmSubsystem.ArmForward(m_driverController.getLeftTriggerAxis()),
                // m_ArmSubsystem));
                // m_driverController.leftTrigger(OIConstants.kArmDeadband).onFalse(new
                // InstantCommand(() -> m_ArmSubsystem.ArmOff(), m_ArmSubsystem));
                // m_driverController.povDown().onTrue(new TurnToAngleProfiled(180,
                // m_robotDrive));
                // m_driverController.povLeft().onTrue(new TurnToAngleProfiled(270,
                // m_robotDrive));
                // m_driverController.povUp().onTrue(new TurnToAngleProfiled(0, m_robotDrive));
                // m_driverController.povRight().onTrue(new
                // TurnToAngleProfiled(90,m_robotDrive));
                m_driverController.leftStick()
                                .whileTrue(new RunCommand(() -> m_ShooterSubsystem.transferReverse(),
                                                m_ShooterSubsystem)
                                                .finallyDo((() -> m_ShooterSubsystem.transferOff())));
                m_driverController.povDown().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.TRAP));
                m_driverController.rightBumper().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.CLIMBFINISH,
                                                ArmConstants.kCurrentLimitClimbing).alongWith(new InstantCommand (() -> m_CANdleSubsystem.changeAnimation(AnimationTypes.Twinkle))));
                // m_operatorController.leftStick().onTrue(new AmpScore(m_ShooterSubsystem, m_ArmSubsystem));
                // m_operatorController.rightStick()
                //                 .onTrue(new InstantCommand(() -> m_ShooterSubsystem.transferOn(false),
                //                                 m_ShooterSubsystem));
               m_operatorController.rightStick().onTrue(
                                new ArmToPosition(m_ArmSubsystem, armPositions.AMPFINISH)
                                                .alongWith(new WarmUpShooter(m_ShooterSubsystem, true)));
                                
                m_operatorController.leftStick().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.TRANSFER));
                m_operatorController.rightTrigger().onTrue(new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.SUBWOOFER)); 
                m_operatorController.leftTrigger().onTrue(new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.TRANSFER)); 

                                
                // m_operatorController.x().onTrue(new ShootNote(m_ShooterSubsystem,
                // m_ArmSubsystem, armPositions.STAGELINE));
                m_operatorController.b()
                                .onTrue(new InstantCommand(() -> m_IntakeSubsystem.intakeOff(), m_IntakeSubsystem));
                m_operatorController.y()
                                .whileTrue(new RunCommand(() -> m_IntakeSubsystem.intakeReverse(), m_IntakeSubsystem));
                m_operatorController.a().onTrue(new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem));
                m_operatorController.povDown().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.TRANSFER));
                m_operatorController.povLeft().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.PODIUM));
                m_operatorController.povUp().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.SUBWOOFER));
                m_operatorController.povRight().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.AMP));
                m_operatorController.leftBumper()
                                .whileTrue(new IntakeSourcePartOne(m_ShooterSubsystem, m_ArmSubsystem));
                m_operatorController.rightBumper().onTrue(
                                new ArmToPosition(m_ArmSubsystem, armPositions.TRAP)
                                                .alongWith(new WarmUpShooter(m_ShooterSubsystem, true)));
                m_operatorController.back().onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.CLIMBSTART));
                // m_operatorController.start()
                //                 .onTrue(new ArmToPosition(m_ArmSubsystem, armPositions.CLIMBFINISH,
                //                                 ArmConstants.kCurrentLimitClimbing).alongWith(new InstantCommand (() -> m_CANdleSubsystem.changeAnimation(AnimationTypes.Twinkle))));
                m_operatorController.x().onTrue(new InstantCommand(() -> m_CANdleSubsystem.setAmplify()));
                m_operatorController.x().onFalse(new InstantCommand(() -> m_CANdleSubsystem.setDefault(!m_ShooterSubsystem.isBeamBroken())));

               
                // TODO change the rotation to be the letter buttons
        }

        // * Use this to pass the autonomous command to the main {@link Robot} class.
        // *
        // * @return the command to run in autonomous

        public Command getAutonomousCommand() {
                boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                                .equals(DriverStation.Alliance.Blue);
                if (isBlue) {
                        m_robotDrive.setPipelineIndex(1);
                } else {
                        m_robotDrive.setPipelineIndex(0);
                }

                m_robotDrive.setAutoAimRotPIDConstants(true);

                return m_autoChooser.getSelected();
        }

        public void registerNamedCommands() {
                NamedCommands.registerCommand("ShootSubwoofer",
                                new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.SUBWOOFER));
                NamedCommands.registerCommand("ShootPodium",
                                new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.PODIUM));
                NamedCommands.registerCommand("ShootStageline",
                                new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.STAGELINE));
                NamedCommands.registerCommand("Intake",
                                new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem)
                                                .finallyDo(() -> m_ShooterSubsystem.transferOff()));
                NamedCommands.registerCommand("Intake.25",
                                new IntakeNote(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem)
                                                .withTimeout(0.30));
                NamedCommands.registerCommand("ArmStow",
                                new ArmToPositionWithEnd(m_ArmSubsystem, armPositions.TRANSFER));
                NamedCommands.registerCommand("AlignAndShootNote",
                                new AlignAndShootNote(m_ShooterSubsystem, m_ArmSubsystem, () -> 0, () -> 0,
                                                m_robotDrive,
                                                m_IntakeSubsystem));
                NamedCommands.registerCommand("AlignAndMoonShoot",
                                new AlignAndMoonShoot(m_ShooterSubsystem, m_ArmSubsystem, () -> 0, () -> 0,
                                                m_robotDrive,
                                                m_IntakeSubsystem));
                NamedCommands.registerCommand("ShootFourthNote",
                                new ShootFast(m_ShooterSubsystem, m_ArmSubsystem, armPositions.FOURTHNOTE));
                NamedCommands.registerCommand("AlignAndIntake",
                                new AlignAndIntake(m_ShooterSubsystem, m_ArmSubsystem, m_IntakeSubsystem, m_robotDrive)
                                                .withTimeout(1.5));
                NamedCommands.registerCommand("AlignShooterOnly",
                                new AlignShooterOnly(m_ShooterSubsystem, m_ArmSubsystem, m_robotDrive,
                                                m_IntakeSubsystem));
                NamedCommands.registerCommand("ShootNoteAfterAlign",
                                new WarmUpAndAutoShoot(m_robotDrive, m_ShooterSubsystem, m_ArmSubsystem, false, true)
                                                .beforeStarting(() -> m_IntakeSubsystem.intakeOff()));
                NamedCommands.registerCommand("BeginAutoAlignTheta",
                                new InstantCommand(() -> m_robotDrive.isAutoAimingSpeaker = true));
                NamedCommands.registerCommand("EndAutoAlignTheta",
                                new InstantCommand(() -> {
                                }));

        }

        public void teleopInit() {
                m_robotDrive.setPipelineIndex(2);
                m_robotDrive.setAutoAimRotPIDConstants(false);
                m_ArmSubsystem.ArmOff();
                m_ShooterSubsystem.shooterOff();
        }

}
