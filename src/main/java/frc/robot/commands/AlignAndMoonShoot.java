// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndMoonShoot extends SequentialCommandGroup {

  public AlignAndMoonShoot(ShooterSubsystem shooterSubsystem,
      ArmSubsystem armSubsystem,
      DoubleSupplier x,
      DoubleSupplier y,
      DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem) {
    addRequirements(driveSubsystem);

    addCommands(
        new InstantCommand(() -> RobotContainer.m_CANdleSubsystem.changeAnimation(AnimationTypes.Larson)),
        new InstantCommand(() -> intakeSubsystem.intakeOff(), intakeSubsystem),
        new ParallelDeadlineGroup(
            new WarmUpAndMoonShoot(driveSubsystem, shooterSubsystem, armSubsystem, true, false),
            new RunCommand(() -> driveSubsystem.driveOnMoonshotTarget(x, y), driveSubsystem)),
        new InstantCommand(() -> shooterSubsystem.shooterOff(), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.transferOff(), shooterSubsystem),
        new InstantCommand(() -> armSubsystem.ArmOff(), armSubsystem),
        new InstantCommand(() -> RobotContainer.m_CANdleSubsystem.setDefault(!shooterSubsystem.isBeamBroken()))
    );
  }
}
