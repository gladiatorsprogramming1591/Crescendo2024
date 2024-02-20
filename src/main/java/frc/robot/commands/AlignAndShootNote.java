// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndShootNote extends SequentialCommandGroup {

  public AlignAndShootNote(ShooterSubsystem shooterSubsystem,
      ArmSubsystem armSubsystem,
      DoubleSupplier x,
      DoubleSupplier y,
      DriveSubsystem driveSubsystem) {

    addCommands(
        new ParallelDeadlineGroup(
           new WarmUpAndAutoShoot(driveSubsystem, shooterSubsystem),
            new RunCommand(() -> armSubsystem
                .ArmToPosition(DriveConstants.DISTANCE_TO_ANGLE_MAP.get(driveSubsystem.getDistanceToSpeaker())), armSubsystem)),
            driveSubsystem.driveOnTargetSpeaker(x, y),
        new InstantCommand(() -> shooterSubsystem.shooterOff(), shooterSubsystem),
        new InstantCommand(() -> shooterSubsystem.transferOff(), shooterSubsystem)

    );
  }
}
