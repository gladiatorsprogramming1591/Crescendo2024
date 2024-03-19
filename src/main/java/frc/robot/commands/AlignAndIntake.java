package frc.robot.commands;

import java.time.Instant;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.CANdleSubsystem.AnimationTypes;
import frc.robot.subsystems.IntakeSubsystem;

public class AlignAndIntake extends SequentialCommandGroup {

    public AlignAndIntake(ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem,
            DriveSubsystem driveSubsystem) {
        addRequirements(shooterSubsystem, armSubsystem, intakeSubsystem, driveSubsystem);
        addCommands(
                new InstantCommand(() -> RobotContainer.m_CANdleSubsystem.setAutoAimOrShoot()),
                new InstantCommand(() -> driveSubsystem.resetNoteHeight()),
                new ParallelRaceGroup(
                        new FinishWhenBeamBroken(shooterSubsystem),
                        new RunCommand(() -> driveSubsystem.driveRobotRelativeToObject(),
                                driveSubsystem)
                                .until(() -> driveSubsystem.isTargetLost()),
                        new SequentialCommandGroup(
                                new ArmToPositionWithEnd(armSubsystem,
                                        armPositions.TRANSFER),
                                new RunCommand(() -> {
                                    intakeSubsystem.intakeOn();
                                    shooterSubsystem.transferOn(true);
                                }, intakeSubsystem, shooterSubsystem))),
                new InstantCommand(() -> RobotContainer.m_CANdleSubsystem.setDefault())
        );

    }
}
