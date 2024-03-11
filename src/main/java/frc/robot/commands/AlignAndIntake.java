package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.IntakeSubsystem;

public class AlignAndIntake extends SequentialCommandGroup {

        public AlignAndIntake(ShooterSubsystem shooterSubsystem,
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem,
                        DriveSubsystem driveSubsystem) {
                addRequirements(shooterSubsystem, armSubsystem, intakeSubsystem, driveSubsystem);
                addCommands(
                                new ParallelRaceGroup(
                                                new RunCommand(() -> driveSubsystem.driveRobotRelativeToObject(),
                                                                driveSubsystem),
                                                new SequentialCommandGroup(
                                                                new ArmToPositionWithEnd(armSubsystem,
                                                                                armPositions.TRANSFER),
                                                                new ParallelDeadlineGroup(
                                                                                new TransferOnWithBeamBreak(
                                                                                                shooterSubsystem),
                                                                                new RunCommand(() -> intakeSubsystem
                                                                                                .intakeOn(),
                                                                                                intakeSubsystem)),
                                                                new InstantCommand(
                                                                                () -> intakeSubsystem.intakeOff()))));

        }
}
