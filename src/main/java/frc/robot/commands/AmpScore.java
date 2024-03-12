package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class AmpScore extends SequentialCommandGroup {

    public AmpScore(ShooterSubsystem shooterSubsystem, 
                        ArmSubsystem armSubsystem
                        ){

        addCommands(
            new ParallelCommandGroup(
                new ArmToPositionWithEnd(armSubsystem, armPositions.AMP)
                // new RunCommand(() -> shooterSubsystem.shooterOn(), shooterSubsystem).withTimeout(1.0)
            ).withTimeout(1.25),
            new ParallelCommandGroup(
                            new RunCommand(()-> shooterSubsystem.transferReverse(), shooterSubsystem).withTimeout(0.25),
                            new SequentialCommandGroup(new WaitCommand(0.1), new ArmToPositionWithEnd(armSubsystem, armPositions.AMPFINISH))
            ),
            new InstantCommand(()-> shooterSubsystem.transferOff(), shooterSubsystem)
        ); 
    }
}