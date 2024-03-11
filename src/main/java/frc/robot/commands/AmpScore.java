package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class AmpScore extends SequentialCommandGroup {

    public AmpScore(ShooterSubsystem shooterSubsystem, 
                        ArmSubsystem armSubsystem,
                        armPositions position
                        ){

        addCommands(
            new ParallelCommandGroup(
                new ArmToPositionWithEnd(armSubsystem, position)
                // new RunCommand(() -> shooterSubsystem.shooterOn(), shooterSubsystem).withTimeout(1.0)
            ).withTimeout(0.75),
            new RunCommand(()-> shooterSubsystem.transferReverse(), shooterSubsystem).withTimeout(0.15),
            new InstantCommand(()-> shooterSubsystem.transferOff(), shooterSubsystem)
        ); 
    }
}