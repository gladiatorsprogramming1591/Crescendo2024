package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class ShootNote extends SequentialCommandGroup {

    public ShootNote(ShooterSubsystem shooterSubsystem, 
                        ArmSubsystem armSubsystem,
                        armPositions position
                        ){

        addCommands(
            new ParallelCommandGroup(
                new ArmToPositionWithEnd(armSubsystem, position), 
                new RunCommand(() -> shooterSubsystem.shooterOn(), shooterSubsystem).withTimeout(1.0)
                // new WarmUpShooter(shooterSubsystem
            ),
            new RunCommand(()-> shooterSubsystem.transferOn(false), shooterSubsystem).withTimeout(1.0),
            new InstantCommand(()-> shooterSubsystem.shooterOff(), shooterSubsystem),
            new InstantCommand(()-> shooterSubsystem.transferOff(), shooterSubsystem),
            new ArmToPositionWithEnd(armSubsystem, armPositions.TRANSFER) //TODO This might need to be moved out for autos

        ); 
    }
}
        