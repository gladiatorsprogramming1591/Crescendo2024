package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.armCommands.ArmToPosition;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootNote extends SequentialCommandGroup {

    public ShootNote(ShooterSubsystem shooterSubsystem, 
                        ArmSubsystem armSubsystem,
                        armPositions position
                        ){

        addCommands(
            new ParallelCommandGroup(
                new ArmToPosition(armSubsystem, position), 
                new WarmUpShooter(shooterSubsystem)
            ),
            new RunCommand(()-> shooterSubsystem.transferOn(false), shooterSubsystem).withTimeout(1.0),
            new InstantCommand(()-> shooterSubsystem.shooterOff(), shooterSubsystem),
            new InstantCommand(()-> shooterSubsystem.transferOff(), shooterSubsystem),
            new ArmToPosition(armSubsystem, armPositions.TRANSFER) //TODO This might need to be moved out for autos

        ); 
    }
}
        