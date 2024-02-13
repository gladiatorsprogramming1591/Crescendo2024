package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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
                        ArmSubsystem armSubsystem
                        ){

        addCommands(
            new ArmToPosition(armSubsystem, armPositions.SUBWOOFER), 
            new ParallelRaceGroup( 
                new RunCommand(()-> shooterSubsystem.shooterOn(), shooterSubsystem),
                new RunCommand(()-> shooterSubsystem.transferOn(false), shooterSubsystem)
            ), 
            new InstantCommand(()-> shooterSubsystem.shooterOff(), shooterSubsystem),
            new InstantCommand(()-> shooterSubsystem.transferOff(), shooterSubsystem)

        ); 
                        }
}
        