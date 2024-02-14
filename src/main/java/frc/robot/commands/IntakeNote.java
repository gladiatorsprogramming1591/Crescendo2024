package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeNote extends SequentialCommandGroup {

    public IntakeNote(ShooterSubsystem shooterSubsystem, 
                        ArmSubsystem armSubsystem,
                        IntakeSubsystem intakeSubsystem
                        ){

        addCommands(
            new ArmToPositionWithEnd(armSubsystem, armPositions.TRANSFER), 
            new ParallelRaceGroup(
                new WarmUpShooter(shooterSubsystem), 
                new RunCommand(()-> intakeSubsystem.intakeOn(), intakeSubsystem)
            ), 
            new InstantCommand(()-> intakeSubsystem.intakeOff(), intakeSubsystem)

        ); 
                        }
}
        