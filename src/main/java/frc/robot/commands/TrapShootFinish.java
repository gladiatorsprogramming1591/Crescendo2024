package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.BlowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class TrapShootFinish extends ParallelCommandGroup {

    public TrapShootFinish(ShooterSubsystem shooterSubsystem, BlowerSubsystem blowerSubsystem) {

        addCommands(
                new InstantCommand(()-> BlowerSubsystem.blowerOff(), blowerSubsystem),
                new InstantCommand(() -> shooterSubsystem.shooterOff(), shooterSubsystem)
            );  
                        
    }  
}
