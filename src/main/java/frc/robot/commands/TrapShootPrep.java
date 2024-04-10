package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.armCommands.ArmToPositionWithEnd;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlowerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class TrapShootPrep extends ParallelCommandGroup {

    public TrapShootPrep(ShooterSubsystem shooterSubsystem,
                        ArmSubsystem armSubsystem,
                        armPositions position, BlowerSubsystem blowerSubsystem) {

        addCommands(
                new InstantCommand(()-> BlowerSubsystem.blowerOn(), blowerSubsystem), 
                new ArmToPositionWithEnd(armSubsystem, armPositions.TRAP), 
                new WarmUpShooter(shooterSubsystem, true)
        );  
                        
    }
            
}
