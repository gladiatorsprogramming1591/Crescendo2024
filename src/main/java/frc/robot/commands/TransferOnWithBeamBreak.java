package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class TransferOnWithBeamBreak extends Command {
    private ShooterSubsystem m_shooter; 

    public TransferOnWithBeamBreak(ShooterSubsystem shooterSubsystem){
        m_shooter = shooterSubsystem; 
        addRequirements(m_shooter);
    }

    @Override
    public void execute(){
       m_shooter.transferOn(true);
    }

    @Override
    public boolean isFinished(){
        return m_shooter.isBeamBroken(); 
        
    }

    @Override
    public void end(boolean isInterrupted){
        m_shooter.transferOff();
    }
}
