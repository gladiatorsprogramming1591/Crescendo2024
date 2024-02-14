package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpShooter extends Command {
    private ShooterSubsystem m_shooter; 

    public WarmUpShooter(ShooterSubsystem shooterSubsystem){
        m_shooter = shooterSubsystem; 
        addRequirements(m_shooter);
    }

    @Override
    public void execute(){
       m_shooter.shooterOn();
    }

    @Override
    public boolean isFinished(){
        return m_shooter.isShooterAtSpeed(); 
    }
}
