package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpShooter extends Command {
    private ShooterSubsystem m_shooter; 
    boolean m_trap = false;

    public WarmUpShooter(ShooterSubsystem shooterSubsystem){
        m_shooter = shooterSubsystem; 
        addRequirements(m_shooter);
    }

    public WarmUpShooter(ShooterSubsystem shooterSubsystem, boolean trap) {
        m_shooter = shooterSubsystem;
        addRequirements(m_shooter);
        m_trap = trap;
    }

    @Override
    public void execute(){
        if (m_trap) {
            m_shooter.shooterOnTrap();
        } else {
            m_shooter.shooterOn();
        }
    }

    @Override
    public boolean isFinished(){
        return m_shooter.isShooterAtSpeed(); 
    }
}
