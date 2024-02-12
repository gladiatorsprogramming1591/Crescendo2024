package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class VisionDriveAligned extends Command {
    private DriveSubsystem m_subsystem;
    private double m_desiredY;
    private double m_desiredId;
    private double m_rotVisionSetpoint;
    private boolean m_isFinished = false;

    public VisionDriveAligned(double desiredId, double desiredY, double rotVisionSetpoint, DriveSubsystem subsystem){
        m_subsystem = subsystem; 
        m_desiredId = desiredId; 
        m_desiredY = desiredY; 
        m_rotVisionSetpoint = rotVisionSetpoint; 
    }

    @Override
    public void execute(){
        m_isFinished = m_subsystem.visionDriveAligned(m_desiredId, m_desiredY, m_rotVisionSetpoint);

    }

    @Override
    public boolean isFinished(){
        return m_isFinished; 
    }
}
