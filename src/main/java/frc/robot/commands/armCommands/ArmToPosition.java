package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmToPosition extends Command {
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private double m_armSetpoint;
    private boolean m_keepRunning;
    private boolean m_customSetpoint = false;
    private int m_currentLimit = ArmConstants.kCurrentLimitDefault;

    public ArmToPosition(ArmSubsystem arm, ArmSubsystem.armPositions pos){
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_arm);
    }

    public ArmToPosition(ArmSubsystem arm, ArmSubsystem.armPositions pos, boolean keepRunning){
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = keepRunning;
        addRequirements(m_arm);
    }

    public ArmToPosition(ArmSubsystem arm, double armSetpoint){
        m_arm = arm;
        m_armSetpoint = armSetpoint;
        m_customSetpoint = true;
        addRequirements(m_arm);
    }

    public ArmToPosition(ArmSubsystem arm, ArmSubsystem.armPositions pos, int currentLimit) {
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = false;
        addRequirements(m_arm);
        m_currentLimit = currentLimit;
    }

    public void initialize(){
        m_arm.setArmCurrentLimit(m_currentLimit);
    }

    @Override
    public void execute(){
        m_arm.ArmToPosition(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        // return m_arm.atLevel(m_targetPos);
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        if (!m_keepRunning) m_arm.ArmOff();
    }
}
