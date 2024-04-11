package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;

public class ArmToPositionWithEnd extends Command {
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private boolean m_wideTolerance = false; 

    public ArmToPositionWithEnd(ArmSubsystem arm, ArmSubsystem.armPositions pos){
        m_arm = arm;
        m_targetPos = pos;
        if(pos == ArmSubsystem.armPositions.TRANSFER){
            m_wideTolerance = true; 
        }
        addRequirements(m_arm);
    }

    public void initialize() {
        m_arm.setArmCurrentLimit(ArmConstants.kCurrentLimitDefault);
    }

    @Override
    public void execute(){
        m_arm.ArmToPosition(m_targetPos);
    }

    @Override
    public boolean isFinished(){
        SmartDashboard.putString("Arm at pos", "" + m_arm.atPosition(m_targetPos, m_wideTolerance));

        return m_arm.atPosition(m_targetPos, m_wideTolerance);   // Needs additional testing
    }

    @Override
    public void end(boolean isInterrupted){
        m_arm.ArmOff();
    }
}
