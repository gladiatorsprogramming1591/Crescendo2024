package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ArmSubsystem.armPositions;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeSourcePartOne extends Command {
    private ShooterSubsystem m_shooter;
    private ArmSubsystem m_arm;
    boolean m_hasNote = false;
    boolean m_pastBeamBreak = false;

    public IntakeSourcePartOne(ShooterSubsystem shooterSubsystem, ArmSubsystem armSubsystem) {
        m_shooter = shooterSubsystem;
        m_arm = armSubsystem;
        addRequirements(m_shooter, m_arm);

    }

    public void initialize() {
        m_hasNote = false;
        m_pastBeamBreak = false;
    }

    @Override
    public void execute() {
        m_arm.ArmToPosition(ArmConstants.kSUBWOOFER - 0.03);
        if (m_hasNote) {
            m_shooter.transferReverse(true);
            m_shooter.shooterIntake();
            m_pastBeamBreak = !m_shooter.isBeamBroken();
        } else {
            m_hasNote = m_shooter.isBeamBroken();
            m_shooter.transferReverse(false);
            m_shooter.shooterIntake();
        }

    }

    @Override
    public boolean isFinished() {
        System.out.println("Finished");
        return m_hasNote && m_pastBeamBreak;
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.shooterOff();
        m_arm.ArmOff();
        m_shooter.transferOff();
    }
}