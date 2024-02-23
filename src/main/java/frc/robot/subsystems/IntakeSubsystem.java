package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;


public class IntakeSubsystem extends SubsystemBase {
    CANSparkMax m_rightIntakeMotor; 
    CANSparkMax m_leftIntakeMotor; 
    // RelativeEncoder m_rightEncoder; 
    // RelativeEncoder m_leftEncoder; 
    // double m_rightTargetVelocity = Constants.ShooterConstants.kRightShooterSpeed; 
    // double m_leftTargetVelocity = Constants.ShooterConstants.kLeftShooterSpeed; 


    public IntakeSubsystem(){
        m_rightIntakeMotor = new CANSparkMax(Constants.IntakeConstants.kRightIntakeCANId, MotorType.kBrushless);
        m_leftIntakeMotor = new CANSparkMax(Constants.IntakeConstants.kLeftIntakeCANId, MotorType.kBrushless);
        // m_rightEncoder = m_rightShooterMotor.getEncoder();
        // m_leftEncoder = m_leftShooterMotor.getEncoder();
        m_rightIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_leftIntakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        m_leftIntakeMotor.follow(m_rightIntakeMotor, true);
        m_rightIntakeMotor.setSmartCurrentLimit(40);
        m_leftIntakeMotor.setSmartCurrentLimit(40);
        // m_rightShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        // m_rightShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        // m_rightShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        // m_rightShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);
        // m_rightShooterMotor.getPIDController().setReference(0, ControlType.kVoltage);

        // m_leftShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        // m_leftShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        // m_leftShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        // m_leftShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);
        // m_leftShooterMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_rightIntakeMotor.enableVoltageCompensation(12);
        m_leftIntakeMotor.enableVoltageCompensation(12);
    }


    public void intakeOn() {
        //Turns on the intake motor
        System.out.println("Turning intake on");
        m_rightIntakeMotor.set(Constants.IntakeConstants.kIntakeSpeed);
                  
        System.out.println("Setting intake speed to " + Constants.IntakeConstants.kIntakeSpeed);
      }

      public void intakeOff() {
        System.out.println("Turning intake off");
        m_rightIntakeMotor.set(0);
        m_leftIntakeMotor.set(0);
      }

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
        }
      

}   

