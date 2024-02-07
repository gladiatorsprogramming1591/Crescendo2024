package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;


public class ShooterSubsystem extends SubsystemBase {
    CANSparkFlex m_rightShooterMotor; 
    CANSparkFlex m_leftShooterMotor; 
    RelativeEncoder m_rightEncoder; 
    RelativeEncoder m_leftEncoder; 
    double m_rightTargetVelocity; 
    double m_leftTargetVelocity; 


    public ShooterSubsystem(){
        m_rightShooterMotor = new CANSparkFlex(0, null);
        m_leftShooterMotor = new CANSparkFlex(0, null);
        m_rightEncoder = m_rightShooterMotor.getEncoder();
        m_leftEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);

        m_rightShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        m_rightShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        m_rightShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        m_rightShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);

        m_leftShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        m_leftShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        m_leftShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        m_leftShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);

        m_rightShooterMotor.enableVoltageCompensation(12);
        m_leftShooterMotor.enableVoltageCompensation(12);
    }


    public void shooterOn() {
        //Turns on the shooter motor
        System.out.println("Turning shooter on");
                  
        
          // m_rightShooterMotor.getPIDController().setFF(speed * 1 + 0);
          System.out.println("Setting shooter speed to " + Constants.kShooterMotorSpeed);
          // System.out.println("Setting shooter speed to " + speed);
          // m_rightShooterMotor.getPIDController().setReference(speed, ControlType.kVelocity);
          m_rightShooterMotor.set(Constants.kShooterMotorSpeed);
        }
    
      }

      public void shooterOff() {
        System.out.println("Turning shooter off");
        m_rightShooterMotor.set(0);
      }

      public boolean isShooterAtSpeed() {
        return ((m_leftEncoder.getVelocity() >= (m_leftTargetVelocity * 1) - 25)
          && (m_leftEncoder.getVelocity() <= (m_leftTargetVelocity * 1) + 25)
          && (m_rightEncoder.getVelocity() >= (m_rightTargetVelocity * 1) - 25)
          && (m_rightEncoder.getVelocity() <= (m_rightTargetVelocity * 1) + 25)); 
          
      } 

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
          SmartDashboard.putString("Shooter Vel", "" + Math.round(m_leftEncoder.getVelocity()));
        }
      

}   

