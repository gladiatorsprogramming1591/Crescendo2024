package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;


public class ShooterSubsystem extends SubsystemBase {
    CANSparkFlex m_rightShooterMotor; 
    CANSparkFlex m_leftShooterMotor; 
    CANSparkMax m_transferMotor;
    RelativeEncoder m_rightEncoder; 
    RelativeEncoder m_leftEncoder; 
    double m_rightTargetVelocity = Constants.ShooterConstants.kRightShooterSpeed; 
    double m_leftTargetVelocity = Constants.ShooterConstants.kLeftShooterSpeed; 
    DigitalInput m_ShooterBeamBreak = new DigitalInput(4); 


    public ShooterSubsystem(){
        m_rightShooterMotor = new CANSparkFlex(Constants.ShooterConstants.kRightShooterCANId, MotorType.kBrushless);
        m_leftShooterMotor = new CANSparkFlex(Constants.ShooterConstants.kLeftShooterCANId, MotorType.kBrushless);
        m_rightEncoder = m_rightShooterMotor.getEncoder();
        m_leftEncoder = m_leftShooterMotor.getEncoder();
        m_rightShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_leftShooterMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_rightShooterMotor.setInverted(false);
        m_leftShooterMotor.setInverted(true);

        m_rightShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        m_rightShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        m_rightShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        m_rightShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);
        m_rightShooterMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_leftShooterMotor.getPIDController().setP(ShooterConstants.kShooterP);
        m_leftShooterMotor.getPIDController().setI(ShooterConstants.kShooterI);
        m_leftShooterMotor.getPIDController().setD(ShooterConstants.kShooterD);
        m_leftShooterMotor.getPIDController().setFF(ShooterConstants.kShooterFF);
        m_leftShooterMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_rightShooterMotor.enableVoltageCompensation(12);
        m_leftShooterMotor.enableVoltageCompensation(12);

        m_transferMotor = new CANSparkMax(Constants.ShooterConstants.kTransferCANId, MotorType.kBrushless);
    }


    public void shooterOn() {
        //Turns on the shooter motor
        System.out.println("Turning shooter on");
        m_rightShooterMotor.getPIDController().setReference(m_rightTargetVelocity, ControlType.kDutyCycle);
        m_leftShooterMotor.getPIDController().setReference(m_leftTargetVelocity, ControlType.kDutyCycle);
                  
        System.out.println("Setting left shooter speed to " + m_leftTargetVelocity);
        System.out.println("Setting right shooter speed to " + m_rightTargetVelocity);
      }

      public void shooterOff() {
        System.out.println("Turning shooter off");
        m_rightShooterMotor.set(0);
        m_leftShooterMotor.set(0);
      }

      public boolean isShooterAtSpeed() {
        // return ((m_leftEncoder.getVelocity() >= (m_leftTargetVelocity * 1) - 25)
        //   && (m_leftEncoder.getVelocity() <= (m_leftTargetVelocity * 1) + 25)
        //   && (m_rightEncoder.getVelocity() >= (m_rightTargetVelocity * 1) - 25)
        //   && (m_rightEncoder.getVelocity() <= (m_rightTargetVelocity * 1) + 25)); 
        return false; //TODO Determine how to get atSpeed. May need to use Roborio PIDcontroller
          
      } 

      public void transferOn(boolean beamBreak){
        

        if(beamBreak){
          if(m_ShooterBeamBreak.get()){
            m_transferMotor.set(Constants.ShooterConstants.kTransferSpeed);
          } else {
            m_transferMotor.set(0);
          }
        } else {
          m_transferMotor.set(Constants.ShooterConstants.kTransferSpeedFull);
        }
      }

      public void transferOff(){
        m_transferMotor.set(0);
      }

      public void transferReverse(){
        m_transferMotor.set(-0.5);
      }

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
          SmartDashboard.putString("Shooter Vel", "" + Math.round(m_leftEncoder.getVelocity()));
          SmartDashboard.putBoolean("ShooterBeamBreak", m_ShooterBeamBreak.get()); 
        }
      

}   

