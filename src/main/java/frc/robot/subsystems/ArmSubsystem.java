package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;


public class ArmSubsystem extends SubsystemBase {
    CANSparkMax m_rightArmMotor; 
    CANSparkMax m_leftArmMotor;
    RelativeEncoder m_rightEncoder; 
    RelativeEncoder m_leftEncoder; 
    double m_rightTargetVelocity = 0; 
    double m_leftTargetVelocity = 0; 


    public ArmSubsystem(){
        m_rightArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmCANId, MotorType.kBrushless);
        m_leftArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmCANId, MotorType.kBrushless);
        m_rightEncoder = m_rightArmMotor.getEncoder();
        m_leftEncoder = m_leftArmMotor.getEncoder();
        m_rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftArmMotor.follow(m_rightArmMotor);

        m_rightArmMotor.getPIDController().setP(ArmConstants.kArmP);
        m_rightArmMotor.getPIDController().setI(ArmConstants.kArmI);
        m_rightArmMotor.getPIDController().setD(ArmConstants.kArmD);
        m_rightArmMotor.getPIDController().setFF(ArmConstants.kArmFF);
        m_rightArmMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_leftArmMotor.getPIDController().setP(ArmConstants.kArmP);
        m_leftArmMotor.getPIDController().setI(ArmConstants.kArmI);
        m_leftArmMotor.getPIDController().setD(ArmConstants.kArmD);
        m_leftArmMotor.getPIDController().setFF(ArmConstants.kArmFF);
        m_leftArmMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_rightArmMotor.enableVoltageCompensation(12);
        m_leftArmMotor.enableVoltageCompensation(12);
    }


    public void ArmForward(double speed) {
        //Turns on the Arm motor
        System.out.println("Turning Arm forward");
        // m_rightArmMotor.getPIDController().setReference(m_rightTargetVelocity, ControlType.kVoltage);
        // m_leftArmMotor.getPIDController().setReference(m_leftTargetVelocity, ControlType.kVoltage);
        m_rightArmMotor.set(speed);
                  
        System.out.println("Setting Arm speed to " + speed);
      }
    
        public void ArmBackward(double speed) {
        //Turns on the Arm motor
        System.out.println("Turning Arm backward");
        // m_rightArmMotor.getPIDController().setReference(m_rightTargetVelocity, ControlType.kVoltage);
        // m_leftArmMotor.getPIDController().setReference(m_leftTargetVelocity, ControlType.kVoltage);
        m_rightArmMotor.set(speed);
                  
        System.out.println("Setting Arm speed to " + speed);
      }

      public void ArmOff() {
        System.out.println("Turning Arm off");
        m_rightArmMotor.set(0);
      }

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
          SmartDashboard.putString("Arm Vel", "" + Math.round(m_leftEncoder.getVelocity()));
        }
      

}   

