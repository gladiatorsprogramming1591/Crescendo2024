package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
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

import java.util.EnumMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;


public class ArmSubsystem extends SubsystemBase {
    CANSparkMax m_rightArmMotor; 
    CANSparkMax m_leftArmMotor;
    RelativeEncoder m_rightEncoder; 
    RelativeEncoder m_leftEncoder; 
    double m_rightTargetVelocity = 0; 
    double m_leftTargetVelocity = 0; 

    private final DutyCycleEncoder armAbsEncoder = new DutyCycleEncoder(0);
    double m_speed = 0.0;
    EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);
    private final PIDController m_AbsPidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD); //i was 0.2 | p is set below
    
    public enum armPositions{
      TRANSFER, 
      SUBWOOFER, 
      PODIUM,
      CLIMBSTART,
      CLIMBFINISH,
      AMP,
      CURRENT
  }


    public ArmSubsystem(){
        m_rightArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmCANId, MotorType.kBrushless);
        m_leftArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmCANId, MotorType.kBrushless);
        m_rightEncoder = m_rightArmMotor.getEncoder();
        m_leftEncoder = m_leftArmMotor.getEncoder();
        m_rightArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftArmMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_leftArmMotor.follow(m_rightArmMotor, true);

        m_rightArmMotor.getPIDController().setP(ArmConstants.kArmP);
        m_rightArmMotor.getPIDController().setI(ArmConstants.kArmI);
        m_rightArmMotor.getPIDController().setD(ArmConstants.kArmD);
        m_rightArmMotor.getPIDController().setFF(ArmConstants.kArmFF);
        m_rightArmMotor.getPIDController().setReference(0, ControlType.kVoltage);

        m_rightArmMotor.enableVoltageCompensation(12);
        m_leftArmMotor.enableVoltageCompensation(12);

        m_rightArmMotor.setSmartCurrentLimit(20);
        m_leftArmMotor.setSmartCurrentLimit(20);

        mapAbs.put(armPositions.TRANSFER, ArmConstants.kTRANSFER);
        mapAbs.put(armPositions.SUBWOOFER, ArmConstants.kSUBWOOFER);
        mapAbs.put(armPositions.PODIUM, ArmConstants.kPODIUM);
        // mapAbs.put(armPositions.CLIMBSTART, ArmConstants.kCLIMBSTART);
        // mapAbs.put(armPositions.CLIMBFINISH, ArmConstants.kCLIMBFINISH); // Single Substation
        mapAbs.put(armPositions.AMP, ArmConstants.kAMP); //At hard stop:
    }

    public void ArmToPosition(armPositions position) {
        if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs) && (position == armPositions.TRANSFER)) ||
            ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs) && (position == armPositions.AMP))) {
            m_leftArmMotor.set(0);
            return;
        }

        // switch (position) {
        //     case SUBWOOFER:
        //     case CLIMBSTART:
        //     case CLIMBFINISH://TODO Finish probably needs its own p
        //         m_AbsPidController.setP(11.0);
        //         break;
        //     case TRANSFER:
        //     case PODIUM:
        //     default:
        //         m_AbsPidController.setP(9.0);
        //         break;
        // }
        double ref = mapAbs.get(position);

        double pidOut = MathUtil.clamp(
            m_AbsPidController.calculate(armAbsEncoder.getAbsolutePosition(),ref),
            Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
            
        SmartDashboard.putNumber("Arm Abs Target Pos", ref);
        SmartDashboard.putNumber("Arm Abs Speed", pidOut);
        m_rightArmMotor.set(pidOut);
    }


    public void ArmForward(double speed) {
        MathUtil.clamp(speed, 0, ArmConstants.kMaxOpenLoopSpeed); 
        //Turns on the Arm motor
        System.out.println("Turning Arm forward");
        // m_rightArmMotor.getPIDController().setReference(m_rightTargetVelocity, ControlType.kVoltage);
        // m_leftArmMotor.getPIDController().setReference(m_leftTargetVelocity, ControlType.kVoltage);
        m_rightArmMotor.set(-speed);
                  
        System.out.println("Setting Arm speed to " + -speed);
      }
    
      public void ArmBackward(double speed) {
        MathUtil.clamp(speed, 0, ArmConstants.kMaxOpenLoopSpeed); 
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

      public boolean atPosition(armPositions pos) {
        double currentEncoderPosition = armAbsEncoder.getAbsolutePosition();
        return (Math.abs(currentEncoderPosition - mapAbs.get(pos)) < Constants.ArmConstants.kAllowedErrAbs);
      }

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
          SmartDashboard.putString("Arm Pos", "" + armAbsEncoder.getAbsolutePosition());
          m_speed = m_rightArmMotor.getEncoder().getVelocity();

          if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs) && (m_speed < 0)) ||
            ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs) && (m_speed > 0))) {
            m_rightArmMotor.set(0);
            }
        }
      

}   

