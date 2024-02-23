package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.NeoMotorConstants;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkBase;


public class ShooterSubsystem extends SubsystemBase {
    private CANSparkFlex m_rightShooterMotor; 
    private CANSparkFlex m_leftShooterMotor; 
    private CANSparkMax m_transferMotor;
    private RelativeEncoder m_rightEncoder; 
    private RelativeEncoder m_leftEncoder; 
    private final double m_rightTargetVelocity = Constants.ShooterConstants.kRightShooterSpeedRPM; 
    private final double m_leftTargetVelocity = Constants.ShooterConstants.kLeftShooterSpeedRPM; 
    private double m_leftSetpoint = Constants.ShooterConstants.kLeftShooterSpeedRPM; 
    private final DigitalInput m_ShooterBeamBreak = new DigitalInput(4); 
    // private final PIDController m_leftPidController = new PIDController(
    //     ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);
    // private final PIDController m_rightPidController = new PIDController(
    //     ShooterConstants.kShooterP, ShooterConstants.kShooterI, ShooterConstants.kShooterD);
    // private final SimpleMotorFeedforward m_leftShooterFeedforward =new SimpleMotorFeedforward(
    //       ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);
    // private final SimpleMotorFeedforward m_rightShooterFeedforward =new SimpleMotorFeedforward(
    //       ShooterConstants.kSVolts, ShooterConstants.kVVoltSecondsPerRotation);

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
        m_transferMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void shooterOnTrap() {
        //Turns on the shooter motors
        System.out.println("Setting left shooter speed to " + ShooterConstants.kLeftShooterTrapSpeedRPM);
        System.out.println("Setting right shooter speed to " + ShooterConstants.kRightShooterTrapSpeedRPM);

        m_rightShooterMotor.getPIDController().setReference(ShooterConstants.kRightShooterTrapSpeedRPM, ControlType.kVelocity);
        m_leftShooterMotor.getPIDController().setReference(ShooterConstants.kLeftShooterTrapSpeedRPM, ControlType.kVelocity);
    }

    public void shooterOn() {
        //Turns on the shooter motors
        System.out.println("Setting left shooter speed to " + m_leftTargetVelocity);
        System.out.println("Setting right shooter speed to " + m_rightTargetVelocity);

        m_leftSetpoint = m_leftTargetVelocity; 
        m_rightShooterMotor.getPIDController().setReference(m_rightTargetVelocity, ControlType.kVelocity);
        m_leftShooterMotor.getPIDController().setReference(m_leftTargetVelocity, ControlType.kVelocity);

        // double leftPidOut = m_leftPidController.calculate(m_leftEncoder.getVelocity()/NeoMotorConstants.kFreeSpeedRpm,m_leftTargetVelocity);
        // double rightPidOut = m_rightPidController.calculate(m_rightEncoder.getVelocity()/NeoMotorConstants.kFreeSpeedRpm,m_rightTargetVelocity);

        // double leftFF = m_leftShooterFeedforward.calculate(m_leftTargetVelocity);
        // double rightFF = m_rightShooterFeedforward.calculate(m_rightTargetVelocity);

        // SmartDashboard.putNumber("Left Shooter PidOut", leftPidOut);
        // SmartDashboard.putNumber("Right Shooter PidOut", rightPidOut);
        // SmartDashboard.putNumber("Left Shooter FF", leftFF);
        // SmartDashboard.putNumber("Right Shooter FF", rightFF);
        // m_leftShooterMotor.setVoltage(leftFF);
        // m_rightShooterMotor.setVoltage(rightFF);             
      }

      public void shooterOn(boolean farShot) {
        if (farShot) {
          m_leftSetpoint = ShooterConstants.kLeftShooterFarSpeed; 
          m_rightShooterMotor.getPIDController().setReference(ShooterConstants.kRightShooterFarSpeed,
              ControlType.kVelocity);
          m_leftShooterMotor.getPIDController().setReference(ShooterConstants.kLeftShooterFarSpeed,
              ControlType.kVelocity);
        }
        else{
          shooterOn();
        }
      }
      public void shooterOff() {
        System.out.println("Turning shooter off");
        m_rightShooterMotor.set(0);
        m_leftShooterMotor.set(0);
      }

      public boolean isShooterAtSpeed() {
        return m_leftEncoder.getVelocity() > m_leftSetpoint - ShooterConstants.kShooterRPMTolerance; 
      } 
      public boolean isBeamBroken(){
        return m_ShooterBeamBreak.get() == false; 
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
        m_transferMotor.set(-0.35);
      }

      @Override
        public void periodic() {
          // This method will be called once per scheduler run
          // SmartDashboard.putString("Shooter Vel", "" + Math.round(m_leftEncoder.getVelocity()));
          SmartDashboard.putString("L Shooter Vel", "" + m_leftEncoder.getVelocity());
          SmartDashboard.putString("R Shooter Vel", "" + m_rightEncoder.getVelocity());
          SmartDashboard.putBoolean("ShooterBeamBreak", m_ShooterBeamBreak.get()); 
          SmartDashboard.putBoolean("ShooterAtSpeed", isShooterAtSpeed()); 
        }


      public Subsystem withTimeout(int i) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'withTimeout'");
      }
      

}   

