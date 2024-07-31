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
  private final PIDController m_AbsPidController = new PIDController(ArmConstants.kArmP, ArmConstants.kArmI,
      ArmConstants.kArmD); // i was 0.2 | p is set below

  public enum armPositions {
    TRANSFER,
    SUBWOOFER,
    PODIUM,
    CLIMBSTART,
    CLIMBFINISH,
    AMP,
    AMPFINISH,
    TRAP,
    STAGELINE,
    FOURTHNOTE

  }

  public ArmSubsystem() {
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

    m_rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitDefault);
    m_leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitDefault);

    mapAbs.put(armPositions.TRANSFER, ArmConstants.kTRANSFER);
    mapAbs.put(armPositions.SUBWOOFER, ArmConstants.kSUBWOOFER);
    mapAbs.put(armPositions.PODIUM, ArmConstants.kPODIUM);
    mapAbs.put(armPositions.TRAP, ArmConstants.kTRAP);
    mapAbs.put(armPositions.STAGELINE, ArmConstants.kSTAGELINE);
    mapAbs.put(armPositions.FOURTHNOTE, ArmConstants.kFOURTHNOTE);
    mapAbs.put(armPositions.CLIMBSTART, ArmConstants.kCLIMBSTART);
    mapAbs.put(armPositions.CLIMBFINISH, ArmConstants.kCLIMBFINISH); // Single Substation
    mapAbs.put(armPositions.AMP, ArmConstants.kAMP); // At hard stop:
    mapAbs.put(armPositions.AMPFINISH, ArmConstants.kAMPFINISH); // At hard stop:
    m_AbsPidController.setTolerance(ArmConstants.kPositionTolerance);
  }

  public void ArmToPosition(double setpoint, int currentLimit) {
    m_rightArmMotor.setSmartCurrentLimit(currentLimit);
    m_leftArmMotor.setSmartCurrentLimit(currentLimit);

    if (((armAbsEncoder.getAbsolutePosition() > ArmConstants.kMinHeightAbs)
        && (setpoint > mapAbs.get(armPositions.TRANSFER))) ||
        ((armAbsEncoder.getAbsolutePosition() < ArmConstants.kMaxHeightAbs)
            && (setpoint < mapAbs.get(armPositions.AMPFINISH)))) {
      m_leftArmMotor.set(0);
      return;
    }
    if (setpoint == ArmConstants.kCLIMBFINISH) {
      m_AbsPidController.setP(ArmConstants.kArmP * 5);
    } else {
      m_AbsPidController.setP(ArmConstants.kArmP);
    }
    double pidOut = MathUtil.clamp(
        m_AbsPidController.calculate(armAbsEncoder.getAbsolutePosition(), setpoint),
        Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);

    SmartDashboard.putNumber("Arm Position Error", m_AbsPidController.getPositionError());
    SmartDashboard.putNumber("Arm Abs Target Pos", setpoint);
    SmartDashboard.putNumber("Arm Abs Speed", pidOut);

    if (setpoint >= ArmConstants.kOffset) {
      m_rightArmMotor.set(0);
      return;
    }
    m_rightArmMotor.set(pidOut);
  }

  public void ArmToPosition(armPositions position) {
    double ref = mapAbs.get(position);
    ArmToPosition(ref, ArmConstants.kCurrentLimitDefault);
  }

  public void ArmToPosition(armPositions position, int currentLimit) {
    double ref = mapAbs.get(position);
    ArmToPosition(ref, currentLimit);
  }

  public void ArmToPosition(double position) {
    ArmToPosition(position, ArmConstants.kCurrentLimitDefault);
  }

  public void ArmForward(double speed) {
    m_rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitManual);
    m_leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitManual);

    MathUtil.clamp(speed, 0, ArmConstants.kMaxOpenLoopSpeed);
    // Turns on the Arm motor
    System.out.println("Turning Arm forward");
    // m_rightArmMotor.getPIDController().setReference(m_rightTargetVelocity,
    // ControlType.kVoltage);
    // m_leftArmMotor.getPIDController().setReference(m_leftTargetVelocity,
    // ControlType.kVoltage);
    m_rightArmMotor.set(-speed);

    System.out.println("Setting Arm speed to " + -speed);
  }

  public void ArmBackward(double speed) {
    m_rightArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitManual);
    m_leftArmMotor.setSmartCurrentLimit(ArmConstants.kCurrentLimitManual);

    MathUtil.clamp(speed, 0, ArmConstants.kMaxOpenLoopSpeed);
    // Turns on the Arm motor
    System.out.println("Turning Arm backward");
    // m_rightArmMotor.getPIDController().setReference(m_rightTargetVelocity,
    // ControlType.kVoltage);
    // m_leftArmMotor.getPIDController().setReference(m_leftTargetVelocity,
    // ControlType.kVoltage);
    m_rightArmMotor.set(speed);

    System.out.println("Setting Arm speed to " + speed);
  }

  public void ArmOff() {
    System.out.println("Turning Arm off");
    m_rightArmMotor.set(0);
  }

  public boolean atPosition(armPositions pos, boolean wideTolerance) {
    double currentEncoderPosition = armAbsEncoder.getAbsolutePosition();
    double tolerance = Constants.ArmConstants.kAllowedErrAbs; 
    if (wideTolerance){
      tolerance = Constants.ArmConstants.kAllowedErrWideToleranceAbs; 
    }
    return (Math.abs(currentEncoderPosition - mapAbs.get(pos)) < tolerance);
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
