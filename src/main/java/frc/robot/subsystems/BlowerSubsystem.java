package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class BlowerSubsystem extends SubsystemBase {
  private TalonSRX blowerMotor;

  public BlowerSubsystem() {
    blowerMotor = new TalonSRX(Constants.BlowerConstants.blowerCANID);
    blowerMotor.setNeutralMode(NeutralMode.Brake);
    //saturation volate?
    blowerMotor.configVoltageCompSaturation(12.0);
    blowerMotor.enableVoltageCompensation(true);
  }

  public void blowerOn() {
    // Turns on the blower motor
    System.out.println("Turning blower on full");
    blowerMotor.set(TalonSRXControlMode.PercentOutput, 1.0);
  }

  public void blowerOff() {
    // Turns on the blower motor
    System.out.println("Turning blower on full");
    blowerMotor.set(TalonSRXControlMode.PercentOutput, 1.0);
  }
}
