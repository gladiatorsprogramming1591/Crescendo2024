// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class WarmUpAndAutoShoot extends Command {

  int transferCount;
  int onTargetCount;
  /**
   * Creates a new RunTransferIfOnTarget.
   * Finishes once the shooter has released the note after running the transfer
   * 
   * @param driveSubsystem
   * @param shooterSubsystem
   */
  DriveSubsystem m_drive;
  ShooterSubsystem m_shooter;
  boolean m_end = false;

  public WarmUpAndAutoShoot(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
    m_drive = driveSubsystem;
    m_shooter = shooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    onTargetCount = 0;
    transferCount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooter.shooterOn();
    if (m_drive.getIsOnTargetSpeaker() && m_shooter.isShooterAtSpeed()) {
      onTargetCount++;
      if (onTargetCount > 10) { // 10 * 20 ms = 200 ms of being on target & at speed
        m_shooter.transferOn(false);
        transferCount++;
        if (transferCount > 10) { // 1010 - 1000 = 10 * 20 ms = 200 ms to run transfer before finishing command
          m_end = true;
        }
      }
    } else {
      onTargetCount = 0;
      transferCount = 0;
      m_shooter.transferOff();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.shooterOff();
    m_shooter.transferOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_end;
  }
}
