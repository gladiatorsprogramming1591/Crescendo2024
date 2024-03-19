package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FindNoteAndIntake extends SequentialCommandGroup {

    public FindNoteAndIntake(ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem,
            DriveSubsystem driveSubsystem) {
        addRequirements(shooterSubsystem, armSubsystem, intakeSubsystem, driveSubsystem);
        addCommands(
            new FindNote(shooterSubsystem,armSubsystem,intakeSubsystem,driveSubsystem),
            new AlignAndIntake(shooterSubsystem, armSubsystem, intakeSubsystem, driveSubsystem)
        );
    }
}
