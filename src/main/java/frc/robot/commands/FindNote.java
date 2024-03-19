package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class FindNote extends Command {

    private static final double AMP_SIDE_NOTE_Y = 0.8;
    private static final double SOURCE_SIDE_NOTE_Y = 7.4;

    ArmSubsystem arm;
    IntakeSubsystem intake;
    DriveSubsystem drive;
    boolean noteFound;
    boolean doneSearching;
    boolean doneLookingRight;
    ProfiledPIDController turnController;
    double initialHeading;
    double rightTurnAngle;

    public FindNote(ShooterSubsystem shooterSubsystem,
            ArmSubsystem armSubsystem,
            IntakeSubsystem intakeSubsystem,
            DriveSubsystem driveSubsystem) {

        turnController = new ProfiledPIDController(
            DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kP,
            DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kI,
            DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kD,
            new TrapezoidProfile.Constraints(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared));

        addRequirements(shooterSubsystem, armSubsystem, intakeSubsystem, driveSubsystem);
        arm = armSubsystem;
        intake = intakeSubsystem;
        drive = driveSubsystem;

        // Set the controller to be continuous (because it is an angle controller)
        turnController.enableContinuousInput(-Math.PI, Math.PI);
        // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
        // setpoint before it is considered as having reached the reference
        turnController.setTolerance(DriveConstants.AUTO_AIM_ROT_TOLERANCE);
    }

    @Override
    public void initialize() {
        noteFound = false;
        doneSearching = false;
        doneLookingRight = false;
        initialHeading = drive.getHeading();

        final double THRESHOLD = 0.2;
        double y = drive.getPosition().getTranslation().getY();
        if (Math.abs(y-SOURCE_SIDE_NOTE_Y) < THRESHOLD) { // Near source note, look right 90 degrees
            rightTurnAngle = Math.PI/2;
        } else if (Math.abs(y-AMP_SIDE_NOTE_Y) < THRESHOLD) { // Near amp note, no right search
            rightTurnAngle = 0;
        } else { // Otherwise, look right 45 degrees
            rightTurnAngle = Math.PI/4;
        }
    }

    @Override
    public void execute() {
        // Check for note already in vision
        if (drive.hasNoteTarget()) {
            noteFound = true;
            drive.drive(0,0,0,false,false); // Remove after tested
        } else {
            // If rightTurnAngle == 0, we are near amp note, so done searching right
            if (rightTurnAngle == 0) {
                doneLookingRight = true;
            }

            if (!doneLookingRight) {
                turnController.setGoal(initialHeading + rightTurnAngle);                
            } else {
                turnController.setGoal(initialHeading - Math.PI/2);
            }
            drive.drive(0,0,turnController.calculate(drive.getHeading()),false,false);
        }
    }

    @Override
    public boolean isFinished() {
        return noteFound || turnController.atGoal();
    }
}
