package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BlowerSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AlignAndTrapShoot extends SequentialCommandGroup {
    public AlignAndTrapShoot(DriveSubsystem driveSubsystem, ArmSubsystem armSubsystem, BlowerSubsystem blowerSubsystem, ShooterSubsystem shooterSubsystem){
        // Since we are using a holonomic drivetrain, the rotation component of this pose
        // represents the goal holonomic rotation
        Pose2d targetPose = new Pose2d(12.47, 5.43, Rotation2d.fromDegrees(-60));

        // Create the constraints to use while pathfinding
        PathConstraints constraints = new PathConstraints(
                3.0, 4.0,
                Units.degreesToRadians(540), Units.degreesToRadians(720));

        // Since AutoBuilder is configured, we can use it to build pathfinding commands
        Command pathfindingCommand = AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0, // Goal end velocity in meters/sec
                0.0 // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
        );

        addCommands(
            pathfindingCommand, 
            new TrapShootPrep(shooterSubsystem, armSubsystem, ArmSubsystem.armPositions.TRAP, blowerSubsystem)
        );
    }
}
