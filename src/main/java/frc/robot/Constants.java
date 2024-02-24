// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxModuleMetersPerSecond = 5.66; // TODO Increase speed when done testing autos at low speed or add speed toggle button
    public static final double kMaxSpeedMetersPerSecond = kMaxModuleMetersPerSecond; // TODO Increase speed when done testing autos at low speed or add speed toggle button
    public static final double kMaxAngularSpeed =  2 * Math.PI; // radians per second
    public static final double kTeleopPercentLimit = .95; 

    public static final Vector<N3> odometryStd = VecBuilder.fill(0.06, 0.06, 0.01);
    public static final Vector<N3> visionStd = VecBuilder.fill(0.35, 0.35, 0.4);

    // Camera Positions
    public static final Transform3d kFrontCameraLocation = new Transform3d(
        new Translation3d(Units.inchesToMeters(10.507), Units.inchesToMeters(5.673),
            Units.inchesToMeters(6.789)),
        new Rotation3d(0.0, Math.toRadians(-20.0), Math.toRadians(0.0)));

    // Field Positions

    public static final double FIELD_LENGTH = 16.5417;
    public static final double FIELD_WIDTH = 8.0136;

    public static final double NOTE_VELOCITY = 10.0;

    public static final Translation2d BLUE_SPEAKER = new Translation2d(0.0241, 5.547868);
    public static final Translation2d RED_SPEAKER = new Translation2d(FIELD_LENGTH - BLUE_SPEAKER.getX(), BLUE_SPEAKER.getY());
    public static final Translation2d STAGE = new Translation2d(4.981067, 4.105783);

    public static final double SPEAKER_HEIGHT = 2.08;
    public static final Pose3d BLUE_SPEAKER_3D = new Pose3d(BLUE_SPEAKER.getX(), BLUE_SPEAKER.getY(), SPEAKER_HEIGHT, new Rotation3d());
    public static final Pose3d RED_SPEAKER_3D = new Pose3d(RED_SPEAKER.getX(), RED_SPEAKER.getY(), SPEAKER_HEIGHT, new Rotation3d());

    public static final double OPPONENT_WING_LINE = 10.66;
    public static final double AMP_X = 1.9;


    public static final PIDConstants AUTO_AIM_ROT_PID_CONSTANTS = new PIDConstants(9.0, 0.02, 1.0);

    public static final double VISION_REJECT_DISTANCE = 2.3;

    public static final double SPIN_COMPENSATION_X = 0.0075;
    public static final double SPIN_COMPENSATION_Y = 0.06;

    // Radians
    public static final double AUTO_AIM_ROT_TOLERANCE = Math.toRadians(1.2);

    // Shooter Angle Map

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    public static final double MAX_ROTATION_SPEED_AUTO_AIM = 5.0;
    public static final double TRANSLATION_SPEED_SCALAR_AUTO_AIM = 0.5;

    static {
      DISTANCE_TO_ANGLE_MAP.put(1.25, ArmConstants.kSUBWOOFER);
      DISTANCE_TO_ANGLE_MAP.put(2.2, ArmConstants.kOffset - 0.075);
      DISTANCE_TO_ANGLE_MAP.put(3.0, ArmConstants.kOffset - 0.058);
      DISTANCE_TO_ANGLE_MAP.put(4.1, ArmConstants.kOffset - 0.037);
      DISTANCE_TO_ANGLE_MAP.put(4.9, ArmConstants.kOffset - 0.0295);
      DISTANCE_TO_ANGLE_MAP.put(5.5, ArmConstants.kOffset - 0.028);
    }


    public static final double kDirectionSlewRate = 2.4; // radians per second
    public static final double kMagnitudeSlewRate = 3.6; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 4.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 11;
    public static final int kRearLeftDrivingCanId = 13;
    public static final int kFrontRightDrivingCanId = 15;
    public static final int kRearRightDrivingCanId = 17;

    public static final int kFrontLeftTurningCanId = 10;
    public static final int kRearLeftTurningCanId = 12;
    public static final int kFrontRightTurningCanId = 14;
    public static final int kRearRightTurningCanId = 16;

    public static final boolean kGyroReversed = false;

    public static final int kPigeonCanId = 9;

    public static final double maxVisionRotSpeed = 0.4;
    public static final double maxVisionStrafeSpeed = 0.4;
    public static final double kMinRotSpeed = 0.1;
    public static final double kMinXSpeed = 0.1; 
    public static final double kMinYSpeed = 0.1; 
  }

  public static final class ModuleConstants{
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 20 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 35; // TODO Increase for competition
    public static final int kTurningMotorCurrentLimit = 15; // amps

    public static final double kExampleTurningP = 0.01; 
    public static final double kExampleTurningI = 0; 
    public static final double kExampleTurningD = 0;
  }
  //Initial values taken from Roberta
  public static final class ShooterConstants{
    public static final double kShooterP = 0.0003;
    public static final double kShooterI = 0.000000;
    public static final double kShooterD = 0.0000;
    public static final double kShooterFF = 0.00015;

    
    public static final int kLeftShooterCANId = 5;
    public static final int kRightShooterCANId = 6;
    public static final int kTransferCANId = 7;

    // public static final double kRightShooterSpeed = 0.55; 
    // public static final double kLeftShooterSpeed = 1.0; 
    public static final double kTransferSpeed = 0.25;
    public static final double kTransferSpeedFull = 1.0;
    public static final double kRightShooterSpeedRPM = 5000.0;
    public static final double kLeftShooterSpeedRPM = 0.6*kRightShooterSpeedRPM; 
    public static final double kRightShooterTrapSpeedRPM = 2000.0;
    public static final double kLeftShooterTrapSpeedRPM = 0.6*kRightShooterTrapSpeedRPM;
    public static final double kMinShooterSpeed = 4800.0; 
    public static final double kRightShooterFarSpeed = 5250.0; 
    public static final double kLeftShooterFarSpeed = 0.6 *kRightShooterFarSpeed; 
    public static final double kShooterRPMTolerance = 200.0;
    public static final double kSVolts = 0.05;
    public static final double kVVoltSecondsPerRotation =
        // Should have value 12V at free speed...
        12.0 / NeoMotorConstants.kFreeSpeedRpm;

    //Current limits
    public static final int kShooterStallLimit = 80; 
    public static final int kShooterFreeLimit = 40; 

  }

  public static final class IntakeConstants{
    public static final int kLeftIntakeCANId = 3;
    public static final int kRightIntakeCANId = 4; 

    public static final int kCurrentLimit = 30;

    public static final double kIntakeSpeed = -0.95;
    public static final double kIntakeReverseSpeed = 0.75;
  }

  public static final class ArmConstants{
    public static final int kLeftArmCANId = 8;
    public static final int kRightArmCANId = 2;

    public static final double kArmP = 15.0; //6.4
    public static final double kArmI = 0.00;
    public static final double kArmD = 0.00;
    public static final double kArmFF = 0.00;
    //TODO Update min and max heights with measured values

    public static final double kMaxOpenLoopSpeed = 0.5; 
    
    //TODO make source position to intake note from there. 
    // Arm Positions
    public static final double kOffset = 0.66;
    public static final double kTRANSFER = kOffset - 0.011;
    public static final double kSUBWOOFER = kOffset - 0.12;
    public static final double kPODIUM = kOffset - 0.055;
    public static final double kTRAP = kOffset - 0.3;
    public static final double kSTAGELINE = kOffset - 0.021;
    public static final double kFOURTHNOTE = kOffset - 0.05; 
    public static final double kCLIMBSTART = kTRAP;
    public static final double kCLIMBFINISH = kPODIUM;
    public static final double kAMP = kOffset - 0.375;
    public static final double kAllowedErrAbs = 0.001;
    public static final double kMinHeightAbs = kOffset;
    public static final double kMaxHeightAbs = 0.2;
    //TODO Update arm positions with measured values

    // Arm Speed
    public static final double kArmMinOutput = -1.0; //-1.00
    public static final double kArmMaxOutput = 1.0; //1.00
    public static final double kPositionTolerance = 0.05; 
    public static final double kVelocityTolerance = 0.02; 

    // Arm Current Limits
    public static final int kCurrentLimitDefault = 20;
    public static final int kCurrentLimitManual = 10;
    public static final int kCurrentLimitClimbing = 40;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
    public static final double kArmDeadband = 0.1;
    public static final int kOperatorControllerPort = 1;
    public static final double kOperatorDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VortexMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }
}
