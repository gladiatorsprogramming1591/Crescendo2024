// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.utils.CommandBuilder;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

// import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase {
  public static final double kTurnRateToleranceDegPerS = 5.0;

  public static final double kTurnToleranceDeg = 1.0;

  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final Pigeon2 m_gyro = new Pigeon2(Constants.DriveConstants.kPigeonCanId);

  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;

  // AprilTag readings from network table
  private double x = 0;
  private double y = 0;
  private double a = 0;

  private final Field2d m_field = new Field2d();
  private final Field2d m_poseEstimatorField = new Field2d();
  private final Field2d m_goalPoseField = new Field2d();
  // private final Trajectory m_trajectory;

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  private double id;

  private double prev_x;
  private double prev_y;
  private double prev_id;
  private double prev_a;

  private int invalidCount = 9999; // initialized to value that represents invalid
  private int maxinvalidCount = 0;

  private final PIDController m_rotVisionPidController = new PIDController(0.020, 0.0, 0.002);
  private final PIDController m_yVisionPidController = new PIDController(0.033, 0.0, 0.005);
  private final PIDController m_xVisionPidController = new PIDController(0.033, 0.0, 0.005);

  // Auto-Aim Vision PID Controller
  private final PIDController m_autoAimRotationPidController = new PIDController(
      DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kP, DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kI,
      DriveConstants.AUTO_AIM_ROT_PID_CONSTANTS.kD);

  PhotonPoseEstimator[] m_photonPoseEstimators;
  SwerveDrivePoseEstimator m_poseEstimator;
  PhotonCamera m_noteCamera;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    final double kWheelBase = 22.5;
    double radius = Units.inchesToMeters((kWheelBase / 2) * Math.sqrt(2));

    m_rotVisionPidController.enableContinuousInput(-180, 180);
    m_rotVisionPidController.setTolerance(0.5);
    m_yVisionPidController.setTolerance(0.5);
    m_xVisionPidController.setTolerance(0.5);

    m_autoAimRotationPidController.enableContinuousInput(-Math.PI, Math.PI);
    m_autoAimRotationPidController.setTolerance(DriveConstants.AUTO_AIM_ROT_TOLERANCE);

    m_noteCamera = new PhotonCamera("Note");

    // Configure AutoBuilder last

    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.5, 0.01, 0.1), // Translation PID constants
            new PIDConstants(1.5, 0.0, 0.0), // Rotation PID constants
            DriveConstants.kMaxModuleMetersPerSecond, // Max module speed, in m/s
            radius, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        // TODO edit constants above
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    // Create the trajectory to follow in autonomous. It is best to initialize
    // trajectories here to avoid wasting time in autonomous.
    // m_trajectory =
    // TrajectoryGenerator.generateTrajectory(
    // new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
    // List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // new Pose2d(3, 0, Rotation2d.fromDegrees(0)),
    // new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));

    // Do this in either robot or subsystem init
    SmartDashboard.putData("Raw Odometry Field", m_field);
    SmartDashboard.putData("Pose Estimator Field", m_poseEstimatorField);
    SmartDashboard.putData("Goal Pose Field", m_goalPoseField);
    // Push the trajectory to Field2d.
    // m_field.getObject("traj").setTrajectory(m_trajectory);

    // Do this in either robot periodic or subsystem periodic

    // Vision Initialization
    if (!DriverStation.isAutonomousEnabled()) {
      m_poseEstimator = new SwerveDrivePoseEstimator(
          DriveConstants.kDriveKinematics,
          Rotation2d.fromDegrees(getHeading()),
          new SwerveModulePosition[] {
              m_frontLeft.getPosition(),
              m_frontRight.getPosition(),
              m_rearLeft.getPosition(),
              m_rearRight.getPosition()
          },
          new Pose2d(),
          DriveConstants.odometryStd,
          DriveConstants.visionStd);

      m_photonPoseEstimators = new PhotonPoseEstimator[] {
          new PhotonPoseEstimator(
              AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
              PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
              new PhotonCamera("Front"),
              DriveConstants.kFrontCameraLocation),
      };
    }
    // Future cameras
    // new PhotonPoseEstimator(
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // new PhotonCamera("Left"),
    // new Transform3d(
    // new Translation3d(-0.0, -0.0, 0.0),
    // new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(170.0))
    // )
    // ),
    // new PhotonPoseEstimator(
    // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(),
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
    // new PhotonCamera("Right"),
    // new Transform3d(
    // new Translation3d(-0.0, -0.0, 0.0),
    // new Rotation3d(0.0, Math.toRadians(-30.0), Math.toRadians(170.0))
    // )
    // ),
  }

  public boolean visionDriveAligned(double desiredId, double desiredY, double rotVisionSetpoint) {
    boolean retValue = true; // true = at desired location
    updateAprilTagInfo(desiredId);
    double rotSpeed = MathUtil.clamp(m_rotVisionPidController.calculate(getHeadingMod180(), rotVisionSetpoint),
        -DriveConstants.maxVisionRotSpeed, DriveConstants.maxVisionRotSpeed);
    double ySpeed = MathUtil.clamp(m_yVisionPidController.calculate(x, 0),
        -DriveConstants.maxVisionStrafeSpeed, DriveConstants.maxVisionStrafeSpeed);
    double xSpeed = MathUtil.clamp(m_xVisionPidController.calculate(-y, desiredY),
        -DriveConstants.maxVisionStrafeSpeed, DriveConstants.maxVisionStrafeSpeed);
    // if (m_yVisionPidController.atSetpoint()){
    if (m_rotVisionPidController.atSetpoint() && m_xVisionPidController.atSetpoint()
        && m_yVisionPidController.atSetpoint()) {
      drive(0, 0, 0, false, true);
    } else {
      drive(xSpeed, ySpeed, rotSpeed, false, true);
      retValue = false;
    }
    return retValue;
  }

  @Override
  public void periodic() {
    double gyroAngle = getHeading();
    double gyroYaw = m_gyro.getYaw().getValueAsDouble();

    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(gyroAngle),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    m_field.setRobotPose(m_odometry.getPoseMeters());
    m_poseEstimatorField.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("gyroAngle:", gyroAngle);
    SmartDashboard.putNumber("gyroYaw:", gyroYaw);
    SmartDashboard.putNumber("Odometry.x:", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Odometry.y:", m_odometry.getPoseMeters().getY());

    // updateAprilTagInfo(5.0);

    for (PhotonPoseEstimator photonPoseEstimator : m_photonPoseEstimators) {
      Optional<EstimatedRobotPose> pose = photonPoseEstimator.update();
      if (pose.isPresent())
        m_poseEstimator.addVisionMeasurement(pose.get().estimatedPose.toPose2d(), pose.get().timestampSeconds);
    }
    m_poseEstimator.update(Rotation2d.fromDegrees(getHeading()), new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    });

    SmartDashboard.putNumber("Distance to Speaker", getSpeakerDistance());
    SmartDashboard.putNumber("Note Angle", getNoteAngle());
    SmartDashboard.putNumber("Note Height", getNoteHeight());

    double driveCurrent = m_frontLeft.getDriveCurrent() + m_frontRight.getDriveCurrent() + m_rearLeft.getDriveCurrent()
        + m_rearRight.getDriveCurrent();
    SmartDashboard.putNumber("Drive Motors Current", driveCurrent);

    double steerCurrent = m_frontLeft.getSteerCurrent() + m_frontRight.getSteerCurrent() + m_rearLeft.getSteerCurrent()
        + m_rearRight.getSteerCurrent();
    SmartDashboard.putNumber("Steer Motors Current", steerCurrent);
  }

  public void updateAprilTagInfo(double desiredId) {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tid = table.getEntry("tid");

    // read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    a = ta.getDouble(0.0);
    id = tid.getDouble(0.0);

    if (id == desiredId) {
      prev_x = x;
      prev_y = y;
      prev_a = a;
      prev_id = id;
      invalidCount = 0;
    } else {
      invalidCount++;
      if (invalidCount != 9999 && invalidCount > maxinvalidCount) {
        maxinvalidCount = invalidCount;
      }
    }

    SmartDashboard.putNumber("AprilTag x", x);
    SmartDashboard.putNumber("AprilTag y", y);
    SmartDashboard.putNumber("AprilTag a", a);
    SmartDashboard.putNumber("AprilTag Id", id);
    SmartDashboard.putNumber("Invalid Count", invalidCount);
    SmartDashboard.putNumber("Max Invalid Count", maxinvalidCount);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param initialpose
   */
  public void resetOdometry(Pose2d initialpose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        }, initialpose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {

    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; // some high number that means the slew rate is effectively instantaneous
      }

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45 * Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      } else if (angleDif > 0.85 * Math.PI) {
        if (m_currentTranslationMag > 1e-4) { // some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        } else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      } else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir,
            directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;

      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);

    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    SmartDashboard.putNumber("XSpeed", xSpeedDelivered);
    SmartDashboard.putNumber("YSpeed", ySpeedDelivered);
    SmartDashboard.putNumber("RotSpeed", rotDelivered);
    SmartDashboard.putNumber("Rot", m_currentRotation);

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(getHeading()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = m_frontLeft.getState();
    states[1] = m_frontRight.getState();
    states[2] = m_rearLeft.getState();
    states[3] = m_rearRight.getState();

    return states;
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getAngle();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeadingMod180() {
    return MathUtil.inputModulus(-m_gyro.getAngle(), -180, 180);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public ChassisSpeeds getSpeeds() {
    return Constants.DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public ChassisSpeeds getFieldRelativeSpeeds(boolean useIMU) {
    Rotation2d yaw = useIMU ? Rotation2d.fromDegrees(getHeading()) : getPosition().getRotation();
    return ChassisSpeeds.fromFieldRelativeSpeeds(DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates()),
        yaw.unaryMinus());
  }

  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    SwerveModuleState[] targetStates = Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Gets the robot's position.
   */
  protected Pose2d getPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Drives the robot using percents of its calculated max velocity while locked
   * at a field relative angle.
   * 
   * @param x          The desired {@code x} speed from {@code -1.0} to
   *                   {@code 1.0}.
   * @param y          The desired {@code y} speed from {@code -1.0} to
   *                   {@code 1.0}.
   * @param angle      The desired field relative angle to point at in radians.
   * @param controller A profiled PID controller to use for translating to and
   *                   maintaining the angle.
   * @param useIMU     If the IMU should be used for determining the robot's
   *                   angle. If {@code false}, the pose estimator is used.
   */
  protected void driveAngle(double x, double y, double angle, PIDController controller, boolean useIMU) {
    driveAngleVelocity(x * DriveConstants.kMaxModuleMetersPerSecond, y * DriveConstants.kMaxModuleMetersPerSecond,
        angle, controller, useIMU);
  }

  /**
   * Drives the robot using velocity while locked at a field relative angle.
   * 
   * @param xV         The desired {@code x} velocity in meters/second.
   * @param yV         The desired {@code y} velocity in meters/second.
   * @param angle      The desired field relative angle to point at in radians.
   * @param controller A profiled PID controller to use for translating to and
   *                   maintaining the angle.
   * @param useIMU     If the IMU should be used for determining the robot's
   *                   angle. If {@code false}, the pose estimator is used.
   */
  protected void driveAngleVelocity(double xV, double yV, double angle, PIDController controller, boolean useIMU) {
    Rotation2d yaw = useIMU ? Rotation2d.fromDegrees(getHeading()) : getPosition().getRotation();
    SmartDashboard.putNumber("Gyro Yaw Rad", yaw.getRadians());
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xV * DriveConstants.TRANSLATION_SPEED_SCALAR_AUTO_AIM,
        yV * DriveConstants.TRANSLATION_SPEED_SCALAR_AUTO_AIM,
        MathUtil.clamp(controller.calculate(MathUtil.angleModulus(yaw.getRadians()), angle),
            -DriveConstants.MAX_ROTATION_SPEED_AUTO_AIM, DriveConstants.MAX_ROTATION_SPEED_AUTO_AIM),
        Rotation2d.fromDegrees(getHeading()));

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(swerveModuleStates);
  }

  /**
   * Gets a field-relative position for the shot to the speaker the robot should
   * take.
   * 
   * @return A {@link Translation2d} representing a field relative position in
   *         meters.
   */
  public Translation2d getSpeakerPosition() {
    boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Blue);
    Translation2d goalPose = isBlue ? DriveConstants.BLUE_SPEAKER : DriveConstants.RED_SPEAKER;
    ChassisSpeeds robotVel = getFieldRelativeSpeeds(false);
    double distanceToSpeaker = getPosition().getTranslation().getDistance(goalPose);
    double directionFlip = isBlue ? 1.0 : -1.0;
    double x = goalPose.getX()
        - (directionFlip * robotVel.vxMetersPerSecond * (distanceToSpeaker / DriveConstants.NOTE_VELOCITY));
    double y = goalPose.getY() - (robotVel.vyMetersPerSecond * (distanceToSpeaker / DriveConstants.NOTE_VELOCITY));
    Translation2d goalPoseAdjusted = new Translation2d(x, y);
    Pose2d speaker = new Pose2d(goalPoseAdjusted, new Rotation2d());
    m_goalPoseField.setRobotPose(speaker);
    return goalPoseAdjusted;
  }

  /**
   * Gets the angle for the robot to face to score in the speaker, in radians.
   */
  private double getSpeakerAngle() {
    Translation2d speakerPosition = getSpeakerPosition();
    double speakerDistance = getSpeakerDistance();
    Translation2d robotPoint = getPosition().getTranslation();
    Rotation2d shotRot = speakerPosition.minus(robotPoint).getAngle();

    // Translation2d targetPosition = new Translation2d(
    // speakerPosition.getX() + shotRot.getSin() * speakerDistance,
    // speakerPosition.getY() + shotRot.getCos() * speakerDistance);

    // Pose3d speakerRotTarget = new Pose3d(targetPosition.getX(),
    // targetPosition.getY(), DriveConstants.SPEAKER_HEIGHT, new Rotation3d());
    // SmartDashboard.putNumber("Angle To Speaker",
    // targetPosition.minus(robotPoint).getAngle().getRadians());
    // SmartDashboard.putNumber("Target Position X", targetPosition.getX());
    // SmartDashboard.putNumber("Target Position Y", targetPosition.getY());

    double angleToSpeaker = MathUtil.angleModulus(speakerPosition.minus(robotPoint).getAngle().getRadians());
    SmartDashboard.putNumber("Angle To Speaker", angleToSpeaker);

    return MathUtil.angleModulus(angleToSpeaker);
  }

  /**
   * This command allows the driver to keep driving, but forces the robot to face
   * the speaker.
   * 
   * @param x The desired {@code x} speed from {@code -1.0} to {@code 1.0}.
   * @param y The desired {@code y} speed from {@code -1.0} to {@code 1.0}.
   */
  public void driveOnTargetSpeaker(DoubleSupplier x, DoubleSupplier y) {
    boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        .equals(DriverStation.Alliance.Blue);
    double directionFlip = isBlue ? -1.0 : -1.0; // TODO remove this eventually
    driveAngle(directionFlip * y.getAsDouble(), directionFlip * x.getAsDouble(), getSpeakerAngle(),
        m_autoAimRotationPidController, false);
  }

  // gets robot relative note angle from 0 (Center)
  public double getNoteAngle() {
    var result = m_noteCamera.getLatestResult();
    if (result != null && result.getBestTarget() != null) {
      return result.getBestTarget().getYaw();
    } else {
      return 0;
    }

  }

  public double getNoteHeight() {
    var result = m_noteCamera.getLatestResult();
    if (result != null && result.getBestTarget() != null) {
      return result.getBestTarget().getPitch();
    } else {
      return -22;
    }

  }

  public void driveRobotRelativeToObject() {
    drive(
        -MathUtil.clamp((getNoteHeight() + 22) * 0.02, -DriveConstants.TRANSLATION_SPEED_SCALAR_AUTO_AIM,
            DriveConstants.TRANSLATION_SPEED_SCALAR_AUTO_AIM),
        getNoteAngle() * 0.003,
        -getNoteAngle() * 0.02,
        false, false);
  }

  // public void driveOnTargetNote(DoubleSupplier x, DoubleSupplier y) {
  // boolean isBlue =
  // DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
  // .equals(DriverStation.Alliance.Blue);
  // double directionFlip = isBlue ? -1.0 : 1.0;
  // driveAngle(directionFlip * y.getAsDouble(), directionFlip * x.getAsDouble(),
  // getNoteAngle(),
  // m_autoAimRotationPidController,
  // true);
  // }

  /**
   * This command gets the distance of the current shot to the speaker.
   * 
   * @return The distance in meters.
   */
  public double getSpeakerDistance() {
    return getPosition().getTranslation().getDistance(getSpeakerPosition());
  }

  public boolean getIsOnTargetSpeaker() {
    return m_autoAimRotationPidController.atSetpoint();
  }

}
