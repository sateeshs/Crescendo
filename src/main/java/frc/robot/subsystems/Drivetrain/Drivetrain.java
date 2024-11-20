package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.Vision.LimelightFront;
import frc.robot.Constants.Vision.LimelightRear;
import frc.robot.Robot;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.Vision.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.Music;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

@SuppressWarnings("unused")
/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005;

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private Field2d mField;

  public final SwerveRequest.ApplyChassisSpeeds autoRequest =
      new SwerveRequest.ApplyChassisSpeeds();

  @Override
  public void periodic() {
    super.setOperatorPerspectiveForward(Rotation2d.fromDegrees((Robot.isRed() ? 180 : 0)));

    if (Constants.Vision.UseLimelight && Robot.isReal()) {
      LimelightHelpers.SetRobotOrientation(
          Constants.Vision.LimelightFront.llAprilTag,
          super.m_odometry.getEstimatedPosition().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);

      LimelightHelpers.SetRobotOrientation(
          Constants.Vision.LimelightRear.llAprilTagRear,
          super.m_odometry.getEstimatedPosition().getRotation().getDegrees(),
          0,
          0,
          0,
          0,
          0);

      PoseEstimate front, back;

      if (DriverStation.isDisabled()
          || LimelightHelpers.getTA(Constants.Vision.LimelightFront.llAprilTag)
              > LimelightFront.MegaTag1AreaThreshold
          || Math.abs(super.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
              < LimelightFront.MegatTag2AngularVelocityThreshold) {
        front =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.LimelightFront.llAprilTag);
      } else {
        front =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                Constants.Vision.LimelightFront.llAprilTag);
      }

      if (DriverStation.isDisabled()
          || LimelightHelpers.getTA(Constants.Vision.LimelightRear.llAprilTagRear)
              > LimelightRear.MegaTag1AreaThreshold
          || Math.abs(super.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
              < LimelightRear.MegatTag2AngularVelocityThreshold) {
        back =
            LimelightHelpers.getBotPoseEstimate_wpiBlue(
                Constants.Vision.LimelightRear.llAprilTagRear);
      } else {
        back =
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                Constants.Vision.LimelightRear.llAprilTagRear);
      }

      if (front.tagCount > 1
          && LimelightHelpers.getTA(Constants.Vision.LimelightFront.llAprilTag)
              > LimelightFront.MegaTag2AreaThreshold) {
        // if (front.tagCount >= 1) {
        this.addVisionMeasurement(front.pose, front.timestampSeconds);
        mField.getObject("Front LL pose").setPose(front.pose);
      }

      if (back.tagCount > 1
          && LimelightHelpers.getTA(Constants.Vision.LimelightRear.llAprilTagRear)
              > LimelightRear.MegaTag2AreaThreshold) {
        // if (back.tagCount >= 1) {
        this.addVisionMeasurement(back.pose, back.timestampSeconds);
        mField.getObject("Rear LL pose").setPose(back.pose);
      }
    }

    SmartDashboard.putBoolean("Swerve/Is In Range", isInRangeOfTarget());
    SmartDashboard.putNumber(
        "Swerve/Rotation Error", (angleToSpeaker() - getPose().getRotation().getDegrees()));
  }

  private final SwerveRequest.SysIdSwerveTranslation TranslationCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveRotation RotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();
  private final SwerveRequest.SysIdSwerveSteerGains SteerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  /* Use one of these sysidroutines for your particular test */
  private SysIdRoutine SysIdRoutineTranslation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(TranslationCharacterization.withVolts(volts)), null, this));

  private final SysIdRoutine SysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(4),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(RotationCharacterization.withVolts(volts)), null, this));
  private final SysIdRoutine SysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Volts.of(7),
              null,
              (state) -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              (volts) -> setControl(SteerCharacterization.withVolts(volts)), null, this));

  /* Change this to the sysid routine you want to test */
  private final SysIdRoutine RoutineToApply = SysIdRoutineTranslation;

  private Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    for (SwerveModule mod : super.Modules) {
      Robot.verifyMotors(mod.getDriveMotor(), mod.getSteerMotor());
      Robot.verifyCANcoder(mod.getCANcoder());
    }

    if (Constants.Vision.UseLimelight) {
      super.setVisionMeasurementStdDevs(Constants.Vision.kPrecisionInMyVision);
    }

    configurePathPlanner();
    if (Utils.isSimulation()) {
      startSimThread();
    }

    mField = Robot.teleopField;
    if (Robot.isRed()) {
      mField.getObject("Speaker").setPose(Constants.Vision.SpeakerPoses.kSpeakerPoseRed);
    } else {
      mField.getObject("Speaker").setPose(Constants.Vision.SpeakerPoses.kSpeakerPoseBlue);
    }

    for (SwerveModule module : Modules) {
      Music.getInstance().addFalcon(module.getDriveMotor(), module.getSteerMotor());
    }

    postStatus("Idle");
  }

  private void configurePathPlanner() {
    AutoBuilder.configureHolonomic(
        () -> this.getState().Pose,
        this::seedFieldRelative,
        this::getCurrentRobotChassisSpeeds,
        (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)),
        new HolonomicPathFollowerConfig(
            Constants.AutoConstants.translationPID,
            Constants.AutoConstants.rotationPID,
            SwerveConstants.SwerveSpeeds.kMaxModuleSpeed,
            Constants.SwerveConstants.RobotMeasurements.kDriveBaseRadius,
            new ReplanningConfig(true, true, 1, 0.25)),
        Robot::isRed,
        this);

    // dont really care about the target pose
    // PathPlannerLogging.setLogTargetPoseCallback((pose) -> getField()
    // .getObject("Target Pose").setPose(pose));
    PathPlannerLogging.setLogActivePathCallback(
        (poses) -> getField().getObject("Trajectory").setPoses(poses));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  /*
   * Both the sysid commands are specific to one particular sysid routine, change
   * which one you're trying to characterize
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return RoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return RoutineToApply.dynamic(direction);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - m_lastSimTime;
              m_lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public ChassisSpeeds getCurrentRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public void addFieldObj(PathPlannerTrajectory trajectory) {
    List<Pose2d> poses = new ArrayList<>();
    AtomicInteger i = new AtomicInteger(0);
    trajectory
        .getStates()
        .forEach(
            (state) -> {
              if (!(state
                      .getTargetHolonomicPose()
                      .equals(trajectory.getInitialTargetHolonomicPose()))
                  && i.get() % 10 == 0) poses.add(state.getTargetHolonomicPose());
              i.incrementAndGet();
            });
    mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
  }

  public void addFieldObj(List<Pose2d> poses) {
    mField.getObject(Constants.AutoConstants.kFieldObjectName).setPoses(poses);
  }

  public Field2d getField() {
    return mField;
  }

  // ankur is mine hehehehhehehehehehehhehe
  public Pose2d getPose() {
    return getState().Pose;
  }

  public Constants.SwerveConstants.Target whatAmILookingAt() {
    double rotation = getPose().getRotation().getDegrees();
    if (rotation > -60 && rotation < -120) {
      return Constants.SwerveConstants.Target.kAmp;
    } else {
      return Constants.SwerveConstants.Target.kSpeaker;
    }
  }

  public double getDistanceToSpeaker() {
    Pose2d speakerPose;

    if (Robot.isRed()) {
      speakerPose = Constants.Vision.SpeakerPoses.kSpeakerPoseRed;
    } else {
      speakerPose = Constants.Vision.SpeakerPoses.kSpeakerPoseBlue;
    }

    /* Swerve Pose calculated in meters */
    Pose2d currentPose = Drivetrain.getInstance().getPose();
    double SpeakerY = speakerPose.getY();

    double distToSpeakerMeters =
        Math.sqrt(
            Math.pow(speakerPose.getX() - currentPose.getX(), 2)
                + Math.pow(SpeakerY - currentPose.getY(), 2));

    return Math.abs(distToSpeakerMeters);
  }

  public double angleToSpeaker() {
    if (Robot.isRed()) {
      return Rotation2d.fromRadians(
              Math.atan2(
                  getPose().getY() - Constants.Vision.SpeakerPoses.kSpeakerPoseRed.getY(),
                  getPose().getX() - Constants.Vision.SpeakerPoses.kSpeakerPoseRed.getX()))
          .getDegrees();
    } else {
      return Rotation2d.fromRadians(
              Math.atan2(
                  getPose().getY() - Constants.Vision.SpeakerPoses.kSpeakerPoseBlue.getY(),
                  getPose().getX() - Constants.Vision.SpeakerPoses.kSpeakerPoseBlue.getX()))
          .getDegrees();
    }
  }

  /**
   * Whether the robot is within a certain range of the speaker
   *
   * @param range the range in degrees
   * @return true if the robot is within the range, false otherwise
   */
  public boolean isInRangeOfTarget(double range) {
    return Math.abs(angleToSpeaker() - getPose().getRotation().getDegrees()) < range;
  }

  /**
   * Whether the robot is within 15 degrees of the speaker
   *
   * @return true if the robot is within 15 degrees, false otherwise
   */
  public boolean isInRangeOfTarget() {
    return isInRangeOfTarget(8);
  }

  public boolean readyToShoot() {
    return this.isInRangeOfTarget();
    // &&
    // Math.abs(this.getState().speeds.omegaRadiansPerSecond) < 0.1;
  }

  public boolean readyToAmp() {
    return Math.abs(getPose().getRotation().getDegrees() - 90) < 5;
    // return this.isInRangeOfTarget(8);
  }

  public void postStatus(String status) {
    SmartDashboard.putString("Swerve/status", status);
  }

  private static Drivetrain mInstance;

  public static Drivetrain getInstance() {
    if (mInstance == null) {
      mInstance =
          new Drivetrain(
              SwerveConstants.TunerConstants.DrivetrainConstants,
              SwerveConstants.TunerConstants.FrontLeft,
              SwerveConstants.TunerConstants.FrontRight,
              SwerveConstants.TunerConstants.BackLeft,
              SwerveConstants.TunerConstants.BackRight);
    }
    return mInstance;
  }

  public static Drivetrain getInstance(
      SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
    if (mInstance == null) {
      mInstance = new Drivetrain(SwerveConstants.TunerConstants.DrivetrainConstants, modules);
    }
    return mInstance;
  }
}
