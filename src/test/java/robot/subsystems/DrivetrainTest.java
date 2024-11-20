package robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.mockito.Mockito.mock;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.SwerveRequest;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

class DriverTrainTest {
  Drivetrain drivetrain;
  PWMSim m_simMotor;
  DoubleSolenoidSim m_simPiston;
  SwerveDrivetrainConstants driveTrainConstants;

  SwerveModuleConstants[] modules;

  /*----------------------------------SwerveModuleConstantsFactory parameters ------------------------------ */

  // The steer motor uses any SwerveModule.SteerRequestType control request with
  // the
  // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
  private static final Slot0Configs steerGains =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);

  // When using closed-loop control, the drive motor uses the control
  // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
  private static final Slot0Configs driveGains =
      new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

  /** The closed-loop output type to use for the steer motors */
  private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

  /** The closed-loop output type to use for the drive motors */
  private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  /** The stator current at which the wheels start to slip */
  private static final double kSlipCurrentA = Robot.isSimulation() ? 200.0 : 80.0;

  /** Theoretical free speed (m/s) at 12v applied output */
  public static final double kSpeedAt12VoltsMps = 5.96;
  // public static final double kSpeedAt12VoltsMps = ;

  /**
   * Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns This may need to be
   * tuned to your individual robot
   */
  private static final double kCoupleRatio = (2.765625 - 0.475586) / 1.011963;
  // private static final double kCoupleRatio = 6.12;

  private static final double kDriveGearRatio = 5.357142857142857;
  private static final double kSteerGearRatio = 21.428571428571427;

  // private static final double kWheelRadiusInches = (2 * (6.45 / 6.6)) *
  // (139.505 / 120);
  // private static final double kWheelRadiusInches = 3.9275 / 2;
  private static final double kWheelRadiusInches = 2 * (3.082 / 3.586);

  /*
   * X:1.4
   * Y:5.55
   *
   * X:-2.153
   * Y:5.359
   *
   * X-delta: 3.553
   * X-delta: 139.88 - 0.375
   */

  private static final boolean kSteerMotorReversed = false;
  private static final boolean kInvertLeftSide = true;
  private static final boolean kInvertRightSide = false;

  private static final String kCANbusName = "*";
  private static final int kPigeonId = 0;

  /** These are only used for simulation */
  private static final double kSteerInertia = 0.00001, kDriveInertia = 0.001;

  /** Simulated voltage necessary to overcome friction */
  private static final double kSteerFrictionVoltage = 0.25, kDriveFrictionVoltage = 0.25;

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId).withCANbusName(kCANbusName);
  /*----------------------------------SwerveModuleConstantsFactory parameters ------------------------------ */
  /*----------------------------------Swerve left right variables ----------------------------------------- */

  // Front Left
  private static final int kFrontLeftDriveMotorId = 11;
  private static final int kFrontLeftSteerMotorId = 12;
  private static final int kFrontLeftEncoderId = 10;
  private static final double kFrontLeftEncoderOffset = -0.321533203125;

  private static final double kFrontLeftXPosInches = 12.875;
  private static final double kFrontLeftYPosInches = 12.05;

  // Front Right
  private static final int kFrontRightDriveMotorId = 21;
  private static final int kFrontRightSteerMotorId = 22;
  private static final int kFrontRightEncoderId = 20;
  private static final double kFrontRightEncoderOffset = -0.375244140625;

  private static final double kFrontRightXPosInches = 12.875;
  private static final double kFrontRightYPosInches = -12.05;

  // Back Left
  private static final int kBackLeftDriveMotorId = 31;
  private static final int kBackLeftSteerMotorId = 32;
  private static final int kBackLeftEncoderId = 30;
  private static final double kBackLeftEncoderOffset = -0.178955078125;

  private static final double kBackLeftXPosInches = -12.875;
  private static final double kBackLeftYPosInches = 12.05;

  // Back Right
  private static final int kBackRightDriveMotorId = 41;
  private static final int kBackRightSteerMotorId = 42;
  private static final int kBackRightEncoderId = 40;
  private static final double kBackRightEncoderOffset = -0.088623046875;

  private static final double kBackRightXPosInches = -12.875;
  private static final double kBackRightYPosInches = -12.05;
  /*----------------------------------Swerve left right variables ----------------------------------------- */
  public static final double swerveTestSpeed = 0.05;

  // Create mock hardware devices

  // private FakeMotorController leftMotor;
  // private FakeMotorController rightMotor;

  /**
   * Lazy loading objects private Optional<HeavyResource> resource = Optional.empty();
   *
   * <p>public HeavyResource getResource() { resource = resource.or(() -> Optional.of(new
   * HeavyResource())); return resource.get(); }
   */
  @BeforeEach
  void setup() {
    driveTrainConstants = mock(SwerveDrivetrainConstants.class);
    // var FrontLeft = mock(SwerveModuleConstants.class);

    // var serveModuleConstantsFactory = mock(SwerveModuleConstantsFactory.class);
    var ConstantCreator =
        new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withWheelRadius(kWheelRadiusInches)
            .withSlipCurrent(kSlipCurrentA)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
            .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(kCoupleRatio)
            .withSteerMotorInverted(kSteerMotorReversed);

    final SwerveModuleConstants FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId,
            kFrontLeftDriveMotorId,
            kFrontLeftEncoderId,
            kFrontLeftEncoderOffset,
            Units.inchesToMeters(kFrontLeftXPosInches),
            Units.inchesToMeters(kFrontLeftYPosInches),
            kInvertLeftSide);
    final SwerveModuleConstants FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId,
            kFrontRightDriveMotorId,
            kFrontRightEncoderId,
            kFrontRightEncoderOffset,
            Units.inchesToMeters(kFrontRightXPosInches),
            Units.inchesToMeters(kFrontRightYPosInches),
            kInvertRightSide);
    final SwerveModuleConstants BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId,
            kBackLeftDriveMotorId,
            kBackLeftEncoderId,
            kBackLeftEncoderOffset,
            Units.inchesToMeters(kBackLeftXPosInches),
            Units.inchesToMeters(kBackLeftYPosInches),
            kInvertLeftSide);
    final SwerveModuleConstants BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId,
            kBackRightDriveMotorId,
            kBackRightEncoderId,
            kBackRightEncoderOffset,
            Units.inchesToMeters(kBackRightXPosInches),
            Units.inchesToMeters(kBackRightYPosInches),
            kInvertRightSide);
    modules = new SwerveModuleConstants[] {FrontLeft, FrontRight, BackLeft, BackRight};
    drivetrain = Drivetrain.getInstance(driveTrainConstants, modules);
  }

  @DisplayName("Drive Forward")
  @Test
  void testSetSpeed() {
    Command command =
        drivetrain.applyRequest(
            () ->
                new SwerveRequest.FieldCentric()
                    .withVelocityX(
                        Constants.SwerveConstants.SwerveSpeeds.kMaxSpeedMetersPerSecond
                            * swerveTestSpeed));
    assertNotNull(command);
    // command.
    // drivetrain.getPigeon2();
    // SwerveDrivetrain big

    // elevator.setSpeed(0.5);
    // assertEquals(0.5, motor.get(), 1e-9);
  }
}
