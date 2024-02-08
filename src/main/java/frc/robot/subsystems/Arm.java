package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.shooter.SetPoint;
import frc.robot.LoggyThings.LoggyCANcoder;
import frc.robot.LoggyThings.LoggyTalonFX;
// import frc.robot.Subsystems.Music;

public class Arm extends SubsystemBase {
    private static Arm mInstance;
    private LoggyTalonFX armMotor;
    private LoggyCANcoder armCoder;
    private double shooterExtensionSetpoint = 0;

    private VelocityDutyCycle velocityDutyCycle;

    private Mechanism2d armMechanism;
    private MechanismLigament2d shooterLigament, shooterExtension, elbowLigament;

    private Mechanism2d simArmMechanism;
    private MechanismLigament2d simShooterLigament, simShooterExtension, simElbowLigament;

    public static Arm getInstance() {
        if (mInstance == null) {
            mInstance = new Arm();
        }

        return mInstance;
    }

    @Override
    public void periodic() {
        if (Robot.isReal()) {
            shooterExtension.setAngle(getShooterExtensionPosition() - 90);
        }
        simShooterExtension.setAngle(shooterExtensionSetpoint - 90);

        if (Robot.isReal()) {
            if (isInRangeOfTarget(getArmPositionFromExtension(shooterExtension.getAngle() - 90))) {
                elbowLigament.setColor(new Color8Bit(Color.kGreen));
            } else {
                elbowLigament.setColor(new Color8Bit(Color.kWhite));
            }
        }

        SmartDashboard.putNumber("Arm/Arm Position", getArmPosition());
        SmartDashboard.putBoolean("Arm/At Setpoint", isInRangeOfTarget(shooterExtensionSetpoint));
    }

    private Arm() {
        armCoder = new LoggyCANcoder(Constants.ArmConstants.armEncoderID, false);
        CANcoderConfiguration armCoderConfig = new CANcoderConfiguration();

        /*
         * Do this directly on the CANcoder, so as to not reset the zero position
         */
        armCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        armCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        /* Do not apply */
        // armCoder.getConfigurator().apply(armCoderConfig);

        armMotor = new LoggyTalonFX(Constants.ArmConstants.armMotorID, false);
        TalonFXConfiguration armMotorConfig = new TalonFXConfiguration();
        armMotorConfig.Feedback.SensorToMechanismRatio = 2;
        armMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
        armMotorConfig.Feedback.RotorToSensorRatio = 50;
        armMotorConfig.Feedback.FeedbackRemoteSensorID = armCoder.getDeviceID();

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        armMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMaxAngle);
        armMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Units
                .degreesToRotations(Constants.ArmConstants.kArmMinAngle);

        armMotorConfig.Slot0.kV = 04;
        armMotorConfig.Slot0.kP = 100;
        armMotorConfig.Slot0.kI = 5;
        armMotorConfig.Slot0.kG = 0.03;
        armMotorConfig.Slot0.kD = 0.2;

        armMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        armMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        armMotorConfig.Audio.AllowMusicDurDisable = true;

        MotionMagicConfigs motionMagicConfigs = armMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 0.25;
        motionMagicConfigs.MotionMagicAcceleration = 0.25;
        motionMagicConfigs.MotionMagicJerk = 100;

        armMotor.getConfigurator().apply(armMotorConfig);

        // Music.getInstance().addFalcon(armMotor);

        double shooterHeightInches = 22;
        double shooterLengthInches = 9.5;
        double shooterExtensionInches = 4.5;
        double shooterExtensionAngle = -90;
        double elbowLigamentAngle = -Constants.ArmConstants.shooterOffset;
        double rootXInches = Units.metersToInches(0.7493);
        double rootYInches = 4;

        if (Robot.isReal()) {
            /* Motor Profile Mechanism */
            armMechanism = new Mechanism2d(0.7493 * 2, Units.inchesToMeters(
                    rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

            shooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                    new Color8Bit(Color.kWhite));

            shooterExtension = new MechanismLigament2d("Shooter Extension",
                    Units.inchesToMeters(shooterExtensionInches), shooterExtensionAngle, 2, new Color8Bit(Color.kGray));

            elbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                    elbowLigamentAngle, 5, new Color8Bit(Color.kWhite));

            armMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                    .append(shooterLigament).append(shooterExtension).append(elbowLigament);
            SmartDashboard.putData("Arm/Motor Profile", armMechanism);
        }

        /* Create Target Mechanism */
        simArmMechanism = new Mechanism2d(0.7493 * 2, Units
                .inchesToMeters(rootYInches + shooterHeightInches + shooterExtensionInches + shooterLengthInches + 5));

        simShooterLigament = new MechanismLigament2d("Shooter Bars", Units.inchesToMeters(shooterHeightInches), 90, 9,
                new Color8Bit(Color.kRed));

        simShooterExtension = new MechanismLigament2d("Shooter Extension", Units.inchesToMeters(shooterExtensionInches),
                shooterExtensionAngle, 2, new Color8Bit(Color.kDarkRed));

        simElbowLigament = new MechanismLigament2d("Shooter", Units.inchesToMeters(shooterLengthInches),
                elbowLigamentAngle, 5, new Color8Bit(Color.kRed));

        simArmMechanism.getRoot("Root", Units.inchesToMeters(rootXInches), Units.inchesToMeters(rootYInches))
                .append(simShooterLigament).append(simShooterExtension).append(simElbowLigament);
        SmartDashboard.putData("Arm/Target Profile", simArmMechanism);

    }

    /**
     * whether the arm is within 0.5 degrees of the target
     * 
     * @param target target position in degrees
     * @return true if in range or false if out of range
     */
    public boolean isInRangeOfTarget(double target) {
        if ((getArmPosition() - target) < 0.5) {
            return true;
        } else {

            return false;
        }
    }

    /**
     * Whether or not the Arm is within the allowable range of motion
     * 
     * @param setpoint the Shooter setpoint in degrees
     * @return true if within range, false if not
     */
    public boolean validSetpoint(double setpoint) {
        if (setpoint > Constants.ArmConstants.kArmMinAngle && setpoint < Constants.ArmConstants.kArmMaxAngle) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * supply a specific value [-1,1] to the arm motor, has a deadzone of 0.06
     * 
     * @param setVal the value to set the arm motor to [-1,1]
     */
    public void aim(double setVal) {
        if (Math.abs(setVal) <= 0.06) {
            setVal = 0.0;
            if (getArmPosition() < 35) {
                setVal = 0.02;
            }
        }

        armMotor.set(setVal);
    }

    /**
     * supply a specific value [-1,1] to the arm motor
     * 
     * @param setVal the value to set the arm motor to [-1,1]
     */
    public void aimRaw(double setVal) {
        if (Math.abs(setVal) > 0.25) {
            setVal = 0;
        }

        armMotor.set(setVal);
    }

    public void aimVelocity(double velocity) {
        armMotor.setControl(velocityDutyCycle.withVelocity(velocity));
    }

    /**
     * Returns the position of the arm in degrees
     * 
     * @return position in degrees
     */
    public double getArmPosition() {
        if (Robot.isSimulation()) {
            return simShooterExtension.getAngle() - Constants.ArmConstants.shooterOffset + 90;
        }
        return Units.rotationsToDegrees(armMotor.getPosition().getValueAsDouble());
    }

    public double getShooterExtensionPosition() {
        return getArmPosition() + Constants.ArmConstants.shooterOffset;
    }

    /**
     * Sets the Sim Arm Target Mechanism to a specific position
     * 
     * @param target in degrees
     */
    public void setArmTarget(double target) {
        shooterExtensionSetpoint = target + Constants.ArmConstants.shooterOffset;
    }

    public double getVelocityRotations() {
        return armMotor.getVelocity().getValueAsDouble();
    }

    public double getVelocity() {
        // if (Robot.isSimulation()) {
        //     return 0;
        // }

        return armMotor.getVelocity().getValueAsDouble() * 360;
    }

    public double calculateArmSetpoint() {
        /* Swerve Pose calculated in meters */
        double returnVal = Constants.ArmConstants.SetPoints.kSpeakerClosestPoint;
        Pose2d currentPose = Drivetrain.getInstance().getState().Pose;
        double distToSpeaker = Math.sqrt(Math.pow(currentPose.getX(), 2) + Math.pow(currentPose.getY(), 2));

        /* TODO: Algorithm to calculate arm setpoint */
        // System.out.println("Distance to speaker" + distToSpeaker);
        returnVal += distToSpeaker * 3;
        // if (distToSpeaker < fieldLength / 5) { // 10% of the field length
        //     returnVal = Constants.ArmConstants.SetPoints.kSpeakerClosestPoint;
        // } else {
        //     returnVal = Constants.ArmConstants.SetPoints.kHorizontal;
        // }

        /* Make sure that we dont accidentally return a stupid value */
        if (validSetpoint(returnVal)) {
            setArmTarget(returnVal);
            return returnVal;
        } else {
            System.out.println(returnVal + " is not a valid setpoint");
            return getArmPosition();
        }
    }

    public double getArmPositionFromExtension(double extension) {
        return extension - Constants.ArmConstants.shooterOffset;
    }

    public double getExtensionFromArmPosition(double armPosition) {
        return armPosition + Constants.ArmConstants.shooterOffset;
    }

    public void setBrake(boolean brake) {
        NeutralModeValue mode = NeutralModeValue.Coast;
        if (brake) {
            mode = NeutralModeValue.Brake;
        }

        armMotor.setNeutralMode(mode);
    }
}
