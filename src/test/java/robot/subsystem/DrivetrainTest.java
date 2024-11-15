//package frc.robot.subsystems.Drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import frc.robot.subsystems.Drivetrain.Drivetrain;

class DriverTrainTest {
  Drivetrain drivetrain;
  PWMSim m_simMotor;
  DoubleSolenoidSim m_simPiston;
  //private FakeMotorController leftMotor;
  //private FakeMotorController rightMotor;

  @BeforeEach
  void setup() {
    // leftMotor = new FakeMotorController();
    // rightMotor = new FakeMotorController();
    // drivetrain = new Drivetrain(leftMotor, rightMotor);
    // motor = new FakeMotorController(0);
    // elevator = new Elevator(motor);
    //drivetrain = new Drivetrain(leftMotor, rightMotor);
    m_simMotor =
        new PWMSim(IntakeConstants.kMotorPort); // create our simulation PWM motor controller
    m_simPiston =
        new DoubleSolenoidSim(
            PneumaticsModuleType.CTREPCM,
            IntakeConstants.kPistonFwdChannel,
            IntakeConstants.kPistonRevChannel); // create our simulation solenoid
    Drivetrain.getInstance();
  }

  @Disabled("Example test")
  @Test
  void testSetSpeed() {
    elevator.setSpeed(0.5);
    assertEquals(0.5, motor.get(), 1e-9);
  }
}
