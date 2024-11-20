// package frc.robot;

import static org.junit.jupiter.api.Assertions.assertAll;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.Drivetrain.Drivetrain;
import java.util.Arrays;
import java.util.List;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

// The Java Doc for the JUnit Jupiter classes can be found at
// https://junit.org/junit5/docs/current/api/org.junit.jupiter.api/org/junit/jupiter/api/package-summary.html
// The user guide is at
// https://junit.org/junit5/docs/current/user-guide/

class TestExample {
  Drivetrain drivetrain;

  @BeforeEach
  void setup() {
    // leftMotor = new FakeMotorController();
    // rightMotor = new FakeMotorController();
    // drivetrain = new Drivetrain(leftMotor, rightMotor);
    // motor = new FakeMotorController(0);
    // elevator = new Elevator(motor);
    // drivetrain = new Drivetrain(leftMotor, rightMotor);
    // m_simMotor =
    //     new PWMSim(IntakeConstants.kMotorPort); // create our simulation PWM motor controller
    // m_simPiston =
    //     new DoubleSolenoidSim(
    //         PneumaticsModuleType.CTREPCM,
    //         IntakeConstants.kPistonFwdChannel,
    //         IntakeConstants.kPistonRevChannel); // create our simulation solenoid
    drivetrain = Drivetrain.getInstance();
  }

  @Test
  void lambdaExpressions() {
    List<Integer> numbers = Arrays.asList(1, 2, 3);
    assertTrue(
        numbers.stream().mapToInt(Integer::intValue).sum() > 5,
        () -> "Sum should be greater than 5");
  }

  // The following test is skipped because it is an example of how to use Lambda
  // functions and assertAll, but as written it fails.
  @Disabled("Example failure")
  @Test
  void groupAssertions() {
    int[] numbers = {0, 1, 2, 3, 4};
    assertAll(
        "numbers",
        () -> assertEquals(1, numbers[0]),
        () -> assertEquals(3, numbers[3]),
        () -> assertEquals(1, numbers[4]));
  }
}
