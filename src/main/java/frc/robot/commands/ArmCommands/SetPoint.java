package frc.robot.commands.ArmCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class SetPoint extends Command {
  Arm arm;
  double armSetpoint;
  boolean autoAim;

  /**
   * Set the shooter to a specific position
   *
   * <p>Only use this if the shooter sensor is not working
   *
   * @param target in degrees of THE SHOOTER, not the extension bar
   */
  public SetPoint(double target) {
    arm = Arm.getInstance();
    armSetpoint = target;

    autoAim = false;

    this.addRequirements(arm);
    this.setName("Setpoint: " + armSetpoint + " degrees");
  }

  /**
   * Set the shooter to a calculated position
   *
   * <p>Only use this if the shooter sensor is not working
   */
  public SetPoint() {
    arm = Arm.getInstance();

    autoAim = true;

    this.setName("Auto Setpoint");
    this.addRequirements(arm);
  }

  @Override
  public void initialize() {
    if (autoAim) {
      armSetpoint = arm.calculateArmSetpoint();
      SmartDashboard.putNumber("Arm/Calculated Setpoint", armSetpoint);
      arm.postStatus("Tracking Speaker");
    } else {
      arm.postStatus("Traveling to " + armSetpoint + " degrees");
    }

    arm.setMotionMagic(armSetpoint);
  }

  @Override
  public void execute() {
    if (autoAim) {
      armSetpoint = arm.calculateArmSetpoint();
      SmartDashboard.putNumber("Arm/Calculated Setpoint", armSetpoint);
      arm.setMotionMagic(armSetpoint);
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      arm.postStatus("Returning Home");
      arm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);
    }
  }

  @Override
  public boolean isFinished() {
    if (autoAim) {
      return false;
    } else {
      return arm.atSetpoint();
    }
  }
}
