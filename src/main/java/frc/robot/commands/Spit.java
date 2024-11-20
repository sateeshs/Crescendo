package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class Spit extends Command {
  Shooter shooter;
  Intake intake;

  public Spit() {
    shooter = Shooter.getInstance();
    intake = Intake.getInstance();

    this.addRequirements(shooter, intake);
    this.setName("Spit");
  }

  @Override
  public void initialize() {
    intake.postStatus("Spitting");
    shooter.postStatus("Spitting");

    shooter.shoot(1, 1);

    intake.setExtend(false);
    intake.setIntake(-1);
    intake.setBelt(-1);
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    intake.stop();
    intake.setHolding(false);

    intake.postStatus("Idle");
    shooter.postStatus("Idle");
  }
}
