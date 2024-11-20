package frc.robot.commands.FeederCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import java.util.function.Supplier;

public class BeltDrive extends Command {
  Intake intake;
  Supplier<Double> manualAdjust;

  /**
   * Indexes the belt manualy
   *
   * @param manualAdjust how much the belt should move
   */
  public BeltDrive(Supplier<Double> manualAdjust) {
    intake = Intake.getInstance();

    this.manualAdjust = manualAdjust;

    this.addRequirements(intake);
    this.setName("Belt Drive");
  }

  @Override
  public void execute() {
    intake.setBelt(manualAdjust.get());
    intake.postStatus("Manually Adjusting Belt");
  }

  @Override
  public void end(boolean interrupted) {
    intake.setBelt(0);
  }
}
