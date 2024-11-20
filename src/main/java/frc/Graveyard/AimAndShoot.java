// Here lies the code of @Ardusa
// it did what it was supposed to do pretty well
// but in got the death from the big man

// package frc.robot.commands.ShooterCommands;

// import java.util.function.Supplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.Robot;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;

// public class AimAndShoot extends Command {
//     private Arm mArm;
//     private Shooter mShooter;
//     private Intake mIntake;

//     private double armSetpoint;
//     private double shootSpeed = 1;

//     private boolean autoAim = false;
//     private boolean pieceHasBeenLoaded;

//     private boolean debugMode = false;

//     /** when this returns true, this indicates that the user wants to shoot */
//     private Supplier<Boolean> chargeUntil;

//     /**
//      * @deprecated
//      * Calculates the desired setpoint of the arm using robotPose and then charges
//      * the shooter motors so that when the arm gets to the correct position, it is
//      * ready to feed and shoot
//      */
//     public AimAndShoot() {
//         mArm = Arm.getInstance();
//         mShooter = Shooter.getInstance();
//         mIntake = Intake.getInstance();

//         autoAim = true;
//         chargeUntil = () -> true;

//         this.setName("Auto aim and shoot");
//         this.addRequirements(mArm, mShooter, mIntake);
//     }

//     /**
//      * @deprecated
//      * Calculates the desired setpoint of the arm using robotPose and then charges
//      * the shooter motors. When the user presses the defined button, the shooter
//      * will shoot and the command ends
//      *
//      * @param chargeUntil the boolean supplier that, when returns true, will shoot
//      *                    the piece
//      */
//     public AimAndShoot(Supplier<Boolean> chargeUntil) {
//         mArm = Arm.getInstance();
//         mShooter = Shooter.getInstance();
//         mIntake = Intake.getInstance();
//         autoAim = true;

//         this.chargeUntil = chargeUntil;

//         this.setName("Auto aim");
//         this.addRequirements(mArm, mShooter, mIntake);
//     }

//     /**
//      * @deprecated
//      * Set the shooter to a specific position and shoots when within 1 degree
//      *
//      * @param target in degrees of THE SHOOTER, not the extension bar
//      */
//     public AimAndShoot(double target) {
//         mArm = Arm.getInstance();
//         mShooter = Shooter.getInstance();
//         mIntake = Intake.getInstance();

//         armSetpoint = target;
//         autoAim = false;
//         chargeUntil = () -> true;

//         this.setName("Aim and Shoot: " + armSetpoint + " degrees");
//         this.addRequirements(mArm, mShooter, mIntake);
//     }

//     /**
//      * @deprecated
//      * Set the shooter to a specific position and shoots when within 1 degree
//      *
//      * @param target      in degrees of THE SHOOTER, not the extension bar
//      * @param chargeUntil the boolean supplier that, when returns true, will shoot
//      *                    the piece
//      */
//     public AimAndShoot(double target, Supplier<Boolean> chargeUntil) {
//         mArm = Arm.getInstance();
//         mShooter = Shooter.getInstance();
//         mIntake = Intake.getInstance();

//         armSetpoint = target;
//         autoAim = false;
//         this.chargeUntil = chargeUntil;

//         this.setName("Aim and Shoot (Charge Until): " + armSetpoint + " degrees");
//         this.addRequirements(mArm, mShooter, mIntake);
//     }

//     @Override
//     public void initialize() {
//         pieceHasBeenLoaded = false;

//         if (mIntake.getShooterSensor()) {
//             pieceHasBeenLoaded = true;
//         }

//         shooter.postStatus("Charging Up");

//         if (debugMode) {
//             System.out.println("\n*************************** Debug Stats (initialize)
// ***************************");
//             System.out.println("Shooter position: " + mArm.getArmPosition());
//             System.out.println("Shooter target position: " + armSetpoint);
//             System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
//             System.out.println("*************************** Debug Stats (initialize)
// ***************************\n");
//         }
//     }

//     @Override
//     public void execute() {
//         if (autoAim) {
//             armSetpoint = mArm.calculateArmSetpoint();
//         }

//         // Pose2d speakerPose;

//         // if (Robot.isRed()) {
//         //     speakerPose = Constants.Vision.SpeakerPoseRed;
//         // } else {
//         //     speakerPose = Constants.Vision.SpeakerPoseBlue;
//         // }

//         // if (Math.abs(speakerPose.getY()
//         //         - Drivetrain.getInstance().getPose().getY()) <
// Constants.Vision.SpeakerDeadBand
//         //         && Drivetrain.getInstance().getPose().getX() < 1.6) {
//         //     shootSpeed = 0.6;
//         // } else {
//         //     shootSpeed = 1;
//         // }

//         // If shooter is empty
//         if (!mIntake.getShooterSensor()) {
//             // Increased the spead of feeder setVal to 0.5 from 0.1
//             // mShooter.shoot(0.5, shootSpeed);

//             // end(true);

//             // if (pieceHasBeenLoaded) {
//             //     end(false);
//             // } else {
//             //     end(true);
//             // }

//             mShooter.shoot(Constants.ShooterConstants.feederFeedForward, 0);
//             mIntake.setBelt(0.75);
//             shooter.postStatus("Feeding");
//         }

//         // else if the arm is within 1.5 degrees of the target and the arm is not moving
//         else if (mArm.atSetpoint()) {
//             pieceHasBeenLoaded = true;

//             // if the shooter i;//s ready to shoot and the user has pressed the button
//             if ((mShooter.readyToShoot()) && chargeUntil.get()) {
//                 mShooter.shoot(Constants.ShooterConstants.feederShootValue, shootSpeed);
//                 shooter.postStatus("Shooting");
//             }

//             // if the shooter isnt charge up yet or the user has not said to shoot yet
//             else {
//                 mShooter.shoot(0, shootSpeed);
//                 shooter.postStatus("Charging Up");
//             }
//         }

//         // If shooter is loaded but arm is not in position
//         else {
//             pieceHasBeenLoaded = true;
//             mArm.setMotionMagic(armSetpoint);
//             mShooter.shoot(0, shootSpeed);
//             shooter.postStatus("Traveling to Setpoint");
//         }

//         if (debugMode) {
//             System.out.println("\n*************************** Debug Stats (execute)
// ***************************");
//             System.out.println("Shooter position: " + mArm.getArmPosition());
//             System.out.println("Shooter target position: " + armSetpoint);
//             System.out.println("Error: " + (armSetpoint - mArm.getArmPosition()));
//             System.out.println("*************************** Debug Stats (execute)
// ***************************\n");
//         }
//     }

//     @Override
//     public boolean isFinished() {
//         if (Robot.isSimulation()) {
//             return false;
//         }

//         return pieceHasBeenLoaded && !mIntake.getShooterSensor();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         if (pieceHasBeenLoaded && !mIntake.getShooterSensor()) {
//             mIntake.setHolding(false);
//         }

//         mArm.setMotionMagic(Constants.ArmConstants.SetPoints.kIntake);

//         if (interrupted) {

//             shooter.postStatus("Aim And Shoot Interrupted");
//         }
//     }
// }
