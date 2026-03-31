package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private final Shooter shooter;
  private final Field2d field;

  private final boolean isBlueHub = DriverStation.getAlliance().get() == Alliance.Blue;

  private final double offsetFactor = 2.0;

  private final Translation2d hubTranslation = new Translation2d(182.11 + (this.isBlueHub ? 287.0 : 0.0), 158.84);
  private final Rotation2d hubRotation = new Rotation2d(0.0, 0.0);
  private final Pose2d poseOfHub = new Pose2d(this.hubTranslation, this.hubRotation);

  public AutoShoot(RobotContainer robotContainer) {
    this.feeder = robotContainer.subsystems.feeder;
    this.shooter = robotContainer.subsystems.shooter;

    addRequirements(feeder, shooter);

    this.field = new Field2d();
  }

  @Override
  public void initialize() {
    final Pose2d robotPos = this.field.getRobotPose();

    final double a_sq = Math.pow(robotPos.getX() - this.poseOfHub.getX() + Math.sqrt(this.offsetFactor), 2.0);
    final double b_sq = Math.pow(robotPos.getY() - this.poseOfHub.getY() + Math.sqrt(this.offsetFactor), 2.0);
    final double hypotenuse = Math.sqrt(a_sq + b_sq);
    final double power = Util.getPowerFromDistance(hypotenuse);

    System.out.println("AUTOSHOOT POWER: " + power + "%");
    this.shooter.motor.set(power);
    this.feeder.motor.set(-power);

    field.close();
  }

  public void end(boolean interrupted) {
    feeder.motor.set(0.0);
    shooter.motor.set(0.0);
  }
}
