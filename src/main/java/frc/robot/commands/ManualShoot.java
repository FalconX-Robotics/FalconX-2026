package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;


public class ManualShoot extends Command {
  // private final Feeder feeder;
  private final Shooter shooter;

  //TODO: 3 below = adjust later based on testing
  private double maxvelocity = 4;
  private double acceleration = 0.5; 
  private double lessenough = 0.25; //subtracted from max velocity -> how much less than max velocity should the motor be for it to start gaining speed again

  public ManualShoot() {
    this.shooter = RobotContainer.getRobotContainer().subsystems.shooter;

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    shooter.setShooterAcceleration(acceleration);
  }

  @Override
  public void execute() {
    if (shooter.getShooterSpeed() >= maxvelocity) {
      shooter.setShooterAcceleration(0);
    }
    if (shooter.getShooterSpeed() < maxvelocity-lessenough) {
      shooter.setShooterAcceleration(acceleration);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterAcceleration(0);
  }
}
