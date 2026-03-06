package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private double speedofShooter;
  private double velocity;

  public AutoShoot(RobotContainer robotContainer) {
    this.feeder = robotContainer.subsystems.feeder;
    
    super.addRequirements(feeder);
  }

  @Override
  public void initialize() {
    speedofShooter = feeder.getShooterSpeed();
    velocity = speedofShooter;
    feeder.setFeederSpeed(velocity);    //remove from storage
  }

  @Override
  public void execute() {
    speedofShooter = feeder.getShooterSpeed();
    velocity = speedofShooter;
    feeder.setFeederSpeed(velocity); //remove from storage
  }

  public void end(boolean interrupted) {
    feeder.setFeederSpeed(0);
  }
}
