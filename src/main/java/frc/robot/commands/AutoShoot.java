package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends Command {
  private final Feeder feeder;
  private final Shooter shooter;
  private double speedofShooter;
  private double velocity;

  public AutoShoot(RobotContainer robotContainer) {
    this.feeder = robotContainer.subsystems.feeder;
     this.shooter = robotContainer.subsystems.shooter;

    addRequirements(feeder, shooter);
  }

  @Override
  public void initialize() {
    speedofShooter = shooter.getShooterSpeed();
    velocity = speedofShooter;
    feeder.setFeederSpeed(-velocity);    //remove from storage
  }

  @Override
  public void execute() {
    speedofShooter = shooter.getShooterSpeed();
    velocity = speedofShooter;
    feeder.setFeederSpeed(-velocity); //remove from storage
    // System.out.println("Autoshoot");
    
      // System.out.println("AutoShoot");
  }

  public void end(boolean interrupted) {
    feeder.motor.set(0);
    // System.out.println("feederspeed set to 0");
    shooter.motor.set(0);
    // System.out.println("AutoShoot ended");
  }
}
