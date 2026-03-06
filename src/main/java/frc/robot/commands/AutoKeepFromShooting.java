package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotContainer;
import frc.robot.Settings;
import frc.robot.subsystems.Feeder;

public class AutoKeepFromShooting extends Command {
    //used paired with GetToSpeed
  private final Settings.OperatorSettings operatorSettings;
  private final Feeder feeder;
  private final RotateToTarget rotateToTarget;
  private final GetToSpeed getToSpeed;

  public AutoKeepFromShooting() {
    final RobotContainer robotContainer = RobotContainer.getRobotContainer();
    this.operatorSettings = robotContainer.settings.operatorSettings;
    this.feeder = robotContainer.subsystems.feeder;
    this.rotateToTarget = robotContainer.commands.rotateToTarget;
    this.getToSpeed = robotContainer.commands.getToSpeed;

    addRequirements(this.feeder);
  }
    
  @Override
  public void execute() {
    // based on how far the trigger is pushed
    double value = feeder.getShooterSpeed();
    this.feeder.motor.set(value);
  }

  public void end(boolean interrupted){
    this.feeder.motor.set(0.0);
  }
  @Override
  public boolean isFinished() {
      return !CommandScheduler.getInstance().isScheduled(getToSpeed) && !CommandScheduler.getInstance().isScheduled(rotateToTarget);
  }
}
