package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Settings;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;


public class ManualShoot extends Command {
  private final Settings.OperatorSettings operatorSettings;
  private final Feeder feeder;
  private final Shooter shooter;

  public ManualShoot(RobotContainer robotContainer) {
    this.operatorSettings = robotContainer.settings.operatorSettings;
    this.feeder = robotContainer.subsystems.feeder;
    this.shooter = robotContainer.subsystems.shooter;
    
    super.addRequirements(this.feeder, this.shooter);
  }

  @Override
  public void execute() {
    // based on how far the trigger is pushed
    final double value = this.operatorSettings.getRightTriggerAxis();
    this.feeder.motor.set(-value * 0.5);
    this.shooter.motor.set(value);
  }

  public void end(boolean interrupted){
    this.feeder.motor.set(0.0);
    this.shooter.motor.set(0.0);
  }
}
