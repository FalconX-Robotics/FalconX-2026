package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
  final Climber climberSubsystem;
  double voltageDown = -2;

  public ClimbDown(RobotContainer robotContainer) {
    this.climberSubsystem = robotContainer.subsystems.climber;

    super.addRequirements(this.climberSubsystem);
  }

  @Override
  public void execute() {
    this.climberSubsystem.motor.setVoltage(voltageDown);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.motor.setVoltage(0);
  }
}
