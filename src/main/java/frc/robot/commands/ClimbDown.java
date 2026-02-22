package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
  final TalonFX motor;
  Climber climberSubsystem;
  double voltageDown = -2;

  public ClimbDown() {
    this.climberSubsystem = RobotContainer.getRobotContainer().subsystems.climber;
    this.motor = this.climberSubsystem.motor;
  }

  @Override
  public void initialize() {
  // motor.setPosition(0);
  }

  @Override
  public void execute() {
    motor.setVoltage(voltageDown);
  }
  
  @Override
  public void end(boolean interrupted) {
    motor.setVoltage(0);
  }
}
