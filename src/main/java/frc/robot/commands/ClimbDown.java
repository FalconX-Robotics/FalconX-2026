package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
  final Climber climberSubsystem;
  final TalonFX motor;
  double voltageDown = -2;
  double maxrotations = 10; //TODO: test value

  public ClimbDown(RobotContainer robotContainer) {
    this.climberSubsystem = robotContainer.subsystems.climber;
    this.motor = this.climberSubsystem.motor;
    addRequirements(this.climberSubsystem);
  }

  @Override
  public void initialize() {
    motor.setPosition(0);
  }

  @Override
  public void execute() {
    this.climberSubsystem.motor.setVoltage(voltageDown);
  }
  
  @Override
  public void end(boolean interrupted) {
    this.climberSubsystem.motor.setVoltage(0);
  }

  @Override
  public boolean isFinished() {
      return motor.getPosition().getValueAsDouble() >= maxrotations;
  }
}
