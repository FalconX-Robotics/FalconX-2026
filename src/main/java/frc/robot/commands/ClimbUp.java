package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
  final TalonFX motor;       
  // final double maxRotations = 5.0; // test & change later
  // double rotationsDone = 0.0;
  final Climber climberSubsystem;

  public ClimbUp() {
    this.climberSubsystem = RobotContainer.getRobotContainer().subsystems.climber;
    this.motor = this.climberSubsystem.motor;
  }

  @Override
  public void execute() {
    motor.setVoltage(2.0);
  }

  @Override
  public boolean isFinished() {
    return climberSubsystem.ClimbUpDone();
  }

  @Override
  public void end(boolean interrupted) {
    motor.setVoltage(0);
  }
}
