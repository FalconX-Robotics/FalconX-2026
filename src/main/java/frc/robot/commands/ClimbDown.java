package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
    final TalonFX motor;
    Climber climberSubsystem;
    double voltageDown = 2;

    public ClimbDown(Climber climberSubsystem) {
        this.climberSubsystem = climberSubsystem;
        this.motor = climberSubsystem.motor;
    }

    @Override
    public void initialize() {
        motor.setPosition(0);

    }

    @Override
    public void execute() {
        
        motor.setVoltage(voltageDown);
        
        
    }
    
    @Override
    public boolean isFinished() {
        return climberSubsystem.IsAtBottom();
    }

   
}
