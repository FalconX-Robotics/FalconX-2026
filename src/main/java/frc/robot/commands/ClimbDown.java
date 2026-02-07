package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climber;

public class ClimbDown extends Command {
    final TalonFX motor;
    Climber climberSystem;
    double voltageDown = 2;

    public ClimbDown(Climber climberSubsytem) {
        this.climberSystem = climberSubsytem;
        this.motor = climberSubsytem.motor;
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
        return climberSystem.IsAtBottom();
    }

   
}
