package frc.robot.commands;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbUp extends Command {
    final TalonFX motor;       
    final double maxRotations = 5.0; // test & change later
    double rotationsDone = 0.0;
    double voltageUp = 2;
    final Climber climberSubsystem;
    

    public ClimbUp(Climber climberSubsytem) {
        this.climberSubsystem = climberSubsytem;
        this.motor = climberSubsytem.motor;
    }

    @Override
    public void initialize() {
        motor.setPosition(0);

    }

    @Override
    public void execute() {
        
        motor.setVoltage(voltageUp);
        
        rotationsDone = motor.getPosition().getValueAsDouble();
        
    }
    
    @Override
    public boolean isFinished() {
        return rotationsDone > maxRotations;
    }

    
    

}
