package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hootake;
import frc.robot.util.Elastic;

public class Intake extends Command {

    Hootake fuelSubsystem;

    public Intake(Hootake fuelSystem) {

        addRequirements(fuelSystem);
        this.fuelSubsystem = fuelSystem;
    }

    @Override 
    public void initialize(){
        
    }
    

}
