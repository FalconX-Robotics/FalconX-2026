package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class Feed extends Command{
    double velocity;
    private final Shooter feeder;

    public Feed(Shooter feeder, double velocity){
        this.feeder = feeder;
        this.velocity = velocity;
        addRequirements(feeder);
    }

    public void initialize(){
        feeder.setFeederSpeed(velocity);
    }

    public void end(boolean interrupted){
        feeder.setFeederSpeed(0);
    }
}
