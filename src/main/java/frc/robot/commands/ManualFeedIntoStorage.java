package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;


public class ManualFeedIntoStorage extends Command {
    private final Feeder feeder;

    //TODO: 3 below = adjust later based on testing
    private double maxvelocity = 4;
    private double acceleration = -0.5; 
    private double lessenough = 0.25; //subtracted from max velocity -> how much less than max velocity should the motor be for it to start up gaining speed again

    public ManualFeedIntoStorage(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

    @Override
    public void initialize() {
        feeder.setFeederAcceleration(acceleration);    //into storage TODO: check if negative or positive acceleration
    }

    @Override
    public void execute() {
        if (feeder.getFeederSpeed() >= maxvelocity) {
            feeder.setFeederAcceleration(0);
        }
         if ((feeder.getFeederSpeed() < maxvelocity-lessenough) && (feeder.getFeederAcceleration() == 0)) {
            feeder.setFeederAcceleration(acceleration);
        }
    }

    public void end(boolean interrupted){
        feeder.setFeederAcceleration(0);

    }
}
