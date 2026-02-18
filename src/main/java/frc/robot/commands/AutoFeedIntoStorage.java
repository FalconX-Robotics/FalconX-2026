package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;

public class AutoFeedIntoStorage extends Command{
    private double velocity;
    private final Feeder feeder;
   
    private double speedofShooter;
    
    public AutoFeedIntoStorage(Feeder feeder){
        this.feeder = feeder;
    
        addRequirements(feeder);
    }

    public void initialize(){
        speedofShooter = feeder.getShooterSpeed();
        velocity = -speedofShooter;
        feeder.setFeederSpeed(velocity); //intake into storage
        }

    @Override
    public void execute() {
        speedofShooter = feeder.getShooterSpeed();
        velocity = speedofShooter;
        feeder.setFeederSpeed(velocity); //intake into storage
    }

    public void end(boolean interrupted){
        feeder.setFeederSpeed(0);

    }

}
