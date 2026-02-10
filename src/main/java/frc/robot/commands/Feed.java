package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class Feed extends Command{
    double velocity;
    private final Shooter feeder;
    private double speedofShooter;

    public Feed(Shooter feeder){
        this.feeder = feeder;
        addRequirements(feeder);
    }

    public void initialize(){
        speedofShooter = feeder.getShooterSpeed();
        velocity = speedofShooter;
        feeder.setFeederSpeed(velocity);
    }

    public void end(boolean interrupted){
        feeder.setFeederSpeed(0);

    }

}

//     @Override
//     public boolean isFinished() {
     
// }
