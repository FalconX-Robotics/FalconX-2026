package frc.robot.commands;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Util;

public class Feed extends Command{
    double velocity;
    private final Shooter shooter;
    private final Feeder feeder;
    private double speedofShooter;

    public Feed(Shooter shooter, Feeder feeder){
        this.shooter = shooter;
        this.feeder = feeder;
        addRequirements(feeder, shooter);
    }

    public void initialize(){
        speedofShooter = shooter.getShooterSpeed();
        velocity = speedofShooter;
        feeder.setFeederSpeed(velocity);
        shooter.setShooterSpeed(velocity);
    }

    public void end(boolean interrupted){
        feeder.setFeederSpeed(0);

    }

}

//     @Override
//     public boolean isFinished() {
     
// }
