package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

public class GetToSpeed extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final Shooter shooter;


    public GetToSpeed(SwerveSubsystem swerveSubsystem, Shooter shooter) {
       
        this.swerveSubsystem = swerveSubsystem;
        this.shooter = shooter;
        addRequirements(shooter);
    }
    public void initialize(){
        Vector2 targetPosition = Util.getTargetPosition();
        Vector2 robotPosition = new Vector2(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY());
        double distance = targetPosition.distance(robotPosition);

        shooter.setShooterSpeed(Util.findVelocity(distance));
    } 
    // public boolean isFinished() {
        
    // }

}
