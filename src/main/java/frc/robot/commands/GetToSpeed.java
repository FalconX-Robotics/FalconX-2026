package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

public class GetToSpeed extends Command {
  private final SwerveSubsystem swerveSubsystem;
  private final Shooter shooter;
  private boolean IsActive = false;
  private double velocity;
  private double tolerance = 0.0001;

  public GetToSpeed(SwerveSubsystem swerveSubsystem, Shooter shooter) {
    this.swerveSubsystem = swerveSubsystem;
    this.shooter = shooter;
    addRequirements(shooter);
  }
  
  public void initialize(){
    Vector2 targetPosition = Util.getTargetPosition();
    Vector2 robotPosition = new Vector2(swerveSubsystem.getPose().getX(), swerveSubsystem.getPose().getY());
    double distance = targetPosition.distance(robotPosition);

    velocity = Util.findVelocity(distance);
    shooter.setShooterSpeed(Util.findVelocity(distance));
    IsActive = true;
  }
    
  public boolean isFinished() {
    if (IsActive){
      boolean result = MathUtil.isNear(velocity, shooter.getSpeed(), tolerance);
      if (result){
        IsActive = false;
      }
        
      return result;   
    }

    return false;
  }
}
