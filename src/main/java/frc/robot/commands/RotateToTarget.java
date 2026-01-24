package frc.robot.commands;

import java.util.Optional;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Util;
import swervelib.SwerveDrive;

public class RotateToTarget extends Command{

    private final Vision vision;
    private final SwerveSubsystem swerveSubsystem;

    double currentAngle;
    double targetAngle;

    private boolean hasFoundTarget = false;

    public RotateToTarget(Vision vision, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.vision = vision;
        addRequirements(swerveSubsystem);
    }

    public void recalculateAngle() {
      // Optional<Transform3d> tagPose= vision.getTagPose(10);
      // if (tagPose.isPresent() == true) {
      // double targetX = tagPose.get().getX();
      // double targetY = tagPose.get().getY();
      Vector2 targetPosition = Util.getTargetPosition();
      double robotX = swerveSubsystem.getPose().getX();
      double robotY = swerveSubsystem.getPose().getY();

     targetAngle = Math.atan2(targetPosition.y-robotY, targetPosition.x-robotX);

    //  hasFoundTarget = true;

//  }
}


     // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recalculateAngle();





  }
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveSubsystem.getYaw().getRadians();


    // if (hasFoundTarget) {
      swerveSubsystem.drive(new ChassisSpeeds(0,0, currentAngle-targetAngle));
    // }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if (hasFoundTarget) {
      double difference = targetAngle-currentAngle;
      double targetAngleRange = Math.toRadians(2);

      boolean angleAtTarget = Math.abs(difference) < targetAngleRange;

      double rotationSpeed = Math.toDegrees(swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

      double targetSpeedRange = 5;
      boolean speedAtTarget = Math.abs(rotationSpeed) < targetSpeedRange;

      return angleAtTarget && speedAtTarget;
    // }  

  }
}