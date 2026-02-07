package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Util;

public class RotateToTarget extends Command{
  private final SwerveSubsystem swerveSubsystem;

  double currentAngle;
  double targetAngle;

  public RotateToTarget(Vision vision, SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    addRequirements(swerveSubsystem);
  }

  public void recalculateAngle() {
    Vector2 targetPosition = Util.getTargetPosition();
    double robotX = swerveSubsystem.getPose().getX();
    double robotY = swerveSubsystem.getPose().getY();

    targetAngle = Math.atan2(targetPosition.y-robotY, targetPosition.x-robotX);
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

    swerveSubsystem.drive(new ChassisSpeeds(0,0, currentAngle - targetAngle));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double difference = targetAngle - currentAngle;
    double targetAngleRange = Math.toRadians(2);

    boolean angleAtTarget = Math.abs(difference) < targetAngleRange;

    double rotationSpeed = Math.toDegrees(swerveSubsystem.getRobotVelocity().omegaRadiansPerSecond);

    double targetSpeedRange = 5;
    boolean speedAtTarget = Math.abs(rotationSpeed) < targetSpeedRange;

    return angleAtTarget && speedAtTarget;
  }
}