package frc.robot.commands;

import org.dyn4j.geometry.Vector2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

public class RotateToTarget extends Command{
  private final SwerveSubsystem swerveSubsystem;
  private final PIDController pidController;
  private final double MAX_SPEED = 2 * Math.PI;

  double currentAngle;
  double targetAngle;

  public RotateToTarget(RobotContainer robotContainer) {
    this.swerveSubsystem = robotContainer.subsystems.swerve;
    this.pidController = new PIDController(2, 0, 0);


    super.addRequirements(this.swerveSubsystem);
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
    pidController.setSetpoint(targetAngle - Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentAngle = swerveSubsystem.getYaw().getRadians();

    swerveSubsystem.drive(new ChassisSpeeds(0,0, MathUtil.clamp(pidController.calculate(currentAngle), -MAX_SPEED, MAX_SPEED) ));

    
      System.out.println("RotateToTarget");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("RotateToTarget ended");
  }

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
