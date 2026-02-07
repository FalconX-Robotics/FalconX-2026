package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.GetToSpeed;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class Shooter extends SubsystemBase {
  final TalonFX mainMotor = new TalonFX(Constants.ID.INTAKE_SHOOTER_ID);
  final TalonFX feederMotor = new TalonFX(Constants.ID.FEEDER_ID);
  final SwerveSubsystem swerveSubsystem;
  final RobotContainer robotContainer;
  final CommandXboxController operatorController; 

  public Shooter(RobotContainer robotContainer) {
    this.robotContainer = robotContainer;
    this.swerveSubsystem = robotContainer.swerve;

    operatorController = robotContainer.operatorXboxController;

    final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = 400;
    talonFXConfigs.MotionMagic.MotionMagicJerk = 4000; // jerk is change in acceleration over time (like acceleration is to velocity, and velocity is to position)

    final Slot0Configs slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25;
    slot0Configs.kV = 0.12;
    slot0Configs.kA = 0.01;
    slot0Configs.kP = 1.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;

    mainMotor.getConfigurator().apply(talonFXConfigs);
  }
  
  public void setShooterSpeed(double speed) {
    speed /= Constants.SHOOTER_WHEEL_RADIUS;
    speed *= Constants.SHOOTER_GEAR_RATIO;
    speed /= 2 * Math.PI;
     
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    mainMotor.setControl(request.withVelocity(speed));      
  }

  public void setFeederSpeed(double speed) {
    feederMotor.set(speed);
  }


  public double getSpeed() {
    double velocity = mainMotor.getVelocity().getValueAsDouble() * (2.0 * Math.PI) / Constants.SHOOTER_GEAR_RATIO; // velocity of wheels

    return velocity * Constants.SHOOTER_WHEEL_RADIUS; // velocity at which objects comes out
  }
}