package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Feeder extends SubsystemBase {
  public final TalonFX feederMotor = new TalonFX(Constants.ID.FEEDER_ID);
  final RobotContainer robotContainer;
  final CommandXboxController operatorController;
  
  public Feeder(RobotContainer robotContainer){
    this.robotContainer = robotContainer;
    this.operatorController = robotContainer.controllers.operator;

    /**
     * motor configs
     */
    final TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotionMagic.MotionMagicAcceleration = 400;
    talonFXConfigs.MotionMagic.MotionMagicJerk = 4000; // jerk is change in acceleration over time (like acceleration is to velocity, and velocity is to position)

    final Slot1Configs slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = 0.25;
    slot1Configs.kV = 0.12;
    slot1Configs.kA = 0.01;
    slot1Configs.kP = 1.0;
    slot1Configs.kI = 0.0;
    slot1Configs.kD = 0.0;

    feederMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setFeederSpeed(double speed) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    feederMotor.setControl(request.withVelocity(speed));
  }

  public void setFeederAcceleration(double acceleration) {
    final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
    feederMotor.setControl(request.withAcceleration(acceleration));
  }

  public double getShooterSpeed() {
    return robotContainer.subsystems.shooter.getShooterSpeed();
  }

  public double getFeederSpeed() {
    return feederMotor.getVelocity().getValueAsDouble();
  }

  public double getFeederAcceleration() {
    return feederMotor.getAcceleration().getValueAsDouble();
  }
}
