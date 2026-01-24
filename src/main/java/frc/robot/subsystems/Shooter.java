package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase{
    TalonFX motor = new TalonFX(Constants.SHOOTERID);
    
    public void setShooterSpeed(double speed) {
       final MotionMagicVelocityVoltage request = new MotionMagicVelocityVoltage(0);
       motor.setControl(request.withVelocity(speed));
        

        
    }

    // public void periodic() {
    //    double voltageToSet = pidController.calculate(motor.get());
    // }

    public Shooter() {
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        Slot0Configs slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.25;
        slot0Configs.kV = 0.12;
        slot0Configs.kA = 0.01;
        slot0Configs.kP = 1.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;

        MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;

        // motionMagicConfigs.MotionMagicCruiseVelocity = 80;
        motionMagicConfigs.MotionMagicAcceleration = 400;
        motionMagicConfigs.MotionMagicJerk = 4000;

        motor.getConfigurator().apply(talonFXConfigs);
    }
}
