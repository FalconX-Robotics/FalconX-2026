package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hootake extends SubsystemBase {
    
    public TalonFX feederRoller;
    public TalonFX intakeLauncherRoller;

    public Hootake() {
        feederRoller = new TalonFX(Constants.HootakeConstants.FEEDER_ROLLER_ID);
        intakeLauncherRoller = new TalonFX(Constants.HootakeConstants.FEEDER_ROLLER_ID);
        TalonFXConfiguration feederConfiguration = new TalonFXConfiguration();
        feederConfiguration.CurrentLimits.StatorCurrentLimit = Constants.HootakeConstants.CURRENT_LIMIT_ID;
    }

     

    public void setIntakeLauncherRoller (double voltage) {
        intakeLauncherRoller.setVoltage(voltage);
    }

    public void setFeederRoller (double voltage) {
        feederRoller.setVoltage(voltage);
    }

    public void stop() {
        feederRoller.set(0);
        intakeLauncherRoller.set(0);
    }

    @Override
    public void periodic() {
     // This method will be called once per scheduler run
  }
}
