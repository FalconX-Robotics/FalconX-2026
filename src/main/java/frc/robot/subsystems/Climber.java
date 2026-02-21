package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Climber extends SubsystemBase {
  public final TalonFX motor = new TalonFX(Constants.ID.CLIMBER_ID);
  CommandXboxController operatorController;
  DigitalInput climbUpLimitSwitchInput = new DigitalInput(Constants.ID.LIMIT_SWITCH_ID);

  public Climber() {
    operatorController = RobotContainer.getRobotContainer().controllers.operator;
  }

  public boolean ClimbUpDone() {
    final boolean result = climbUpLimitSwitchInput.get();

    SmartDashboard.putBoolean("climbUpLimitSwitchInput", result);
    return result;
  }
}
