package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;

@Logged
public class Climber extends SubsystemBase {
  public final TalonFX motor = new TalonFX(Constants.ID.CLIMBER_ID);
  CommandXboxController operatorController;
  DigitalInput climbUpLimitSwitchInput = new DigitalInput(Constants.ID.LIMIT_SWITCH_ID);

  private boolean previousState = false;

  public Climber(RobotContainer robotContainer) {
    operatorController = robotContainer.controllers.operator;
  }

  // Put into variable then log/return current state (triggered or untriggered)
  public boolean ClimbUpDone() {
    boolean currentState = climbUpLimitSwitchInput.get();

    // Logs whether the limit switch was triggered or not 
    if (previousState != currentState) {
      previousState = currentState; // Makes previousState the currentState
    }
    
    SmartDashboard.putBoolean("Triggered", currentState);
    return currentState;
  }
}
