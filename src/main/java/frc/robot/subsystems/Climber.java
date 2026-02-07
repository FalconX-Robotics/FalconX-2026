package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Climber extends SubsystemBase{
    public final TalonFX motor = new TalonFX(Constants.ID.ARM_ID);
    RobotContainer robotContainer;
    CommandXboxController operatorController;
    final double maxRotations = 2.0;
    double rotationsDone = 0.0;
    DigitalInput climbDownLimitSwitchInput = new DigitalInput(Constants.ID.LIMIT_SWITCH_ID);

    public static Climber instance;
    


    public Climber(RobotContainer robotContainer) {
        Climber.instance = this;
        
        this.robotContainer = robotContainer;

        operatorController = robotContainer.operatorXboxController;

    }

    @Override
    public void periodic() {
        
    }

  

    public boolean IsAtBottom() {
        return climbDownLimitSwitchInput.get();
    }

    
}
