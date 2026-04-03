package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class CheckInputs extends Command {
    private final Climber climberSubsystem;

    public CheckInputs(RobotContainer robotContainer) {
        this.climberSubsystem = robotContainer.subsystems.climber;
    }

    @Override
    public void execute() {
        this.climberSubsystem.printInputStates();
    }
    
    @Override
    public boolean isFinished() {
        return true;
    }
}
