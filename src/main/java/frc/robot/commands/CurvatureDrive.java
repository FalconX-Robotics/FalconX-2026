package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TankDrive;

public class CurvatureDrive extends Command {

    TankDrive driveSubsystem;
    CommandXboxController controller;
    
    public CurvatureDrive (TankDrive driveSystem, CommandXboxController driveController) {
        addRequirements(driveSystem);
        driveSystem = driveSystem;
        controller = driveController;
        
    }
}
