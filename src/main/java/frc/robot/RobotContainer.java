// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.LocalDateTime;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.drivebase.ChangeSpeed;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.util.Util;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public static class Controllers {
    public final CommandXboxController driver = new CommandXboxController(0);   // Driver Controller Port is port 0
    public final CommandXboxController operator = new CommandXboxController(1); // Operator Controller Port is port 1
  }

  public static class Subsystems {
    public SwerveSubsystem swerve;
    public Shooter shooter;
  }
  
  public static class Commands {
    public Command driveInputs;
    public Command dhara;
  }

  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  
  /**
   * The controllers associated with controlling the robot.
   */
  public final Controllers controllers = new Controllers();

  /**
   * The settings (controller bindings) for the controllers.
   */
  public final Settings settings = new Settings(this.controllers.driver, this.controllers.operator);
  
  /**
   * All related subsystems with the robot.
   */
  public final Subsystems subsystems = new Subsystems();

  /**
   * All related commands with the robot.
   */
  public final Commands commands = new Commands();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());

    this.subsystems.swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    this.subsystems.shooter = new Shooter(this);

    final Command driveInputs = this.subsystems.swerve.driveInputs(
      () -> -this.settings.driverSettings.getLeftY(),
      () -> -this.settings.driverSettings.getLeftX(),
      () -> -this.settings.driverSettings.getRightX()
    );

    this.commands.driveInputs = new ParallelCommandGroup(new ChangeSpeed(this.subsystems.swerve), driveInputs);
    this.commands.dhara = new ParallelCommandGroup(driveInputs);

    this.subsystems.swerve.setupPathPlanner();

    this.autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);

    // Configure thse trigger bindings
    this.configureBindings();
  }

  /**
   * Robot periodic method. Runs at 50Hz (once every 20ms).
   */
  public void robotPeriodic() {
    SmartDashboard.putBoolean("inverted", this.settings.driverSettings.inverted);
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    this.settings.driverSettings.speedModeButton.whileTrue(this.commands.driveInputs); // settings.driverSettings.speedModeButton is a Trigger with the condition of RT value > 0.5
    this.subsystems.swerve.setDefaultCommand(this.commands.dhara);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return this.autoChooser.getSelected();
  }

  public void setDriveMode() {
    this.configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    this.subsystems.swerve.setMotorBrake(brake);
  }
}
