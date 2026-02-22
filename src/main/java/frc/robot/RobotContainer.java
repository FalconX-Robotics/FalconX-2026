// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.LocalDateTime;
import java.util.Optional;

import org.photonvision.PhotonCamera;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.AutoShoot;
import frc.robot.commands.GetToSpeed;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualShoot;
import frc.robot.commands.RotateToTarget;
import frc.robot.commands.swervedrive.drivebase.ChangeSpeed;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Vision;
import frc.robot.util.Util;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public SendableChooser<Command> autoChooser = new SendableChooser<Command>();
  private static Optional<RobotContainer> robotContainer = null; // singleton

  public static class Controllers {
    /**
     * Driver controller object. The driver controller is connected on port 0.
     * 
     * <p>Use to drive the swerve drive.
     */
    public final CommandXboxController driver = new CommandXboxController(0);

    /**
     * Operator controller object. The operator controller is connected on port 1.
     * 
     * <p>Use to control all robot functions that don't involve swerve.
     */
    public final CommandXboxController operator = new CommandXboxController(1);
  }

  public static class Subsystems {
    public SwerveSubsystem swerve;
    public Shooter shooter;
    public Climber climber;
    public Feeder feeder;
    public Vision vision;
  }

  public static class Commands {
    public ParallelCommandGroup driveInputs;
    public ParallelCommandGroup dhara;

    public ClimbDown climbDown;
    public ClimbUp climbUp;
    public GetToSpeed getToSpeed;
    public AutoShoot autoShoot;
    public Intake intake;
    public ManualShoot manualShoot;
    public RotateToTarget rotateToTarget;
  }

  public final Controllers controllers = new Controllers();
  public final Subsystems subsystems = new Subsystems();
  public final Commands commands = new Commands();

  public final Settings settings = new Settings(this.controllers.driver, this.controllers.operator);

  public final PhotonCamera visionCamera = new PhotonCamera("Limelight");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    if (RobotContainer.robotContainer.isEmpty()) {
      RobotContainer.robotContainer = Optional.of(this);
    }

    // Initialize the logging module
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());

    // Initialize subsystems
    this.subsystems.swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    this.subsystems.shooter = new Shooter();
    this.subsystems.climber = new Climber();
    this.subsystems.feeder = new Feeder();
    this.subsystems.vision = new Vision();

    // Initialize commands
    this.commands.climbDown = new ClimbDown();
    this.commands.climbUp = new ClimbUp();
    this.commands.getToSpeed = new GetToSpeed();
    this.commands.autoShoot = new AutoShoot();
    this.commands.intake = new Intake();
    this.commands.manualShoot = new ManualShoot();
    this.commands.rotateToTarget = new RotateToTarget();

    this.commands.driveInputs = new ParallelCommandGroup(new ChangeSpeed(this.subsystems.swerve), this.subsystems.swerve.driveInputs(
      () -> -this.settings.driverSettings.getLeftY(),
      () -> -this.settings.driverSettings.getLeftX(),
      () -> -this.settings.driverSettings.getRightX()
    ));
    
    this.commands.dhara = new ParallelCommandGroup(this.subsystems.swerve.driveInputs(
      () -> -this.settings.driverSettings.getLeftY(),
      () -> -this.settings.driverSettings.getLeftX(),
      () -> -this.settings.driverSettings.getRightX()
    ));

    this.subsystems.swerve.setupPathPlanner();
    NamedCommands.registerCommand("rotateToTarget", this.commands.rotateToTarget);
    NamedCommands.registerCommand("getToSpeed", this.commands.getToSpeed);
    NamedCommands.registerCommand("intake", this.commands.intake);
    NamedCommands.registerCommand("autoShoot", this.commands.autoShoot);
    
    if (DriverStation.isAutonomous()) {
      this.autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", this.autoChooser);
    }

    // Configure trigger bindings
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
    this.settings.driverSettings.speedModeButton.whileTrue(this.commands.driveInputs);
    this.subsystems.swerve.setDefaultCommand(this.commands.dhara);

    this.settings.operatorSettings.climbUpButton.whileTrue(this.commands.climbUp);
    this.settings.operatorSettings.climbDownButton.whileTrue(this.commands.climbDown);

    //MANUAL SHOOTING

    //gets shooter to speed you want:
    this.settings.operatorSettings.feederButton.whileTrue(this.commands.intake);
    
    //fires:
    this.settings.operatorSettings.feederButton.and(this.settings.operatorSettings.shooterButton).whileTrue(this.commands.autoShoot);

    //SHOOTING AUTO
    
    this.settings.operatorSettings.shootingAutoButton.onTrue(new PathPlannerAuto("Shooting Auto"));
    
    // this.settings.operatorSettings.shootingAutoButton.onTrue(this.subsystems.swerve.driveToPose(new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(0))).andThen(new PathPlannerAuto("Shooting Auto")));
  }
    
  public static RobotContainer getRobotContainer() {
    // Optional<T>#orElse(T) returns the value held by Optional<T> if it exists, or the argument T if it doesn't.
    return RobotContainer.robotContainer.orElse(null);
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
