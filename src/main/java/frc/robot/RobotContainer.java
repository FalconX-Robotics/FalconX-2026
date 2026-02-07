// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.LocalDateTime;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.GetToSpeed;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
import frc.robot.commands.swervedrive.drivebase.ChangeSpeed;
import frc.robot.subsystems.Climber;
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
  public SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXboxController = new CommandXboxController(0);
  public final CommandXboxController operatorXboxController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final Settings settings = new Settings(driverXboxController, operatorXboxController);
  private final Shooter shooter;
  private final Climber climber;
  

  Thread camThread;

  AbsoluteFieldDrive absFieldDrive;

  Command zeroMotion;
  Command driveFieldOrientedDirectAngle;
  Command driveInputs;
  Command dhara;
  Command absoluteDrive;
  Command driveFieldOrientedAnglularVelocity;
  Command driveFieldOrientedDirectAngleSim;
  Command climbDown;
  Command climbUp;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());
    
   
    this.shooter = new Shooter(this);
    this.climber = new Climber(this);
    this.climbUp = new ClimbUp(this.climber);
    this.climbDown = new ClimbDown(this.climber);
    
    absFieldDrive = new AbsoluteFieldDrive(swerve,
      () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
      () -> Math.atan2(-settings.driverSettings.getRightX(), settings.driverSettings.getRightY())
    );

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    zeroMotion = swerve.driveCommand(() -> 0.0, () -> 0.0, () -> 0.0);
  
    driveFieldOrientedDirectAngle = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -settings.driverSettings.getRightX(),
      () -> -settings.driverSettings.getRightY()
    );

    this.driveInputs = new ParallelCommandGroup(new ChangeSpeed(this.swerve), driveInputs);
    this.dhara = new ParallelCommandGroup(driveInputs);

    this.swerve.setupPathPlanner();

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
    settings.driverSettings.speedModeButton.whileTrue(driveInputs);
    swerve.setDefaultCommand(dhara);
    
    SequentialCommandGroup climberSequence= new SequentialCommandGroup(climbUp, new WaitCommand(3.5), climbDown);
    this.settings.operatorSettings.climbButton.onTrue(climberSequence);

    this.settings.operatorSettings.shooterButton.onTrue(new GetToSpeed(swerve, shooter));
    
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
    this.swerve.setMotorBrake(brake);
  }
}
