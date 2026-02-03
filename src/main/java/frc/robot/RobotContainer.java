// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.time.LocalDateTime;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.swervedrive.drivebase.AbsoluteFieldDrive;
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
  public SendableChooser<Command> autoChooser = new SendableChooser<>();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  public final CommandXboxController operatorXbox = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerve;
  private final Settings settings = new Settings(driverXbox, operatorXbox);
  private final Shooter shooter;
  
  CvSink cvSink;
  CvSource camOutput;


  Thread camThread;


  
      AbsoluteFieldDrive absFieldDrive;
      Command zeroMotion;
      Command driveFieldOrientedDirectAngle;
      Command driveInputs;
      Command dhara;
      Command absoluteDrive;
      Command driveFieldOrientedAnglularVelocity;
      Command driveFieldOrientedDirectAngleSim;


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    swerve = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
    Util.setStartTime(LocalDateTime.now());
    DataLogManager.start(Filesystem.getOperatingDirectory() + "/logs", Util.getLogFilename());
  
    this.shooter = new Shooter(this);

    

    
   absFieldDrive = new AbsoluteFieldDrive(swerve,
  () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
  () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), 
  () -> Math.atan2(-settings.driverSettings.getRightX(), settings.driverSettings.getRightY()));

  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the desired angle NOT angular rotation
   zeroMotion = swerve.driveCommand(
    ()-> 0.0, ()->0.0, ()->0.0);
  
   driveFieldOrientedDirectAngle = swerve.driveCommand(
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
      () -> MathUtil.applyDeadband(-settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
      () -> -settings.driverSettings.getRightX(),
      () -> -settings.driverSettings.getRightY());

   driveInputs = new ParallelCommandGroup(new ChangeSpeed(swerve), swerve.driveInputs(()->-settings.driverSettings.getLeftY(), ()->-settings.driverSettings.getLeftX(), ()->-settings.driverSettings.getRightX()));
   dhara = new ParallelCommandGroup(swerve.driveInputs(()->-settings.driverSettings.getLeftY(), ()->-settings.driverSettings.getLeftX(), ()->-settings.driverSettings.getRightX()));
  
  // Applies deadbands and inverts controls because joysticks
  // are back-right positive while robot
  // controls are front-left positive
  // left stick controls translation
  // right stick controls the angular velocity of the robot
   absoluteDrive = new AbsoluteDrive(swerve, 
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY() * -1, OperatorConstants.LEFT_X_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX() * -1, OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getRightX(), OperatorConstants.RIGHT_X_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getRightY(), OperatorConstants.RIGHT_X_DEADBAND)
  );

   driveFieldOrientedAnglularVelocity = swerve.driveCommand(
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY() * -1, OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX() * -1, OperatorConstants.LEFT_X_DEADBAND),
    () -> settings.driverSettings.getRightX() * -1
  );

   driveFieldOrientedDirectAngleSim = swerve.simDriveCommand(
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
    () -> MathUtil.applyDeadband(settings.driverSettings.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
    () -> driverXbox.getRawAxis(2)
  );





    swerve.setupPathPlanner();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure thse trigger bindings
    configureBindings();






  }

  public void robotPeriodic() {
    SmartDashboard.putBoolean("inverted", settings.driverSettings.inverted);
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
     return autoChooser.getSelected();
    // return Commands.none();
   
  }

  public void setDriveMode() {
    configureBindings();
  }

  public void setMotorBrake(boolean brake) {
    swerve.setMotorBrake(brake);
  }
}


