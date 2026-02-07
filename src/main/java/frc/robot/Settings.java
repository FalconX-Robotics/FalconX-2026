package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Settings {
  public final CommandXboxController driverController;
  public final CommandXboxController operatorController;

  public class DriverSettings {
    public final Trigger speedModeButton = new Trigger(()-> {
      return driverController.getRightTriggerAxis() > 0.5;
    });

    public final Trigger invertButton = driverController.start();
    public final boolean inverted = false;

    public double getLeftX() {
      double axis = MathUtil.applyDeadband(driverController.getLeftX(), 0.1);
      if (inverted) {
        axis = -axis;
      }

      return axis;
    }

    public double getLeftY() {
      double axis = MathUtil.applyDeadband(driverController.getLeftY(), 0.1);
      if (inverted) {
        axis = -axis;
      }
      
      return axis;
    }

    public double getRightX() {
      double axis = MathUtil.applyDeadband(driverController.getRightX(), 0.1);
      if (inverted) {
        axis = -axis;
      }
      
      return axis;
    }

    public double getRightY() {
      double axis = MathUtil.applyDeadband(driverController.getRightY(), 0.1);
      if (inverted) {
        axis = -axis;
      }
      
      return axis;
    }
  }

  /**
   * Controller bindings and such for controlling arm and arm adjacent parts (eg:intake and elevator)
   */
  public class OperatorSettings {
    // public Trigger coralIntakeButton       = operatorController.x();
    // // public Trigger armAngleButton          = operatorController.b();
    // public Trigger releaseButton    = operatorController.b();
    // public Trigger travelButton = operatorController.y();
    // public Trigger overrideArm = new Trigger(() -> {return Math.abs(operatorController.getRightY()) > 0.1;});

    // public Trigger climbButton = operatorController.leftBumper();
    // public Trigger unClimbButton = operatorController.rightBumper();

    // public Trigger moveToL2 = operatorController.povDown();
    // public Trigger moveToL3 = operatorController.povUp();
    // public Trigger moveToIntake = operatorController.a();
    // public Trigger moveToLoAlgae = operatorController.leftTrigger();
    // public Trigger moveToHiAlgae = operatorController.rightTrigger();

    // public final double intakeSpeed = -1/3.0;
    // public final double releaseSpeed = 0.5;

    // public double climbSpeed = 5;

    public double getLeftX() {
      return MathUtil.applyDeadband(operatorController.getLeftX(), 0.1);
    }

    public double getLeftY() {
      return MathUtil.applyDeadband(operatorController.getLeftY(), 0.1);
    }

    public double getRightX() {
      return MathUtil.applyDeadband(operatorController.getRightX(), 0.1);
    }

    public double getRightY() {
      return MathUtil.applyDeadband(operatorController.getRightY(), 0.1);
    }
  }

  public OperatorSettings operatorSettings;
  public DriverSettings driverSettings;

  public Settings(CommandXboxController driveController, CommandXboxController operatorController) {
    this.operatorController = operatorController;
    this.driverController = driveController;

    operatorSettings = new OperatorSettings();
    driverSettings = new DriverSettings();
  }
}
