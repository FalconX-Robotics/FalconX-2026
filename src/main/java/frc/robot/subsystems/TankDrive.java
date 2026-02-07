package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TankDrive extends SubsystemBase {

    private TalonFX leftFront;
    private TalonFX rightFront;
    private TalonFX leftRear;
    private TalonFX rightRear;

    private DifferentialDrive drive;

    public TankDrive() {
  
    leftFront = new TalonFX(Constants.DriveTrainConstants.LEFT_FRONT_ID);
    rightFront = new TalonFX(Constants.DriveTrainConstants.RIGHT_FRONT_ID);
    leftRear = new TalonFX(Constants.DriveTrainConstants.LEFT_REAR_ID);
    rightRear = new TalonFX(Constants.DriveTrainConstants.RIGHT_REAR_ID);

    DifferentialDrive.curvatureDriveIK(0, 0, false);
  }




}
