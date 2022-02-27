// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.invoke.LambdaMetafactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.NavX;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class DriveSys implements Subsystem {

  private static DriveSys mDriveSys; 

  private DifferentialDrive mDifferentialDrive; 

  private WPI_TalonFX L_Master; 
  private WPI_TalonFX L_Slave;

  private WPI_TalonFX R_Master; 
  private WPI_TalonFX R_Slave; 

  private PositionPoint presentXYA; 
  private PositionPoint pastXYA;

  private double totalDistanceTraveled; 

  public static DriveSys getInstance() {

    if (mDriveSys == null) {

      mDriveSys = new DriveSys(); 
    }
    return mDriveSys; 
  }

  public DriveSys() {

    L_Master = new WPI_TalonFX(DriveSystemConstants.LEFT_FALCON_MASTER);
    L_Slave = new WPI_TalonFX(DriveSystemConstants.LEFT_FALCON_SLAVE); 

    R_Master = new WPI_TalonFX(DriveSystemConstants.RIGHT_FALCON_MASTER); 
    R_Slave = new WPI_TalonFX(DriveSystemConstants.RIGHT_FALCON_SLAVE);

    totalDistanceTraveled = 0; 

    configMotors();
  }

  public void configMotors() {

    L_Slave.follow(L_Master);
    R_Slave.follow(R_Master);

    L_Master.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);
    L_Slave.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);

    R_Master.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);
    R_Slave.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);

    R_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    L_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);

    mDifferentialDrive = new DifferentialDrive(L_Master, R_Master);
    }

    /**
     * Input as percent out 
     * Number between 1 and -1
     */
    public void sideIndependentControl(double leftPercentOut, double rightPercentOut) {

      R_Master.set(ControlMode.PercentOutput, rightPercentOut);
      L_Master.set(ControlMode.PercentOutput, leftPercentOut);
    }

    public void arcadeDrive(double mag, double yaw) {

        mDifferentialDrive.arcadeDrive(-mag/5, yaw/5);
      
    }

    public double getLeftTicks() {

      return -L_Master.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getLeftVelocity() {

      return L_Master.getSensorCollection().getIntegratedSensorVelocity(); 
    }

    public double getRightTicks() {

      return R_Master.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getRightVelocity() {

      return R_Master.getSensorCollection().getIntegratedSensorVelocity(); 
    }

    public double ticksToInches(double ticks) {

      //return ticks * 18.84956 / 2048;
      return ticks; 
    }

}
