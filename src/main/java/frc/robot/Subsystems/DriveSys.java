// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.invoke.LambdaMetafactory;
import java.lang.reflect.Array;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.NavX;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Pathfinder.Path.PositionPoint;
//import frc.robot.LogitechDualAction;

/** Add your docs here. */
public class DriveSys implements Subsystem {

  private static DriveSys mDriveSys; 

  private DifferentialDrive mDifferentialDrive; 

  private WPI_TalonFX L_Master; 
  private WPI_TalonFX L_Slave;

  private WPI_TalonFX R_Master; 
  private WPI_TalonFX R_Slave; 
  

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

    configMotors();
  }

  public void configMotors() {

    L_Master.follow(L_Slave);
    R_Slave.follow(R_Master);

    L_Master.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);
    L_Slave.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);

    R_Master.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);
    R_Slave.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);

    R_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    L_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);



    mDifferentialDrive = new DifferentialDrive(L_Slave, R_Master);
    }

    /**
     * Input as percent out 
     * Number between 1 and -1
     */
    public void sideIndependentControl(double leftPercentOut, double rightPercentOut) {

      //Swap to which controller is needed:
      //Joystick Mode:
      //R_Master.set(ControlMode.PercentOutput, rightPercentOut/2.3);
      //L_Slave.set(ControlMode.PercentOutput, leftPercentOut/2.3);
      mDifferentialDrive.arcadeDrive(leftPercentOut, 0.0);
      //Dual Action Mode:
      //R_Master.set(ControlMode.PercentOutput, rightPercentOut/4.3);
      //L_Slave.set(ControlMode.PercentOutput, leftPercentOut/4.3);

    }

    double speed = 0.8; 
    boolean magIsPositive;
    boolean yawIsPositive; 

    public void arcadeDrive(double mag, double yaw, double speed) {
/* 
        double leftMag; 
        double rightMag; 
      
        //mLog.periodicPrint("mag: " + mag + "  yaw: " + yaw + "  maxPercentOut: " + speed, 25);
       
        speed = (((0.7+(speed * 0.3)) ) / 3);
        magIsPositive = (mag > 0);
        yawIsPositive = (yaw > 0); 
        
        yaw = 1 - Math.abs(yaw);
        
        if (yaw > 0.9) {// to drive straight forward or backward

            leftMag = mag * speed; 
            rightMag = mag * speed; 

        }else if ((yaw < 0.50) && (Math.abs(mag)< 0.35)) {// to turn in place 

            yaw = 1 - yaw; 

            if (yawIsPositive) {

                leftMag = yaw * speed / 1.4;
                rightMag = yaw * speed * -1 / 1.4;
            } else {

                leftMag = yaw * speed * -1 / 1.4;
                rightMag = yaw * speed / 1.4;
            }

        } else { // to turn and move. 
        
            mag = DriveSystemConstants.DRIVE_MIN_PERCENT_OUT + ((mag * mag) * (1 - DriveSystemConstants.DRIVE_MIN_PERCENT_OUT));
            
            if (magIsPositive) {

                if (yawIsPositive) {

                    leftMag = mag * speed; 
                    rightMag = mag * (0.5 + (yaw/2)) * speed; 
                }else{

                    rightMag = mag * speed; 
                    leftMag = mag * (0.5 + (yaw/2)) * speed; 
                }

            }else{

                if (yawIsPositive) {

                    leftMag = mag * speed * -1; 
                    rightMag = mag * (0.5 + (yaw/2)) * speed * -1;
                }else{

                    rightMag = mag * speed * -1; 
                    leftMag = mag * (0.5 + (yaw/2)) * speed * -1; 
                }
            }     
        }  

        L_Master.set(ControlMode.PercentOutput, leftMag);
        R_Master.set(ControlMode.PercentOutput, rightMag); */



        mDifferentialDrive.arcadeDrive((mag * (0.35 + (0.2 * speed))) , (yaw * (0.35 + (0.2 * speed))), false);
      
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

    public ArrayList <WPI_TalonFX> getMotorList() 
    {
      ArrayList <WPI_TalonFX> motorList = new ArrayList <WPI_TalonFX>();
      motorList.add(L_Master);
      motorList.add(L_Slave);
      motorList.add(R_Master);
      motorList.add(R_Slave);
      
      return motorList;
    }

}
