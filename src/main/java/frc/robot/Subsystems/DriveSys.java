// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.invoke.LambdaMetafactory;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.NavX;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class DriveSys implements Subsystem {

  private static DriveSys mDriveSys; 

  private WPI_TalonFX L_Master; 
  private WPI_TalonFX L_Slave;

  private WPI_TalonFX R_Master; 
  private WPI_TalonFX R_Slave; 

  private PositionPoint CurrentPositionXYA; 
  private PositionPoint PreviousPositionXYA;

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
    
    PreviousPositionXYA = new PositionPoint(DriveSystemConstants.ROBOT_STARTING_POSITION_X, DriveSystemConstants.ROBOT_STARTING_POSITION_y, 
                                        DriveSystemConstants.ROBOT_STARTING_POSITION_ANGLE); 

    CurrentPositionXYA =  new PositionPoint(DriveSystemConstants.ROBOT_STARTING_POSITION_X, DriveSystemConstants.ROBOT_STARTING_POSITION_y, 
                                        DriveSystemConstants.ROBOT_STARTING_POSITION_ANGLE); 

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

      
    }

    public double getLeftTicks() {

      return L_Master.getSensorCollection().getIntegratedSensorPosition();
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

    private void pushToSmartDashboard() {

      SmartDashboard.putNumber("x position: ", CurrentPositionXYA.xCoord);
      SmartDashboard.putNumber("y position: ", CurrentPositionXYA.yCoord);
      SmartDashboard.putNumber("total distance traveled: ", totalDistanceTraveled);

      SmartDashboard.putNumber("x position inches: ", CurrentPositionXYA.xCoord * DriveSystemConstants.TICK_TO_INCH_RATIO);
      SmartDashboard.putNumber("y position inches: ", CurrentPositionXYA.yCoord * DriveSystemConstants.TICK_TO_INCH_RATIO);
      SmartDashboard.putNumber("total distance traveled inches: ", totalDistanceTraveled * DriveSystemConstants.TICK_TO_INCH_RATIO);
      

      SmartDashboard.putNumber("right side ticks:  ", R_Master.getSensorCollection().getIntegratedSensorPosition()); 
      SmartDashboard.putNumber("left side ticks: ", -1 * L_Master.getSensorCollection().getIntegratedSensorPosition());
    }

    public PositionPoint getAbsolutePosition() {

      PreviousPositionXYA = CurrentPositionXYA; 

      double relativeDistanceTicksLeft = -L_Master.getSensorCollection().getIntegratedSensorPosition() + PreviousPositionXYA.leftTicks; 
      double relativeDistanceTicksRight = R_Master.getSensorCollection().getIntegratedSensorPosition() - PreviousPositionXYA.rightTicks; 

      //double angle = Math.abs(PreviousPositionXYA.angle - NavX.getInstance().getFusedHeading());
      double AngleChange = Math.abs((360 * (relativeDistanceTicksLeft - relativeDistanceTicksRight) * DriveSystemConstants.TICK_TO_INCH_RATIO) / (26.85 * 2 * Math.PI)); 

      if (relativeDistanceTicksRight > relativeDistanceTicksLeft) {
        AngleChange = AngleChange * -1; 
      }

      double AngleChangeAbsolute = Math.abs(AngleChange); 

      SmartDashboard.putNumber("angleChange", AngleChange); 
      SmartDashboard.putNumber("angleChangeAbsolute", AngleChangeAbsolute);
      

      //SmartDashboard.putNumber("relative distance inches right", relativeDistanceTicksRight); 
      //SmartDashboard.putNumber("relative distance inches left", relativeDistanceTicksLeft);

      //SmartDashboard.putNumber("previous ticks left", PreviousPositionXYA.leftTicks); 
      //SmartDashboard.putNumber("previous ticks right", PreviousPositionXYA.rightTicks); 

      //SmartDashboard.putNumber("radius left side ", relativeDistanceTicksLeft / (2 * Math.PI * (AngleChange / 360))); // radius of the circle generated by left side
      //SmartDashboard.putNumber("radius right side ", relativeDistanceTicksRight / (2 * Math.PI * (AngleChange / 360))); //radius for the right side circl 
      
      double distanceTraveled; 
      
      if (AngleChange != 0) {

      // Center radius is the average of the 2 radius 

      distanceTraveled = (Math.sin(AngleChangeAbsolute) * ((relativeDistanceTicksRight / (2 * Math.PI * (AngleChangeAbsolute / 360))) + (relativeDistanceTicksLeft / (2 * Math.PI * (AngleChangeAbsolute / 360)))) / 2) /
                                (Math.sin(0.5 * (180 - AngleChangeAbsolute))); 

      SmartDashboard.putNumber("distance traveled relative", distanceTraveled ); 

    }else{
      distanceTraveled = (relativeDistanceTicksLeft + relativeDistanceTicksRight) / 2; 
    }

    double angleRelative; 
    double angleTestable = (PreviousPositionXYA.angleCumulative + AngleChange) % 360; 

    int yMod; 
    int xMod; 

    if (angleTestable < 0) {

      angleTestable += 360; 
    }

    if ((90 <= angleTestable) && (angleTestable < 180)) {

      angleRelative = 180 - angleTestable; 
      xMod = -1; 
      yMod = 1; 
    } else if ((180 <= angleTestable) && (angleTestable < 270)) {

      angleRelative = angleTestable - 180; 
      xMod = -1; 
      yMod = -1; 
    } else if ((270 <= angleTestable) && (angleTestable < 360)) {

      angleRelative = 360 - angleTestable; 
      xMod = 1; 
      yMod = -1; 
    } else {

      angleRelative = angleTestable; 
      xMod = 1; 
      yMod = 1; 
    }
      SmartDashboard.putNumber("Cumulotive angle previous", PreviousPositionXYA.angleCumulative); 
      SmartDashboard.putNumber("angle relative", angleRelative);

      CurrentPositionXYA.yCoord += Math.sin(angleRelative) * distanceTraveled * yMod; 
      CurrentPositionXYA.xCoord += Math.cos(angleRelative) * distanceTraveled * xMod;

      CurrentPositionXYA.leftTicks = L_Master.getSensorCollection().getIntegratedSensorPosition(); 
      CurrentPositionXYA.rightTicks = R_Master.getSensorCollection().getIntegratedSensorPosition(); 

      totalDistanceTraveled += distanceTraveled; 

      CurrentPositionXYA.angleCumulative += AngleChange; 


      pushToSmartDashboard();

      return CurrentPositionXYA; 

    }
}
