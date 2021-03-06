// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.NavX;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class PositionTrackerXYA {

    private static PositionTrackerXYA mPositionTrackerXYA;
    
    private PositionPoint pastXYA; 
    private PositionPoint presentXYA;
    private PositionPoint predictedXYA; 


    private double totalDistanceTraveled = 0; 

    public static PositionTrackerXYA getInstance() {

        if (mPositionTrackerXYA == null) {

            mPositionTrackerXYA = new PositionTrackerXYA(); 
        }

        return mPositionTrackerXYA;
    }

    public PositionTrackerXYA() {

        pastXYA = new PositionPoint(DriveSystemConstants.ROBOT_STARTING_POSITION_X, DriveSystemConstants.ROBOT_STARTING_POSITION_y, 
                                        DriveSystemConstants.ROBOT_STARTING_POSITION_ANGLE); 

        presentXYA =  new PositionPoint(DriveSystemConstants.ROBOT_STARTING_POSITION_X, DriveSystemConstants.ROBOT_STARTING_POSITION_y, 
                                        DriveSystemConstants.ROBOT_STARTING_POSITION_ANGLE); 
    }

    public PositionPoint getPreviousPosition() {
        return pastXYA; 
    }

    public PositionPoint getPresentPosition() {
        return presentXYA; 
    }

    public PositionPoint getPredictedPosition() {
        return predictedXYA; 
    }

    public void pushToSmartDashboard() {

        SmartDashboard.putNumber("X", presentXYA.xCoord); 
        SmartDashboard.putNumber("Y", presentXYA.yCoord); 
        SmartDashboard.putNumber("Heading", presentXYA.angleCumulative); 
    }

    /**
     * runs in periodic
     * Method generates a distance traveled relative to its previous 
     * position. Method sets the current position to the previous position
     * + the relative distance traveled at the current heading angle. 
     * Calculates the predicted position of the robot which represents an
     * estimate of where it will be after the next loop based on the current
     * and previous positions
     * 
     * @see
     * a Time Relative or tr variables are use to store values that represent 
     * the change between the previous and current positions
     */
    public void CalculatePositionData() {

        pastXYA = presentXYA; 

      double relativeDistanceTicksLeft = -DriveSys.getInstance().getLeftTicks() + pastXYA.leftTicks; 
      double relativeDistanceTicksRight = DriveSys.getInstance().getRightTicks() - pastXYA.rightTicks; 

      //double AngleChange = Math.abs(pastXYA.angleCumulative - NavX.getInstance());
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
    double angleTestable = (pastXYA.angleCumulative + AngleChange) % 360; 

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
      SmartDashboard.putNumber("Cumulotive angle previous", pastXYA.angleCumulative); 
      SmartDashboard.putNumber("angle relative", angleRelative);

      presentXYA.yCoord += Math.sin(angleRelative) * distanceTraveled * yMod; 
      presentXYA.xCoord += Math.cos(angleRelative) * distanceTraveled * xMod;

      presentXYA.leftTicks = DriveSys.getInstance().getLeftTicks(); 
      presentXYA.rightTicks = DriveSys.getInstance().getRightTicks();  

      totalDistanceTraveled += distanceTraveled; 

      presentXYA.angleCumulative += AngleChange; 

    }



    
}
