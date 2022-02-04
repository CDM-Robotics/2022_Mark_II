// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Constants.PathfindingConstants;
import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class PositionTrackerXYA {

    private static PositionTrackerXYA mPositionTrackerXYA;
    
    private PositionPoint pastXYA; 
    private PositionPoint presentXYA;
    private PositionPoint predictedXYA; 

    public static PositionTrackerXYA getInstance() {

        if (mPositionTrackerXYA == null) {

            mPositionTrackerXYA = new PositionTrackerXYA(); 
        }

        return mPositionTrackerXYA;
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

        double trTickLeft = DriveSys.getInstance().getLeftTicks() - pastXYA.leftTicks; 
        double trTickRight = DriveSys.getInstance().getRightTicks() - pastXYA.rightTicks;

        double trAngle = Math.abs((360 * (trTickLeft - trTickRight) 
        * PathfindingConstants.TICK_TO_INCH_RATIO) / (PathfindingConstants.WHEELBASE * 2 * Math.PI)); 

    }



    
}
