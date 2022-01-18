// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

/** Add your docs here. */
public class PositionPoint {

    public double xCoord; 
    public double yCoord; 
    public double angleCumulative; 

    public double leftTicks = 0; 
    public double rightTicks = 0;
    

    public PositionPoint(double x, double y, double angle) {
        
        xCoord = x; 
        yCoord = y;
        
        this.angleCumulative = angle; 
    }
}
