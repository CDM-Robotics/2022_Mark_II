// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

/** Add your docs here. */
public class PathPoint {


    // x coord, y coord, heading(angle)
    private double x, y, h; 

    /**Time that the point occurs on in the path measured in 1/50 of a second*/
    private int time;
    /**Predicted velocity at current position and time */
    private double velocity; 

}
