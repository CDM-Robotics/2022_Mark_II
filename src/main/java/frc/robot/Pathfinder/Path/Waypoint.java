// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

/** Add your docs here. */
public class Waypoint {

    private double x, y, a; 

    public Waypoint(double x, double y, double a) {

        this.x = x; 
        this.y = y; 
        this.a = a; 
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getA() {
        return a;
    }
    


}
