// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

/** Add your docs here. */
public class Objective {

    // x coord, y coord, heading(angle)
    private double x, y;

    private String actionList; 
    
    public Objective(double x, double y, String actionList) {

        this.x = x;
        this.y = y; 
        this.actionList = actionList; 
         
    }

    public double getX() {

        return x; 
    }

    public double getY() {

        return y; 
    }
    
    public void readActions() {

        for(int a = 0; a < actionList.length(); a++) {

            if (actionList.charAt(a) == 'a') {
                
            }
        }

    }

}
