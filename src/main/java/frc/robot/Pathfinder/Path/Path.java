// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

import java.util.ArrayList;

/** Add your docs here. */
public class Path {

    private static ArrayList<PathPoint> mPath; 


    public static ArrayList<PathPoint> getInstance() {

        if (mPath == null) {

            mPath = new ArrayList<PathPoint>();
        }

        return mPath; 
    }

    

}
