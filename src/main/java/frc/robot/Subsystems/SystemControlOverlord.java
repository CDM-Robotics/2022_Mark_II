// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Pathfinder.Path.PositionActionPath;

/** Add your docs here. */
public class SystemControlOverlord implements Subsystem{

    private static SystemControlOverlord mControlOverlord; 

    private static PositionActionPath mPositionActionPath;

    public static SystemControlOverlord getInstance() {

        if (mControlOverlord == null) {

            mControlOverlord = new SystemControlOverlord(); 
        }

        return mControlOverlord;
    }

    public SystemControlOverlord() {

    }

    public PositionActionPath getPositionActionPath() {

        if (mPositionActionPath != null) {

            return mPositionActionPath;
        }



        return mPositionActionPath;
    }

    //- - - - - - - - Tracking - - - - - - - - 


    // - - Actions To Position Waypoints - - - 


    // - - - - - Obstacle Correction - - - - -


    // - - - - - Generate Trajectory - - - - -


    //- - - Create Action Path Overlay - - - -
    
}
