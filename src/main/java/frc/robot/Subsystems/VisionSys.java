// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.VisionCostants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/** Add your docs here. */
public class VisionSys implements Subsystem {

    public static VisionSys mLimeLight; 

    private NetworkTable mTable;

    NetworkTableEntry tx;// = mTable.getEntry("tx");
    NetworkTableEntry ty;// = mTable.getEntry("ty");
    NetworkTableEntry tv;// = mTable.getEntry("tv");

    public static VisionSys getInstance() {

        if (mLimeLight == null) {

            mLimeLight = new VisionSys();
        }

        return mLimeLight; 
    }

    public VisionSys() {

    mTable = NetworkTableInstance.getDefault().getTable("limelight");
        
    tx = mTable.getEntry("tx");
    ty = mTable.getEntry("ty");
    tv = mTable.getEntry("tv");

    }

    public double getX() {
    
        return tx.getDouble(0.0); 
    }

    public double getY() {

        return ty.getDouble(0.0); 
    }

    public double checkForValidTargets() {

        return tv.getDouble(0); 
    }

    public double getDistanceToTargetInches() {

        double angleToGoal = VisionCostants.limelightMountAngleDegrees + getY();
        angleToGoal *= Math.PI / 180.0;

        double distanceToTargetInches = (VisionCostants.GOAL_HEIGHT_INCHES - VisionCostants.LIMELIGHT_MOUNT_HEIGHT_INCHES) / Math.tan(angleToGoal);

        SmartDashboard.putNumber("Limelight Distance To Target", distanceToTargetInches);

        return distanceToTargetInches;
    }

    

    


    



}
