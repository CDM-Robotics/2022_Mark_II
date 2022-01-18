// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class NavX {

    private static AHRS ahrs; 

    public static AHRS getInstance() {

        if (ahrs == null) {
            ahrs = new AHRS(SPI.Port.kMXP); 
        } 

        return ahrs; 
    }

    public static void connectToClassMethods(AHRS toClass) {
         
        if (ahrs == null) {
            ahrs = toClass;  
        } 
    }

    public static void pushToSmartDashboard() {

        SmartDashboard.putBoolean(  "IMU_Connected",        ahrs.isConnected());
        SmartDashboard.putBoolean(  "IMU_IsCalibrating",    ahrs.isCalibrating());
        SmartDashboard.putNumber(   "IMU_Yaw",              ahrs.getYaw());
        SmartDashboard.putNumber(   "IMU_Pitch",            ahrs.getPitch());
        SmartDashboard.putNumber(   "IMU_Roll",             ahrs.getRoll());
    }

}
