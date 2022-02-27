// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import frc.robot.LogitechDualAction;
import frc.robot.LogitechJoystick;

/** Add your docs here. */
public class ControlBoard {

    public static final int drivePort = 0;
    public static final int shootPort = 1;

    public LogitechJoystick mDrivestick; 
    public LogitechDualAction mShootStick; 

    private static ControlBoard mControlBoard; 


    public static ControlBoard getInstance() {

        if (mControlBoard == null) {

            mControlBoard = new ControlBoard(); 
        }
        return mControlBoard; 
    }

    public ControlBoard() {

        mDrivestick = new LogitechJoystick(drivePort);

        mShootStick = new LogitechDualAction(shootPort);
    }


}
