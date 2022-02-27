// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.io.ObjectInputFilter.Config;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.IntakeSysConstants;

/** Add your docs here. */
public class IntakeSys implements Subsystem{


    private static IntakeSys mIntakeSys; 

    private Solenoid mSol_1; 
    private Solenoid mSol_2; 
    private Solenoid mSol_3; 

    private WPI_TalonSRX mIntake; 

    private boolean isExtended = false; 


    public static IntakeSys getInstance() {

        if (mIntakeSys ==  null) {

            mIntakeSys = new IntakeSys();
        }

        return mIntakeSys; 
    }

    public IntakeSys() {

        mSol_1 = new Solenoid(0, PneumaticsModuleType.CTREPCM, IntakeSysConstants.Solenoid_1_channel); 
        mSol_2 = new Solenoid(0, PneumaticsModuleType.CTREPCM, IntakeSysConstants.Solenoid_2_channel);
        mSol_3 = new Solenoid(0, PneumaticsModuleType.CTREPCM, IntakeSysConstants.Solenoid_3_channel);

        mIntake = new WPI_TalonSRX(IntakeSysConstants.Intake); 

        config();        
    }

    private void config() {

        mIntake.setInverted(IntakeSysConstants.Intake_isInverted);

        mSol_1.set(true);
        mSol_2.set(true);
        mSol_3.set(false);
    }

    private int tIntakeTimer = 0; 

    public void IntakeOutIn(boolean wasPressed) {

        if (isExtended) {

            if (wasPressed) {
                retract();

                tIntakeTimer = 0; 
            }

            if (tIntakeTimer >= 0) {

                tIntakeTimer++;

                if (tIntakeTimer < 100) {

                    mSol_1.set(true);
                    mSol_2.set(true);
                    mSol_3.set(true);
                     
                } else if ((tIntakeTimer >= 100 )&& (tIntakeTimer < 200)) {

                    mSol_2.set(false);
                    mSol_3.set(false);

                } else {

                    mSol_3.set(true);
                    tIntakeTimer = -32; 
                }
            }

        } else if (!isExtended) {

            if (wasPressed) {
                extend();
            }
        }


    }

    private void extend() {

        mSol_1.set(true);
        mSol_2.set(true);
        mSol_3.set(true);

        isExtended  = true; 

    }

    private void retract() {

        mSol_1.set(false);
        mSol_2.set(true);
        mSol_3.set(true);

        isExtended = false; 
    }
}
