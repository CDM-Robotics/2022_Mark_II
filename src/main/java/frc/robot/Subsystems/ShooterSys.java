// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LogitechDualAction;
import frc.robot.Constants.ShooterSystemConstants;

/** Add your docs here. */
public class ShooterSys implements Subsystem{

    private static ShooterSys mShooterSys; 

    private WPI_TalonSRX mVerticalAngleControl;
    private WPI_TalonSRX mHorizontalAngleControl;

    private WPI_TalonFX mShooterMaster; 
    private WPI_TalonFX mShooterSlave0; 

    private WPI_TalonSRX mSerializer; 

    private static final double ShooterSpin = 0.5; 

    public static ShooterSys getInstance() {

        if (mShooterSys == null) {

            mShooterSys = new ShooterSys();
        }

        return mShooterSys;
    }

    public ShooterSys() {

        mVerticalAngleControl = new WPI_TalonSRX(ShooterSystemConstants.verticalControlTalon); 
        mHorizontalAngleControl = new WPI_TalonSRX(ShooterSystemConstants.horizontalControlTalon);

        mShooterMaster = new WPI_TalonFX(ShooterSystemConstants.ShooterMaster); 
        mShooterSlave0 = new WPI_TalonFX(ShooterSystemConstants.ShooterSlave0); 

        config();

    }

    public void config() {

        mShooterMaster.setInverted(ShooterSystemConstants.ShooterMaster_isInverted);
        mShooterSlave0.setInverted(ShooterSystemConstants.ShooterSlave0_isInverted);

        mVerticalAngleControl.setInverted(ShooterSystemConstants.verticalInverted);
        mHorizontalAngleControl.setInverted(ShooterSystemConstants.horizontalInverted);

        mVerticalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.verticalEncoderInitial); 
        mHorizontalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.horizontalEncoderInitial);

        mShooterSlave0.follow(mShooterMaster);
    }

    public void shooterAimControl(LogitechDualAction mController) {

        double tVertical = mController.getRawAxis(3) / 5;
        double tHorizontal = mController.getZ() / 5; 

        mVerticalAngleControl.set(ControlMode.PercentOutput, tVertical);
        mHorizontalAngleControl.set(ControlMode.PercentOutput, tHorizontal);
    }


    public void runSerializer(boolean is_pressed) {

        double tSerializerSpeed = 0.5;

        mSerializer.set(ControlMode.PercentOutput, 0.5);
    }

    private double tSpinVal = 0.5;

    public void ShooterSpinUpToggle(boolean is_pressed) {

        if (is_pressed) {

            tSpinVal -= ShooterSpin; 
            tSpinVal = Math.abs(tSpinVal);
        }

        mShooterMaster.set(ControlMode.PercentOutput, tSpinVal);
    }

    
}
