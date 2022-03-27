// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.LogitechDualAction;
import frc.robot.PID_Controller;
import frc.robot.Constants.ShooterSystemConstants;


/** Add your docs here. */
public class ShooterSys implements Subsystem{

    private static ShooterSys mShooterSys; 

    


    //private WPI_TalonSRX mVerticalAngleControl;
    private WPI_TalonSRX mHorizontalAngleControl;

    private WPI_TalonFX mShooterMaster; 
    private WPI_TalonFX mShooterSlave0; 

    private WPI_TalonSRX mSerializer; 

    private DigitalInput IntakeSide; 
    private DigitalInput InverseSide; 

    private PID_Controller mDefaultFrontSideVelocityController;
    private PID_Controller mInverseFrontSideVelocityController;

    private PID_Controller mVisionCorrection; 

    ShuffleboardTab shooterTuningShuffleboardTab;
    NetworkTableEntry ShooterSpeed;


    public static ShooterSys getInstance() {

        if (mShooterSys == null) {

            mShooterSys = new ShooterSys();
        }

        return mShooterSys;
    }

    public ShooterSys() {

        //mVerticalAngleControl = new WPI_TalonSRX(ShooterSystemConstants.verticalControlTalon); 
        mHorizontalAngleControl = new WPI_TalonSRX(ShooterSystemConstants.horizontalControlTalon);

        mShooterMaster = new WPI_TalonFX(ShooterSystemConstants.ShooterMaster); 
        mShooterSlave0 = new WPI_TalonFX(ShooterSystemConstants.ShooterSlave0); 

        mSerializer = new WPI_TalonSRX(ShooterSystemConstants.Serializer); 



        mDefaultFrontSideVelocityController = new PID_Controller(2, 0, 0.00, 2500);

        mInverseFrontSideVelocityController = new PID_Controller(2, 0.00, 0.1, 2500);

        mVisionCorrection = new PID_Controller(0.6, 0, 0.0, 30); 
        config();

        shooterTuningShuffleboardTab = Shuffleboard.getTab("Shooter Tuning"); 

        ShooterSpeed = shooterTuningShuffleboardTab.add("Speed", 0).getEntry();


    }

    public void config() {

        mShooterMaster.setInverted(ShooterSystemConstants.ShooterMaster_isInverted);
        mShooterSlave0.setInverted(ShooterSystemConstants.ShooterSlave0_isInverted);

        //mVerticalAngleControl.setInverted(ShooterSystemConstants.verticalInverted);
        mHorizontalAngleControl.setInverted(ShooterSystemConstants.horizontalInverted);

        //mVerticalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.verticalEncoderInitial); 
        mHorizontalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.horizontalEncoderInitial);

        mShooterMaster.configVoltageCompSaturation(10); 
        mShooterMaster.enableVoltageCompensation(true);

        mShooterSlave0.configVoltageCompSaturation(10);
        mShooterSlave0.enableVoltageCompensation(true); 

        //mVerticalAngleControl.getSensorCollection().setQuadraturePosition(0, 0);

        mShooterMaster.setSelectedSensorPosition(0.0); 
        mShooterSlave0.setSelectedSensorPosition(0.0);

        mShooterSlave0.setSensorPhase(true);

        IntakeSide = new DigitalInput(ShooterSystemConstants.IntakeSideLimitNumber); 
        InverseSide = new DigitalInput(ShooterSystemConstants.InverseSideLimitNumber); 

        
        //mShooterSlave0.follow(mShooterMaster);
    }

    private double ComputerControlStrength = 0; 
    private double xCorrection = 0; 
    private double yCorrection = 0; 

    public void turnAimShoot(boolean turn, boolean aim, boolean shoot) {

        if (turn) {
            mHorizontalAngleControl.set(ControlMode.PercentOutput, 1);
        } else { 
            mHorizontalAngleControl.set(ControlMode.PercentOutput, 0);
        }

        if (aim) {

            xCorrection = VisionSys.getInstance().getX() / 27; 
            yCorrection = VisionSys.getInstance().getY() / 20.5;

            mHorizontalAngleControl.set(ControlMode.PercentOutput, xCorrection);
            //mVerticalAngleControl.set(ControlMode.PercentOutput, 0);
        } 

        if (shoot) {

            mHorizontalAngleControl.set(ControlMode.PercentOutput, 0);
            runSerializer(true);
        }else {
            runSerializer(false);
        }

    }

    public void shooterAimControl(LogitechDualAction mController) {

        // if (mVerticalAngleControl.getSelectedSensorPosition() < -450) {

        //     mHorizontalAngleControl.setInverted(true);
        // } else { 
        //     mHorizontalAngleControl.setInverted(false);
        // }

        xCorrection = VisionSys.getInstance().getX() / 27; 
        yCorrection = VisionSys.getInstance().getY() / 20.5;

        xCorrection = xCorrection; 
        //yCorrection = -yCorrection;

        double tVertical; 
        double tHorizontal;

        yCorrection = 0;

        if (mController.getRawButton(2)) {

            tHorizontal = -mVisionCorrection.calc(0, xCorrection);
            tVertical = mController.getRawAxis(3);
        } else {

            tHorizontal = mController.getX();
            tVertical = mController.getRawAxis(3); 
        }

        if ((mController.getRawAxis(3) < -0.01) && !IntakeSide.get())  {

            tVertical = 0; 
        }

        if ((mController.getRawAxis(3) > 0.01) && !InverseSide.get())  {

            tVertical = 0; 
        }
        
        //mVerticalAngleControl.set(ControlMode.PercentOutput, tVertical);
        mHorizontalAngleControl.set(ControlMode.PercentOutput, tHorizontal);

        checkLimits();
 
    }


    public void runSerializer(boolean is_pressed) {

        if (is_pressed) {

            mSerializer.set(ControlMode.PercentOutput, 0.75);
        } else {

            mSerializer.set(ControlMode.PercentOutput, 0);
        }
        
    }

    private double tDesiredSpinVal = 0;

    private boolean is_spinning = false; 

    private double past = 0; 
    private double current; 
    


    public void ShooterSpinUpToggle(boolean is_pressed, boolean beef_isPressed) {

        //mDefaultFrontSideVelocityController.assignValues(P.getDouble(0), I.getDouble(0), D.getDouble(0));

        double tMasterCorrection;
        double tSlaveCorrection;

        if ((is_spinning) && (is_pressed)) {

            tDesiredSpinVal = 0; 
            is_spinning = false;

        } else if ((is_spinning == false) && (is_pressed)) {

            tDesiredSpinVal = 11000;  
            is_spinning = true;  
        }

        // if (is_spinning) {
        //     tDesiredSpinVal = ShooterSpeed.getDouble(0) * 21777;
        // }

        double tMasterCurrentVelocity = mShooterMaster.getSelectedSensorVelocity(0); 
        double tSlaveCurrentVelocity = mShooterSlave0.getSelectedSensorVelocity(0);

        tMasterCorrection = mDefaultFrontSideVelocityController.calc(0.69 * tDesiredSpinVal, tMasterCurrentVelocity);
        tSlaveCorrection = mInverseFrontSideVelocityController.calc(tDesiredSpinVal, tSlaveCurrentVelocity);

        SmartDashboard.putNumber("shooter master velocity", tMasterCurrentVelocity);
        SmartDashboard.putNumber("shooter Slave Velocity", tSlaveCurrentVelocity);
        SmartDashboard.putNumber("Desired Spin Value", tDesiredSpinVal);

        mShooterMaster.set(ControlMode.PercentOutput, tMasterCorrection/21777);
        mShooterSlave0.set(ControlMode.PercentOutput, tSlaveCorrection/21777);
        
        SmartDashboard.putNumber("Error", mInverseFrontSideVelocityController.returnError()); 

        /* current = ((mShooterMaster.getSelectedSensorPosition() + mShooterSlave0.getSelectedSensorPosition()) / 2) - past;
        past = (mShooterMaster.getSelectedSensorPosition() + mShooterSlave0.getSelectedSensorPosition()) / 2;

        if (beef_isPressed) {

            ShooterSpin = 0.90; 
        } else {

            ShooterSpin = 0.56;
        }

        if (is_pressed && !is_spinning) {

            is_spinning = true; 
 
        } else if (is_pressed && is_spinning) {

            tDesiredSpinVal = 0; 
            is_spinning = false; 
        }

        if (is_spinning && (tDesiredSpinVal < ShooterSpin)) {

            tDesiredSpinVal += 0.02; 
        }

        if(is_spinning && (tDesiredSpinVal > ShooterSpin)) {

            tDesiredSpinVal -= 0.02;
        }


        mShooterMaster.set(ControlMode.PercentOutput, tDesiredSpinVal); */
    }

    public void pushToSmartDashboard() {

        //SmartDashboard.putNumber("Horizontal Encoder Position: ", mHorizontalAngleControl.getSelectedSensorPosition());
        //SmartDashboard.putNumber("Vertical Encoder Position: ", mVerticalAngleControl.getSensorCollection().getQuadraturePosition()); 
        //SmartDashboard.putNumber("X Correction: ", xCorrection); 
        //SmartDashboard.putNumber("Y Correction: ", yCorrection); 

        

        
    }

    public void checkLimits() {

        //SmartDashboard.putBoolean("IntakeSideLimit", IntakeSide.get());
        //SmartDashboard.putBoolean("InverseSideLimit", InverseSide.get());
        //SmartDashboard.putNumber("Vel Shooter", current);
        //SmartDashboard.putNumber("master", mShooterMaster.getSelectedSensorPosition()); 
        //SmartDashboard.putNumber("slave", mShooterSlave0.getSelectedSensorPosition());

        
    }

    

    
}
