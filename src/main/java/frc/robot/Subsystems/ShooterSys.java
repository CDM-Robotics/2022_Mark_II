// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import org.opencv.core.Mat;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private DigitalInput IntakeSide; 
    private DigitalInput InverseSide; 

    private double ComputerControlStrength = 0; 
    private double xCorrection = 0; 
    private double yCorrection = 0; 
    private int isDefaulting = 0; 


    private static double ShooterSpin = 0.46;
    private static double CurrentSpin = 0.0; 

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

        mSerializer = new WPI_TalonSRX(ShooterSystemConstants.Serializer); 

        config();

    }

    public void config() {

        mShooterMaster.setInverted(ShooterSystemConstants.ShooterMaster_isInverted);
        mShooterSlave0.setInverted(ShooterSystemConstants.ShooterSlave0_isInverted);

        mVerticalAngleControl.setInverted(ShooterSystemConstants.verticalInverted);
        mHorizontalAngleControl.setInverted(ShooterSystemConstants.horizontalInverted);

        mVerticalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.verticalEncoderInitial); 
        mHorizontalAngleControl.setSelectedSensorPosition(ShooterSystemConstants.horizontalEncoderInitial);

        mVerticalAngleControl.getSensorCollection().setQuadraturePosition(0, 0);

        mShooterMaster.setSelectedSensorPosition(0.0); 
        mShooterSlave0.setSelectedSensorPosition(0.0);

        IntakeSide = new DigitalInput(ShooterSystemConstants.IntakeSideLimitNumber); 
        InverseSide = new DigitalInput(ShooterSystemConstants.InverseSideLimitNumber); 

        mShooterSlave0.follow(mShooterMaster);
    }

    
    public boolean defaultPosition(boolean wasPressed) {

        if (wasPressed) {

            isDefaulting -= 1; 
            isDefaulting = Math.abs(isDefaulting); 
        }

        

        double defaultVerticalCorrection =  + (0.9 * Math.abs((mVerticalAngleControl.getSensorCollection().getQuadraturePosition() / 90.0) * (mVerticalAngleControl.getSensorCollection().getQuadraturePosition() / 90.0)));

        double defaultHorrizontalCorrection = (Math.abs((mHorizontalAngleControl.getSelectedSensorPosition() / 3000)));
        double defaultHorrizontalCorrectionNegativeSide = 0.10 + (Math.abs((mHorizontalAngleControl.getSelectedSensorPosition() / 4000) * (mHorizontalAngleControl.getSelectedSensorPosition() / 4000)));

        if (defaultHorrizontalCorrection > 0.7) {
            defaultHorrizontalCorrection = 0.7;
        }

        if (defaultVerticalCorrection > 0.7) {
            defaultVerticalCorrection = 0.7;
        }
        SmartDashboard.putNumber("default y correction", defaultVerticalCorrection);
        SmartDashboard.putNumber("default x correction +", defaultHorrizontalCorrection);
        SmartDashboard.putNumber("default x correction -", defaultHorrizontalCorrectionNegativeSide);

        if (isDefaulting > 0) {

            if (mVerticalAngleControl.getSensorCollection().getQuadraturePosition() < -35) {

                mVerticalAngleControl.set(ControlMode.PercentOutput, defaultVerticalCorrection);
            } else { 
                mVerticalAngleControl.set(ControlMode.PercentOutput, 0);
            }


            if (mHorizontalAngleControl.getSelectedSensorPosition() >= 10) {

                mHorizontalAngleControl.set(ControlMode.PercentOutput, defaultHorrizontalCorrection);
            } else if (mHorizontalAngleControl.getSelectedSensorPosition() <= -10) {

                mHorizontalAngleControl.set(ControlMode.PercentOutput, -defaultHorrizontalCorrection);
            } else {

                mHorizontalAngleControl.set(ControlMode.PercentOutput, 0);
            }
            

        }

        if (((mHorizontalAngleControl.getSelectedSensorPosition() <= 15) && (mHorizontalAngleControl.getSelectedSensorPosition() >= -15)) && (mVerticalAngleControl.getSensorCollection().getQuadraturePosition() > -25)) {

            isDefaulting = 0; 
            return true;
        } else {

            return true; 
        }

        

    }


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
            mVerticalAngleControl.set(ControlMode.PercentOutput, 0);
        } 

        if (shoot) {

            mHorizontalAngleControl.set(ControlMode.PercentOutput, 0);
            runSerializer(true);
        }else {
            runSerializer(false);
        }


    }
    
    public void shooterAimControl(LogitechDualAction mController) {
        double tVertical; 
        double tHorizontal; 

        if (mVerticalAngleControl.getSelectedSensorPosition() < -450) {

            mHorizontalAngleControl.setInverted(true);
        } else { 
            mHorizontalAngleControl.setInverted(false);
        }

        if(mController.getRawButton(2)) {
            if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(0).intValue() != 3) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
            }
            
            xCorrection = VisionSys.getInstance().getX() / 27; 
            //yCorrection = VisionSys.getInstance().getY() / 20.5;
            ComputerControlStrength = 1; 
        } else {
            if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").getNumber(0).intValue() != 1) {
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
            }
            //xCorrection = 0.0;
            ComputerControlStrength = 0.0;
        }

        tHorizontal = (mController.getX() * ((1 + (-ComputerControlStrength)))) + ((xCorrection * ComputerControlStrength) * Math.abs(xCorrection * ComputerControlStrength));
        tVertical = (mController.getRawAxis(3) * ((1 + (-ComputerControlStrength)))) - ((yCorrection * ComputerControlStrength) * Math.abs(yCorrection * ComputerControlStrength));

        if ((mVerticalAngleControl.getSelectedSensorPosition() > -40) && (mController.getRawAxis(3) > 0.01)) {
            tVertical = 0; 
        }

        if ((mVerticalAngleControl.getSelectedSensorPosition() < -760) && (mController.getRawAxis(3) < -0.01)) {
            tVertical = 0; 
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

    private double tSpinVal = 0;

    private boolean is_spinning = false; 

    private double past = 0; 
    private double current; 
    public void ShooterSpinUpToggle(boolean is_pressed, boolean beef_isPressed, boolean beefiest_isPressed) {

        current = ((mShooterMaster.getSelectedSensorPosition() + mShooterSlave0.getSelectedSensorPosition()) / 2) - past;
        past = (mShooterMaster.getSelectedSensorPosition() + mShooterSlave0.getSelectedSensorPosition()) / 2;
        
        CurrentSpin = SmartDashboard.getNumber("Default ShooterSys Spin Speed", 0.46);
        //CurrentSpin = 0.46;

        if (beef_isPressed) {

            ShooterSpin = 0.66;
            CurrentSpin = ShooterSpin;
             
        } else {

            ShooterSpin = CurrentSpin; 
        }

        if (beefiest_isPressed) {

            ShooterSpin = 0.26;
            CurrentSpin = ShooterSpin; 
        } else {
            ShooterSpin = CurrentSpin;
        }

        if (is_pressed && !is_spinning) {

            is_spinning = true; 
 
        } else if (is_pressed && is_spinning) {

            tSpinVal = 0; 
            is_spinning = false; 
        }

        if (is_spinning && (tSpinVal < ShooterSpin)) {

            tSpinVal += 0.02; 
        }

        if(is_spinning && (tSpinVal > ShooterSpin)) {

            tSpinVal -= 0.02;
        }


        mShooterMaster.set(ControlMode.PercentOutput, tSpinVal);
    }

    public void pushToSmartDashboard() {

        //SmartDashboard.putNumber("Horizontal Encoder Position: ", mHorizontalAngleControl.getSelectedSensorPosition());
        //SmartDashboard.putNumber("Vertical Encoder Position: ", mVerticalAngleControl.getSensorCollection().getQuadraturePosition()); 
        //SmartDashboard.putNumber("X Correction: ", xCorrection); 
        //SmartDashboard.putNumber("Y Correction: ", yCorrection); 

        

        
    }

    public void checkLimits() {

        SmartDashboard.putBoolean("IntakeSideLimit", IntakeSide.get());
        SmartDashboard.putBoolean("InverseSideLimit", InverseSide.get());
        SmartDashboard.putNumber("Vel Shooter", current);
        SmartDashboard.putNumber("master", mShooterMaster.getSelectedSensorPosition()); 
        SmartDashboard.putNumber("slave", mShooterSlave0.getSelectedSensorPosition());

        
    }

    

    
}
