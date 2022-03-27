// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberSystemConstants;

public class ClimberSys implements Subsystem {

  private static ClimberSys mClimberSys; 

  private Solenoid mLeftSolenoid; 
  private Solenoid mRightSolenoid; 

  private Solenoid mLeftSolenoida; 
  private Solenoid mRightSolenoida;

  private WPI_TalonSRX mLeftClimber; 
  private WPI_TalonSRX mRightClimber; 

  public static ClimberSys getInstance() {

    if (mClimberSys == null) {

      mClimberSys = new ClimberSys(); 
    }

    return mClimberSys; 
  }
  /** Creates a new ClimberSys. */
  public ClimberSys() {

    mLeftSolenoid = new Solenoid(60, PneumaticsModuleType.CTREPCM, ClimberSystemConstants.Left_solenoid_Channel);
    mRightSolenoid = new Solenoid(60, PneumaticsModuleType.CTREPCM, ClimberSystemConstants.Right_Solenoid_Channel);

    mLeftSolenoida = new Solenoid(60, PneumaticsModuleType.CTREPCM, ClimberSystemConstants.Left_solenoid_Channela);
    mRightSolenoida = new Solenoid(60, PneumaticsModuleType.CTREPCM, ClimberSystemConstants.Right_Solenoid_Channela);

    mRightClimber = new WPI_TalonSRX(ClimberSystemConstants.Right_Climber_Motorcontrol);
    mLeftClimber = new WPI_TalonSRX(ClimberSystemConstants.Left_Climber_Motorcontrol);

    config();
  }

  private void config() {

    mLeftSolenoid.set(false);
    mRightSolenoid.set(false);

    mLeftSolenoida.set(true);
    mRightSolenoida.set(true);

    mLeftClimber.setInverted(ClimberSystemConstants.Left_Climber_isInverted);
    mRightClimber.setInverted(ClimberSystemConstants.Right_Climber_isInverted);

    //mLeftClimber.follow(mRightClimber);
  }

  private boolean isDefaultPosition = false; 

  private boolean climbSequenceInProgress = false;


  private void extendPnematicArms() {

    mLeftSolenoid.toggle();
    mLeftSolenoida.toggle();

    mRightSolenoid.toggle();
    mRightSolenoida.toggle();

  }

  public void toggleLeft() {
    mLeftSolenoid.toggle();
    mLeftSolenoida.toggle();
  }

  public void toggleRight() {
    mRightSolenoid.toggle();
    mRightSolenoida.toggle();
  }

  boolean sideToggle = false; 
  public void ClimberRun(boolean leftup, boolean leftdown, boolean rightup, boolean rightdown) {

  if (rightup) {
    mRightClimber.set(ControlMode.PercentOutput, 1);
  } else if (rightdown) {
    mRightClimber.set(ControlMode.PercentOutput, -1);
  } else {
    mRightClimber.set(ControlMode.PercentOutput, 0);
  }

  if (leftup) {
    mLeftClimber.set(ControlMode.PercentOutput, 1);
  } else if (leftdown) {
    mLeftClimber.set(ControlMode.PercentOutput, -1);
  } else {
    mLeftClimber.set(ControlMode.PercentOutput, 0);
  }


  }

  int timeCounter = 0; 
  private void grabTheBar() {

    if (timeCounter < 200) {

      mRightClimber.set(ControlMode.PercentOutput, 0.40); 
      timeCounter++;

    } else if (timeCounter >= 200 && timeCounter < 300) {

      //DriveSys.getInstance().arcadeDrive(0.35, 0);

    } else if (timeCounter >= 300 && timeCounter < 500) {

      //DriveSys.getInstance().arcadeDrive(0, 0);
      mRightClimber.set(ControlMode.PercentOutput, -0.40);
    } else {

      timeCounter = 0; 
      climbSequenceInProgress = false;
      isDefaultPosition = false; 
    }

  }

  
}
