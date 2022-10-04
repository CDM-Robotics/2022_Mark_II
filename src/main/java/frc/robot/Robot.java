// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Commands.ArcadeDriveCommand;
import frc.robot.Commands.ClimbCommand;
import frc.robot.Commands.IntakeControlCommand;
import frc.robot.Commands.ShooterControlCommand;
import frc.robot.Constants.ControlBoard;
import frc.robot.Pathfinder.Path.PositionPoint;
import frc.robot.Subsystems.ClimberSys;
import frc.robot.Subsystems.DriveSys;
import frc.robot.Subsystems.IntakeSys;
import frc.robot.Subsystems.PositionTrackerXYA;
import frc.robot.Subsystems.ShooterSys;
import frc.robot.Subsystems.ThermalReadingSys;
import frc.robot.Subsystems.VisionSys;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private CommandScheduler mScheduler; 

 
  
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

   mScheduler = CommandScheduler.getInstance();

   ControlBoard.getInstance(); 
   DriveSys.getInstance(); 
   ShooterSys.getInstance(); 
   IntakeSys.getInstance();
   VisionSys.getInstance(); 
   ClimberSys.getInstance();
   ThermalReadingSys.getInstance(DriveSys.getInstance().getMotorList());
  
  PositionTrackerXYA.getInstance().CalculatePositionData(); 
  PositionTrackerXYA.getInstance().pushToSmartDashboard();

   NavX.getInstance();
   
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

   ShooterSys.getInstance().ShooterSpinUpToggle(true, false);
   AutoCount = 0;
  }

  int AutoCount = 0; 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    ShooterSys.getInstance().ShooterSpinUpToggle(false, false);
  if (AutoCount < 125) {

    DriveSys.getInstance().sideIndependentControl(0.3, 0.3);

    if (AutoCount < 50) {

      ShooterSys.getInstance().turnAimShoot(true, false, false);
    }
    
  } else {

    DriveSys.getInstance().sideIndependentControl(0, 0);
  }
    AutoCount++; 

    if ((AutoCount < 200) && (AutoCount > 50)) {

      ShooterSys.getInstance().turnAimShoot(false, true, false);
    }

    if ((AutoCount < 750 ) && (AutoCount > 400)) {

      ShooterSys.getInstance().turnAimShoot(false, false, true);
    } 
  }


  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

    mScheduler.cancelAll();

    
    
    //ArcadeDriveCommand arcadeDriveCMD = new ArcadeDriveCommand(ControlBoard.getInstance().mDrivestick); 
    ArcadeDriveCommand arcadeDriveCMD = new ArcadeDriveCommand(ControlBoard.getInstance().mDrivestick); 

    ShooterControlCommand shootControlCMD = new ShooterControlCommand(ControlBoard.getInstance().mShootStick);

    IntakeControlCommand intakeControlCMD = new IntakeControlCommand(ControlBoard.getInstance().mDrivestick);

    ClimbCommand climbCMD = new ClimbCommand(ControlBoard.getInstance().mDrivestick, ControlBoard.getInstance().mShootStick);


    mScheduler.schedule(arcadeDriveCMD);
    mScheduler.schedule(shootControlCMD);
    mScheduler.schedule(intakeControlCMD);
    mScheduler.schedule(climbCMD);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    mScheduler.run();

    NavX.getInstance().pushToSmartDashboard();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
