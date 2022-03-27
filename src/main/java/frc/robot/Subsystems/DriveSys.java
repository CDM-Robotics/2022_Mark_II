// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.lang.invoke.LambdaMetafactory;
import java.lang.reflect.Field;

import javax.swing.text.DefaultEditorKit.PasteAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import org.opencv.core.Mat;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.NavX;
import frc.robot.PID_Controller;
import frc.robot.Constants.DriveSystemConstants;
import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class DriveSys implements Subsystem {

  private static DriveSys mDriveSys; 

  private DifferentialDrive mDifferentialDrive; 

  private static final double MaximumInchesPerSecond = (6380.0 * 5.95 * Math.PI) / (5.0 * 60.0); 
  private static final double TicksPer100ms_to_InchesPerSecond = 2.0 * 5.97 * Math.PI / 2048.0;

  private static final double Ticks_to_Meters = (5.95 * Math.PI * 0.0254) / (2048 * 5);

  private WPI_TalonFX L_Master; 
  private WPI_TalonFX L_Slave;

  private WPI_TalonFX R_Master; 
  private WPI_TalonFX R_Slave; 

  private PIDController leftSideController; 
  private PIDController rightSideController;

  private DifferentialDriveOdometry mDifferentialDriveOdometry; 
  private Pose2d mRobotPosition; 
  private Rotation2d mRotation2d; 

  private Field2d mField2d;


  private ShuffleboardTab PID_tuner; 
        private NetworkTableEntry P; 
        private NetworkTableEntry I;
        private NetworkTableEntry D; 

        private NetworkTableEntry Input; 

        private NetworkTableEntry LeftSideVelocity; 
        private NetworkTableEntry RightSideVelocity; 


  private ShuffleboardTab PositionTracking;
        private NetworkTableEntry Feild2d;


  
  

  public static DriveSys getInstance() {

    if (mDriveSys == null) {

      mDriveSys = new DriveSys(); 
    }
    return mDriveSys; 
  }

  public DriveSys() {

    L_Master = new WPI_TalonFX(DriveSystemConstants.LEFT_FALCON_MASTER);
    L_Slave = new WPI_TalonFX(DriveSystemConstants.LEFT_FALCON_SLAVE); 

    R_Master = new WPI_TalonFX(DriveSystemConstants.RIGHT_FALCON_MASTER); 
    R_Slave = new WPI_TalonFX(DriveSystemConstants.RIGHT_FALCON_SLAVE);

    //PID_tuner = Shuffleboard.getTab("PID Tuner"); 
    //PositionTracking = Shuffleboard.getTab("Position Tracker"); 

    leftSideController = new PIDController(0, 0, 0); 
    rightSideController = new PIDController(0, 0, 0);

    mField2d = new Field2d();

    configMotors();

  }

  public void configMotors() {

    L_Slave.follow(L_Master);
    R_Slave.follow(R_Master);

    L_Master.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);
    L_Slave.setInverted(DriveSystemConstants.LEFT_FALCON_MASTER_isInverted);

    L_Master.setSensorPhase(true);

    R_Master.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);
    R_Slave.setInverted(DriveSystemConstants.RIGHT_FALCON_MASTER_isInverted);

    R_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);
    L_Master.getSensorCollection().setIntegratedSensorPosition(0.0, 0);


    //P = PID_tuner.add(" P " , 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    //I = PID_tuner.add(" I " , 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    //D = PID_tuner.add(" D " , 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();
    //Input = PID_tuner.add("Velocity Input", 0).withWidget(BuiltInWidgets.kNumberSlider).getEntry();

    

    //RightSideVelocity = PID_tuner.add("Right Side Velocity", 0).withWidget(BuiltInWidgets.kGraph).getEntry();
    //LeftSideVelocity = PID_tuner.add("Left Side Velocity", 0).withWidget(BuiltInWidgets.kGraph).getEntry();

    mDifferentialDrive = new DifferentialDrive(L_Master, R_Master);

    posConfig(10, 10, 0);
    }

    public void posConfig(double x, double y, double h) {

      NavX.getInstance().setInitial(h);

      mRotation2d = Rotation2d.fromDegrees(h); 

      mRobotPosition = new Pose2d(x, y, mRotation2d);

      mDifferentialDriveOdometry = new DifferentialDriveOdometry(mRotation2d, mRobotPosition);

      mField2d.setRobotPose(mDifferentialDriveOdometry.getPoseMeters());

      
    }

    public double getLinearDistanceMeters() {

      return R_Master.getSelectedSensorPosition() * Ticks_to_Meters;
    }


    public void updatePos() {

      VisionSys.getInstance().getDistanceToTargetInches();

      double currentRightPositionMeters = R_Master.getSelectedSensorPosition() * Ticks_to_Meters;
      double currentLeftPositionMeters = L_Master.getSelectedSensorPosition() * Ticks_to_Meters;

      mRobotPosition = mDifferentialDriveOdometry.update(Rotation2d.fromDegrees(-NavX.getInstance().getYaw()), currentLeftPositionMeters, currentRightPositionMeters);

      mField2d.setRobotPose(mRobotPosition);

      SmartDashboard.putNumber("Left Position", currentLeftPositionMeters);
      SmartDashboard.putNumber("Right Position", currentRightPositionMeters);

      SmartDashboard.putData("Feild", mField2d);
    }


    /**
     * 
     * @param LeftVelocity desired left side velocity in inches per second 
     * @param RightVelocity desired right side velocity in inches per second
     */
    public void sideVelocityControl(double LeftVelocityInput, double RightVelocityInput) {

      leftSideController.setPID(P.getDouble(0), I.getDouble(0), D.getDouble(0));
      rightSideController.setPID(P.getDouble(0), I.getDouble(0), D.getDouble(0));

      double leftCurrentVelocity = L_Master.getSelectedSensorVelocity(0) * TicksPer100ms_to_InchesPerSecond;
      double rightCurrentVelocity = R_Master.getSelectedSensorVelocity(0) * TicksPer100ms_to_InchesPerSecond;

      RightSideVelocity.setDouble(rightCurrentVelocity);
      LeftSideVelocity.setDouble(leftCurrentVelocity);

      LeftVelocityInput = Input.getDouble(0); 
      RightVelocityInput = Input.getDouble(0);

      double leftCorrection = leftSideController.calculate(leftCurrentVelocity, LeftVelocityInput); 
      double rightCorrection = rightSideController.calculate(rightCurrentVelocity, RightVelocityInput);
    }

    /**
     * Input as percent out 
     * Number between 1 and -1
     */
    public void sideIndependentControl(double leftPercentOut, double rightPercentOut) {

      R_Master.set(ControlMode.PercentOutput, rightPercentOut);
      L_Master.set(ControlMode.PercentOutput, leftPercentOut);
    }

    public void Run_Falcon_Number_5_Not_Any_of_the_other_falcons_specificaly_and_exclusively_number_5(boolean isPressed) {

      if (isPressed) {

        //L_Master.set(ControlMode.PercentOutput, 0.5);

      } else {

        //L_Master.set(ControlMode.PercentOutput, 0.0);
      }
    }
    //double speed = 0.8; 
    boolean magIsPositive;
    boolean yawIsPositive; 

    public void arcadeDrive(double mag, double yaw, double speed) {
/* 
        double leftMag; 
        double rightMag; 
      
        //mLog.periodicPrint("mag: " + mag + "  yaw: " + yaw + "  maxPercentOut: " + speed, 25);
       
        speed = (((0.7+(speed * 0.3)) ) / 3);
        magIsPositive = (mag > 0);
        yawIsPositive = (yaw > 0); 
        
        yaw = 1 - Math.abs(yaw);
        
        if (yaw > 0.9) {// to drive straight forward or backward

            leftMag = mag * speed; 
            rightMag = mag * speed; 

        }else if ((yaw < 0.50) && (Math.abs(mag)< 0.35)) {// to turn in place 

            yaw = 1 - yaw; 

            if (yawIsPositive) {

                leftMag = yaw * speed / 1.4;
                rightMag = yaw * speed * -1 / 1.4;
            } else {

                leftMag = yaw * speed * -1 / 1.4;
                rightMag = yaw * speed / 1.4;
            }

        } else { // to turn and move. 
        
            mag = DriveSystemConstants.DRIVE_MIN_PERCENT_OUT + ((mag * mag) * (1 - DriveSystemConstants.DRIVE_MIN_PERCENT_OUT));
            
            if (magIsPositive) {

                if (yawIsPositive) {

                    leftMag = mag * speed; 
                    rightMag = mag * (0.5 + (yaw/2)) * speed; 
                }else{

                    rightMag = mag * speed; 
                    leftMag = mag * (0.5 + (yaw/2)) * speed; 
                }

            }else{

                if (yawIsPositive) {

                    leftMag = mag * speed * -1; 
                    rightMag = mag * (0.5 + (yaw/2)) * speed * -1;
                }else{

                    rightMag = mag * speed * -1; 
                    leftMag = mag * (0.5 + (yaw/2)) * speed * -1; 
                }
            }     
        }  

        L_Master.set(ControlMode.PercentOutput, leftMag);
        R_Master.set(ControlMode.PercentOutput, rightMag); */



        mDifferentialDrive.arcadeDrive((mag * (0.3 + (0.17 * speed))) , (yaw * (0.3 + (0.17 * speed))), false);

      
    }

    public double getLeftTicks() {

      return -L_Master.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getLeftVelocity() {

      return L_Master.getSensorCollection().getIntegratedSensorVelocity(); 
    }

    public double getRightTicks() {

      return R_Master.getSensorCollection().getIntegratedSensorPosition();
    }

    public double getRightVelocity() {

      return R_Master.getSensorCollection().getIntegratedSensorVelocity(); 
    }

    public double ticksToInches(double ticks) {

      //return ticks * 18.84956 / 2048;
      return ticks; 
    }

}
