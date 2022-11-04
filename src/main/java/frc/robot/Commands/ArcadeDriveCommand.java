// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechDualAction;
import frc.robot.LogitechJoystick;
import frc.robot.Subsystems.DriveSys;

public class ArcadeDriveCommand extends CommandBase {

  // Switch Between Dual Action and Joystick 
  //LogitechDualAction mStick; 
  LogitechJoystick mStick; 
  
/** Creates a new ArcadeDriveCommand. */

  //Switch between Joytick and Dual Action
  
  public ArcadeDriveCommand(LogitechJoystick tStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSys.getInstance());

    this.mStick = tStick; 
  }
  /*
  public ArcadeDriveCommand(LogitechDualAction tStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSys.getInstance());

    this.mStick = tStick; 
  }
  */


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    DriveSys.getInstance(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    DriveSys.getInstance().arcadeDrive(-mStick.getY(), mStick.getX(), (mStick.getRawAxis(3) - 1) / -2);

    SmartDashboard.putNumber("speed Val", ((mStick.getRawAxis(3) - 1) / -2)); 

    //DriveSys.getInstance().sideIndependentControl(-mStick.getY(), -mStick.getRawAxis(3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
