// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechDualAction;
import frc.robot.LogitechJoystick;
import frc.robot.Subsystems.ClimberSys;

public class ClimbCommand extends CommandBase {

  LogitechJoystick mStick5; 
  LogitechDualAction mAction; 
  /** Creates a new ClimbCommand. */
  public ClimbCommand(LogitechJoystick Stick5, LogitechDualAction action) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ClimberSys.getInstance());

    mStick5 = Stick5; 
    mAction = action; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    ClimberSys.getInstance(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // ClimberSys.getInstance().climbSequence(mStick5.getRawButtonPressed(3));

    //ClimberSys.getInstance().ClimberRun(-mAction.getY(), mAction.getRawButtonPressed(9), mAction.getRawButtonPressed(10));
    
    ClimberSys.getInstance().ClimberRun(mStick5.getRawButton(3), mStick5.getRawButton(4), mStick5.getRawButton(5), mStick5.getRawButton(6));

    if(mStick5.getRawButtonPressed(11)) {
      ClimberSys.getInstance().toggleRight();
      ClimberSys.getInstance().toggleLeft();
    }

    
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
