// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechDualAction;
import frc.robot.LogitechJoystick;
import frc.robot.Subsystems.IntakeSys;

public class IntakeControlCommand extends CommandBase {

  LogitechJoystick mStick2; 
  /** Creates a new IntakeControlCommand. */
  public IntakeControlCommand(LogitechJoystick stick) {
    // Use addRequirements() here to declare subsystem dependencies.
    mStick2 = stick; 
    addRequirements(IntakeSys.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    IntakeSys.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    IntakeSys.getInstance().IntakeOutIn(mStick2.getRawButtonPressed(2));

    IntakeSys.getInstance().runIntake(mStick2.getRawButton(1));
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
