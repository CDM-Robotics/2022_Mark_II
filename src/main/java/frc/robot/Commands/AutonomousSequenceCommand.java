// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSys;
import frc.robot.Subsystems.IntakeSys;
import frc.robot.Subsystems.ShooterSys;
import frc.robot.Subsystems.VisionSys;

public class AutonomousSequenceCommand extends CommandBase {

  private double feet_to_meters = 0.3048;
  private boolean notRun = true; 

  /** Creates a new AutonomousSequenceCommand. */
  public AutonomousSequenceCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(DriveSys.getInstance());
    addRequirements(ShooterSys.getInstance());
    addRequirements(IntakeSys.getInstance());

  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    DriveSys.getInstance();
    ShooterSys.getInstance();
    IntakeSys.getInstance();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double distanceTraveled = DriveSys.getInstance().getLinearDistanceMeters();

    //double distanceToTarget = VisionSys.getInstance().getDistanceToTargetInches();
    if (notRun) {

      ShooterSys.getInstance().ShooterSpinUpToggle(notRun, false);
      notRun = false;
    } else {

      ShooterSys.getInstance().ShooterSpinUpToggle(false, false);

    }

    IntakeSys.getInstance().runIntake(true);

      if (distanceTraveled < (41 * 0.0254)) {

        DriveSys.getInstance().sideIndependentControl(0.1, 0.1);
      } else {

        DriveSys.getInstance().sideIndependentControl(0, 0);
        ShooterSys.getInstance().runSerializer(true);
        
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
