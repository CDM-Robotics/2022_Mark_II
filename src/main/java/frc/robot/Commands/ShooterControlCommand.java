package frc.robot.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LogitechDualAction;
import frc.robot.LogitechJoystick;
import frc.robot.Subsystems.DriveSys;
import frc.robot.Subsystems.ShooterSys;

public class ShooterControlCommand extends CommandBase {

  LogitechDualAction mStick1; 
  /** Creates a new ArcadeDriveCommand. */
  public ShooterControlCommand(LogitechDualAction mStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ShooterSys.getInstance());

    this.mStick1 = mStick; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ShooterSys.getInstance(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ShooterSys.getInstance().runSerializer(mStick1.getRawButton(8));
    ShooterSys.getInstance().shooterAimControl(mStick1);
    ShooterSys.getInstance().ShooterSpinUpToggle(mStick1.getRawButtonPressed(6), mStick1.getRawButton(5));
    ShooterSys.getInstance().pushToSmartDashboard();
    ShooterSys.getInstance().defaultPosition(mStick1.getRawButtonPressed(4));

    SmartDashboard.putNumber("stick y", mStick1.getRawAxis(3)); 
    SmartDashboard.putNumber("stick x", mStick1.getX()); 
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
