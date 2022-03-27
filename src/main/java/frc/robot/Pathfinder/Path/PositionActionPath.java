// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Subsystems.DriveSys;

/** Add your docs here. */
public class PositionActionPath {

    private static final double Inches_to_Meters = 0.0254;

    private TrajectoryConfig mConfig;


    public PositionActionPath(double MaximumVelocity, double MaximumAcceleration) {

        mConfig = new TrajectoryConfig(MaximumVelocity * Inches_to_Meters, MaximumAcceleration * Inches_to_Meters);
        

    }

    

}
