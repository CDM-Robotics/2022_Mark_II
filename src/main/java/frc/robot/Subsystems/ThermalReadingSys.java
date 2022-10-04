// Thermal Reading SubSystem:
// (Displays current temp and warnings)

package frc.robot.Subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import java.util.ArrayList;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ThermalReadingSys implements Subsystem
{
    //Constant overheat temperature
    private final double OVERHEATTEMPERATURE = 82.5;
    
    //variables with temperature from Falcon 500
    private static ArrayList<WPI_TalonFX> mMotors;
    public int tempTimer;
    public static ThermalReadingSys mThermalReadingSys;
    public int warningCounter;
    
    public static ThermalReadingSys getInstance(ArrayList<WPI_TalonFX> motors) 
    {
        if (mThermalReadingSys == null) {
            mThermalReadingSys = new ThermalReadingSys(); 
            mMotors = motors;
          }
          return mThermalReadingSys;
    }

    public ThermalReadingSys() 
    {
        tempTimer = 0;
        warningCounter = 0;
    }

    public void checkTemp() 
    {
        //Updates current temperature
        double motorTemp = 0.00;
        double maxMotorTemp = 0.00;
        
        //Counter for periodic temperature value display
        tempTimer++;

        //Displays current temperature periodically
        if (tempTimer == 500) 
        {
            for (WPI_TalonFX m : mMotors) 
            {
                motorTemp = m.getTemperature();
                if (motorTemp > maxMotorTemp) 
                {
                    maxMotorTemp = motorTemp;
                }
            }
            SmartDashboard.putNumber("Current Temperature.", maxMotorTemp);
            tempTimer = 0;

            //Warning to indicated "overheating" of motors
            if (maxMotorTemp > OVERHEATTEMPERATURE && warningCounter == 0) 
            { 
                SmartDashboard.putNumber("Warning. Exceding Recommended Operational Temperature.", maxMotorTemp);
                warningCounter = 1;
            }
            if (maxMotorTemp < OVERHEATTEMPERATURE && warningCounter == 1) 
            {
                SmartDashboard.putNumber("Warning. Temperature Returning To Operation Level.", maxMotorTemp);
                warningCounter = 0;
            }
        }
    }
}