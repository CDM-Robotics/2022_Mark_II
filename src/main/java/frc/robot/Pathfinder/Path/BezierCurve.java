// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class BezierCurve {


    public static TempPoint getNextPower(double iNum, TempPoint w0, TempPoint w1) {

        TempPoint waypoint0 = w0; 
        TempPoint waypoint1 = w1; 

        double rX = waypoint1.x - waypoint0.x; 
        double rY = waypoint1.y - waypoint0.y; 

        double nX = waypoint0.x + (rX * (iNum / Path.iterationAccuracy_double)); 
        double nY = waypoint0.y + (rY * (iNum / Path.iterationAccuracy_double)); 

        return new TempPoint(nX, nY); 
    }









    

    /* public static void getBezierPoints(Waypoint p1, Waypoint p2, Waypoint p3) {

        int bezierCount = 10; 

        ArrayList<Waypoint> bezierPoints = new ArrayList<Waypoint>(); 

        TempPoint[] p1p2 = new TempPoint[bezierCount]; 
        TempPoint[] p2p3 = new TempPoint[bezierCount]; 

        p1p2[0] = new TempPoint(0,0);
        p2p3[bezierCount - 1] = new TempPoint(0,0);
        p2p3[0] = new TempPoint(0,0);  

        p1p2[0].x = p1.getX(); 
        p1p2[0].y = p1.getY(); 

        p2p3[bezierCount - 1].x = p3.getX(); 
        p2p3[bezierCount - 1].y = p3.getY(); 

        


        double lineslope0; 
        double lineslope1; 

        double wpx; 
        double wpy; 

        String out; 
        String num; 

        double q_double = 1.0;
        double bezierCount_double = 10.0;


        for (int q = 1; q < bezierCount; q++) {

            p1p2[q] = new TempPoint(0, 0); 
            p2p3[q] = new TempPoint(0, 0); 


            //creates points on line p1p2
            p1p2[q].x = p1.getX() - ((q_double/bezierCount_double) * (p1.getX() - p2.getX())); 

            p1p2[q].y = p1.getY() - ((q_double/bezierCount_double) * (p1.getY() - p2.getY()));

            //SmartDashboard.putNumber("q  beziercount: " + q, q_double / bezierCount_double); 
            //SmartDashboard.putNumber(" p1y - p2y : " + q, p1.getY() - p2.getY());
            //SmartDashboard.putNumber("p1p2 y : " + q, p1p2[q].y);

            q_double = q_double + 1.0; 

            //creates points on line p2p3
            p2p3[q-1].x = p2.getX() - (((q_double)/bezierCount_double) * (p2.getX() - p3.getX())); 
            p2p3[q-1].y = p2.getY() - (((q_double)/bezierCount_double) * (p2.getY() - p3.getY())); 
            
            // get line slopes

            wpx = 0;
            wpy = 0; 

            if ((p1p2[q-1].x == p2p3[q-1].x) || (p1p2[q].x == p2p3[q].x)) {

                if (p1p2[q-1].x == p2p3[q-1].x) {

                    wpx = p1p2[q-1].x; 

                    lineslope1 = (p1p2[q].y - p2p3[q].y) / (p1p2[q].x - p2p3[q].x);

                    wpy = (lineslope1 * wpx) - (lineslope1 * p1p2[q].x) + (p1p2[q].y);
                }                
            
                if (p1p2[q].x == p2p3[q].x) {

                    wpx = p1p2[q].x; 

                    lineslope0 = (p1p2[q-1].y - p2p3[q-1].y) / (p1p2[q-1].x - p2p3[q-1].x);

                    wpy = (lineslope0 * wpx) - (lineslope0 * p1p2[q-1].x) + (p1p2[q-1].y); 
                }  

            } else {

            lineslope0 = (p1p2[q-1].y - p2p3[q-1].y) / (p1p2[q-1].x - p2p3[q-1].x);
            lineslope1 = (p1p2[q].y - p2p3[q].y) / (p1p2[q].x - p2p3[q].x); 

            wpx = ((-lineslope0 * p1p2[q-1].x) + (lineslope1 * p1p2[q].x) + p1p2[q-1].y + (-p1p2[q].y))
                        / (lineslope0 - lineslope1);

            wpy = (lineslope0 * wpx) - (lineslope0 * p1p2[q-1].x) + (p1p2[q-1].y);

            }

            //get waypoint x coord

            out = "(" + wpx + "," + wpy + ")";

            num = "temp point:" + q; 

            SmartDashboard.putString(num, out); 
            SmartDashboard.putString("point" + q, "(" + p1p2[q-1].x + "," + p1p2[q-1].y + ")"); 
            SmartDashboard.putString("Point" + q, "(" + p2p3[q-1].x + "," + p2p3[q-1].y + ")"); 

        }

    }
 */
}
