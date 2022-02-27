// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Path;

import java.util.ArrayList;

/** Add your docs here. */
public class Path {

    public static final int iterationAccuracy = 100; 
    public static final double iterationAccuracy_double = iterationAccuracy; 
    public static final double tMax = 750.00;
    public static final double tPercent = 0.65;

    private static Path mPath;

    private static Objective[] mObjectiveArray = {}; 

    private static ArrayList<TempPoint> mObjectiveBasePath = new ArrayList<TempPoint>(); // array of the objectives the robot is to complete 

    private static ArrayList<TempPoint> mBezierPath = new ArrayList<TempPoint>(); // array containing all positions on the path

    private static int[] mActionDrivable = {}; 


    public static Path getInstance() {

        if (mPath == null) {

            mPath = new Path();
        }

        return mPath; 
    }

    private static void getObjectiveBasePath() {

        mObjectiveBasePath.add(new TempPoint(mObjectiveArray[0].getX(),mObjectiveArray[0].getY())); 

       

    }

    private static TempPoint getOffsetPoint(TempPoint p1, TempPoint p2, TempPoint p3) {

        TempPoint p4 = new TempPoint(0,0);

        p4.x = (((tPercent * tPercent) * (p1.x + p3.x)) - (2 * p1.x * tPercent) + (p1.x - p2.x)) / (2 * tPercent * (tPercent - 1));
        p4.y = (((tPercent * tPercent) * (p1.y + p3.y)) - (2 * p1.y * tPercent) + (p1.y - p2.y)) / (2 * tPercent * (tPercent - 1));

        return p4;
    }

    

    private static void getBezierPath() {

        ArrayList<ArrayList<TempPoint>> powerList = new ArrayList<ArrayList<TempPoint>>(); // arraylist of arraylists 

        double iNum = 0; 

        

        for (int i = 1; i <= iterationAccuracy; i++) {

            iNum++; 
 
            powerList.add(mObjectiveBasePath); // sets the 0 position to the arraylist holding the calculated objective points

            for (int q = 1; q <= mObjectiveBasePath.size(); q++) { // q represents the current power beginning at 0
            

                for (int w = 0; w < mObjectiveBasePath.size() - q; w++) { // w - 1 is the number of points at the current power 

                    //for each pair 
                    powerList.get(q).add(BezierCurve.getNextPower(iNum, powerList.get(q-1).get(w), powerList.get(q-1).get(w + 1)));

                    if (q == mObjectiveBasePath.size()) {

                        mBezierPath.add(powerList.get(q).get(w));
                    }

                }


            }

            powerList.clear();

        }    


    }

    

}
