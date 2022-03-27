// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Pathfinder.Obstacles;

import frc.robot.Pathfinder.Path.PositionPoint;

/** Add your docs here. */
public class ObjectSide {

    private double x1, y1, x2, y2; 




    public ObjectSide(double x1, double y1, double x2, double y2) {

        this.x1 = x1;
        this.x2 = x2;
        this.y1 = y1;
        this.y2 = y2;
    }

    private double getSmallest(double p1, double p2) {

        if (p1 > p2) {

            return p2; 
        }else {

            return p1; 
        }
    }

    private double getLargest(double p1, double p2) {

        if (p1 > p2) {

            return p1; 
        }else {
            
            return p2; 
        }
    } 

    private double getSideSlope() {

        if (getSmallest(x1, x2) == x1) {

            return (x1 - x2) / (y1 - y2); 
        } else {

            return (x2 - x1) / (y2 - y1);
        }
               
    }

    private double getRelativeIntercept() {

        if (getSmallest(x1, x2) == x1) {

            return y1;
        } else {

            return y2; 
        }

    }

    /**  
     *  @return
     *   If returns 0 the position given does not occupy the sides collision hitbox.         
     *   If returns 1 the position given occupies the sides rectangular hitbox below the slope line of the side. 
     *   If returns 2 the position given occupeis the sides rectangular hitbox above the slope line of the side. 
     */
    public double getCollisionInformation(PositionPoint roboPosition) {

        //checks if point falls within the hitbox for the obstacle side
        if (((roboPosition.xCoord >= getSmallest(x1, x2)) || (roboPosition.xCoord <= getLargest(x1, x2))) 
        || ((roboPosition.yCoord >= getSmallest(y1, y2)) || (roboPosition.yCoord <= getLargest(y1, y2)))) {
           
            if (roboPosition.yCoord >= (((roboPosition.xCoord - getSmallest(x1, x2)) * getSideSlope()) + getRelativeIntercept())) {

                return 2; 
            }else {

                return 1; 
            }

           
        } else {

            return 0; 
        }

       

        
    }

    


}
