// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PID_Controller {

    private double kp, ki, kd; 

    private double errorCumulative; 
    private double error;

    private double lastError;

    private double maxAccel; 

    public PID_Controller(double kp, double ki, double kd, double maxAccel) {

        this.kp = kp; 
        this.ki = ki; 
        this.kd = kd; 

        this.maxAccel = maxAccel; 
    }

    public double calc(double target, double current) {

        double output;
        double dError;

        error = target - current; 
        
        errorCumulative += error * 0.02; 

        dError = (error - lastError) / 0.02;
        
        output = (kp * error) + (ki * errorCumulative) + (kd * dError);

        lastError = error;
 
        output = current + output;

        if (current > (output) ) {

            if ((current - output) > maxAccel) {
                return current - maxAccel;
            }

        } else { 

            if ((output - current) > maxAccel) {
                return current + maxAccel;
            }
        }

        return output; 
    }

    public void assignValues(double p, double i, double d) {

        this.kp = p; 
        this.ki = i; 
        this.kd = d; 
    } 

    public double returnError() {

        return error;
    }


}
