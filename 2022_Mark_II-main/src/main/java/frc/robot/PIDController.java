// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class PIDController {

    private double kp, ki, kd; 

    private double errorCumulative; 

    private double lastError;

    public PIDController(double kp, double ki, double kd) {

        this.kp = kp; 
        this.ki = ki; 
        this.kd = kd; 
    }

    public double calc(double target, double current) {

        double output;
        double error; 
        double dError;

        error = target - current; 
        
        errorCumulative += error * 0.02; 

        dError = (error - lastError) / 0.02;
        
        output = (kp * error) + (ki * errorCumulative) + (kd * dError);

        lastError = error;

        return output; 
    }

    public void assignValues(double p, double i, double d) {

        this.kp = p; 
        this.ki = i; 
        this.kd = d; 
    } 


}
