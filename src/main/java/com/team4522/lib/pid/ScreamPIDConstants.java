package com.team4522.lib.pid;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc2024.Constants.FeedforwardConstants;

/**
 * A container class for PID constants, along with additional methods.
 */
public class ScreamPIDConstants implements Cloneable{
    private double kP, kI, kD, kF = 0;
    private double period = 0.02;
    private double minOutput = -1;
    private double maxOutput = 1;
    private double integralZone = Double.MAX_VALUE;
    private double maxIntegralAccumulator = Double.POSITIVE_INFINITY;
    private double minIntegralAccumulator = Double.NEGATIVE_INFINITY;

    public ScreamPIDConstants(){}

    public ScreamPIDConstants(double p, double i, double d){
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public ScreamPIDConstants(double p, double i, double d, double f){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

	public void setPID(double p, double i, double d){
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void setPIDF(double p, double i, double d, double f){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
    }

    public void setPeriod(double period){
        this.period = period;
    }

    public void setP(double p){
        this.kP = p;
    }
    
    public void setI(double i){
        this.kI = i;
    }
    
    public void setD(double d){
        this.kD = d;
    }

    public void setF(double f){
        this.kF = f;
    }

    public void setIntegralZone(double Izone){
        this.integralZone = Izone;
    }

    public void setIntegralAccumulatorBounds(double max, double min){
        this.maxIntegralAccumulator = max;
        this.minIntegralAccumulator = min;
    }

    public void setOutputBounds(double max, double min){
        this.maxOutput = max;
        this.minOutput = min;
    }

    public ScreamPIDConstants withPID(double p, double i, double d){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        return this;
    }

    public ScreamPIDConstants withPIDF(double p, double i, double d, double f){
        this.kP = p;
        this.kI = i;
        this.kD = d;
        this.kF = f;
        return this;
    }

    public ScreamPIDConstants withPeriod(double period){
        this.period = period;
        return this;
    }

    public ScreamPIDConstants withP(double p){
        this.kP = p;
        return this;
    }
    
    public ScreamPIDConstants withI(double i){
        this.kI = i;
        return this;
    }
    
    public ScreamPIDConstants withD(double d){
        this.kD = d;
        return this;
    }

    public ScreamPIDConstants withF(double f){
        this.kF = f;
        return this;
    }

    public ScreamPIDConstants withIntegralZone(double Izone){
        this.integralZone = Izone;
        return this;
    }

    public ScreamPIDConstants withIntegralAccumulatorBounds(double max, double min){
        this.maxIntegralAccumulator = max;
        this.minIntegralAccumulator = min;
        return this;
    }

    public ScreamPIDConstants withOutputBounds(double max, double min){
        this.maxOutput = max;
        this.minOutput = min;
        return this;
    }

    public double period(){
        return period;
    }

    public double kP(){
        return kP;
    }

    public double kI(){
        return kI;
    }

    public double kD(){
        return kD;
    }

    public double kF(){
        return kF;
    }

    public double integralZone(){
        return integralZone;
    }

    public double maxIntegralAccumulator(){
        return maxIntegralAccumulator;
    }

    public double minIntegralAccumulator(){
        return minIntegralAccumulator;
    }

    public double maxOutput(){
        return maxOutput;
    }

    public double minOutput(){
        return minOutput;
    }

    public Slot0Configs toSlot0Configs(FeedforwardConstants ffConstants){
        Slot0Configs config = new Slot0Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = ffConstants.kV();
        config.kA = ffConstants.kA();
        config.kG = ffConstants.kG();
        config.kS = ffConstants.kS();
        config.GravityType = ffConstants.gravityType();
        return config;
    }

    public Slot1Configs toSlot1Configs(FeedforwardConstants ffConstants){
        Slot1Configs config = new Slot1Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = ffConstants.kV();
        config.kA = ffConstants.kA();
        config.kG = ffConstants.kG();
        config.kS = ffConstants.kS();
        return config;
    }

    public Slot2Configs toSlot2Configs(FeedforwardConstants ffConstants){
        Slot2Configs config = new Slot2Configs();
        config.kP = kP;
        config.kI = kI;
        config.kD = kD;
        config.kV = ffConstants.kV();
        config.kA = ffConstants.kA();
        config.kG = ffConstants.kG();
        config.kS = ffConstants.kS();
        return config;
    }

    public PIDConstants toPathPlannerPIDConstants(){
        return new PIDConstants(kP, kI, kD, integralZone);
    }

    public PIDController toPIDController(){
        return new PIDController(kP, kI, kD, period);
    }

    public ProfiledPIDController toProfiledPIDController(Constraints constraints){
        return new ProfiledPIDController(kP, kI, kD, constraints, period);
    }

    public boolean equals(ScreamPIDConstants other){
        return
            this.period == other.period &&
            this.kP == other.kP &&
            this.kI == other.kI &&
            this.kD == other.kD &&
            this.kF == other.kF &&
            this.minOutput == other.minOutput &&
            this.maxOutput == other.maxOutput &&
            this.integralZone == other.integralZone &&
            this.maxIntegralAccumulator == other.maxIntegralAccumulator &&
            this.minIntegralAccumulator == other.minIntegralAccumulator;
    }

    public ScreamPIDConstants clone(){
        ScreamPIDConstants copy = new ScreamPIDConstants();
        copy.period = this.period;
        copy.kP = this.kP;
        copy.kI = this.kI;
        copy.kD = this.kD;
        copy.kF = this.kF;
        copy.minOutput = this.minOutput;
        copy.maxOutput = this.maxOutput;
        copy.integralZone = this.integralZone;
        copy.maxIntegralAccumulator = this.maxIntegralAccumulator;
        copy.minIntegralAccumulator = this.minIntegralAccumulator;
        return copy;
    }
}
