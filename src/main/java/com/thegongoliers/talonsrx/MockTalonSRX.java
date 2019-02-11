package com.thegongoliers.talonsrx;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class MockTalonSRX implements ITalonSRX {

    private ITalonSRX master;
    private double speed, current, voltage, velocity, position, ramp, neutralDeadband;
    private double kp, ki, kd, kf;
    private double sensorCoefficient;
    private boolean inverted, motionProfileFinished, isBrakeMode, currentLimited, sensorPhase;
    private int peakCurrentAmps, maintainCurrentAmps, pidThreshold;
    private FeedbackDevice feedbackDevice;
    private int remoteTalonID;
    private BufferedTrajectoryPointStream trajectory;


    public MockTalonSRX(){
        master = null;
        speed = 0;
        current = 0;
        voltage = 0;
        velocity = 0;
        position = 0;
        ramp = 0;
        neutralDeadband = 0;
        sensorCoefficient = 0;
        kp = ki = kd = kf = 0;
        sensorPhase = false;
        currentLimited = false;
        isBrakeMode = false;
        motionProfileFinished = false;
        inverted = false;
        peakCurrentAmps = 0;
        maintainCurrentAmps = 0;
        pidThreshold = 0;
        remoteTalonID = 0;
        feedbackDevice = null;
        trajectory = null;
    }

    @Override
    public void setSensor(FeedbackDevice feedbackDevice) {
        this.feedbackDevice = feedbackDevice;
    }

    public FeedbackDevice getSensor(){
        return feedbackDevice;
    }

    @Override
    public void setSensorCoefficient(double sensorCoefficient) {
        this.sensorCoefficient = sensorCoefficient;
    }

    public double getSensorCoefficient() {
        return sensorCoefficient;
    }

    @Override
    public void setRemoteSensor(int remoteTalonID) {
        this.remoteTalonID = remoteTalonID;
    }

    public int getRemoteTalonID() {
        return remoteTalonID;
    }

    @Override
    public void setNeutralDeadband(double percentOutput) {
        this.neutralDeadband = percentOutput;
    }

    public double getNeutralDeadband() {
        return neutralDeadband;
    }

    @Override
    public void useBrakeMode() {
        isBrakeMode = true;
    }

    @Override
    public void useCoastMode() {
        isBrakeMode = false;
    }

    public boolean isBrakeMode(){
        return isBrakeMode;
    }

    public boolean isCoastMode(){
        return !isBrakeMode;
    }

    public ITalonSRX getMaster(){
        return master;
    }

    @Override
    public void follow(ITalonSRX master) {
        this.master = master;
    }

    @Override
    public void unfollow() {
        master = null;
    }

    @Override
    public void setPID(double kp, double ki, double kd, int threshold) {
        setPIDF(kp, ki, kd, 0, threshold);
    }

    @Override
    public void setPIDF(double kp, double ki, double kd, double kf, int threshold) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;
        this.pidThreshold = threshold;
    }

    public double getKp(){
        return kp;
    }

    public double getKi() {
        return ki;
    }

    public double getKd() {
        return kd;
    }

    public double getKf() {
        return kf;
    }

    public int getPidThreshold() {
        return pidThreshold;
    }

    @Override
    public void setRamp(double secondsFromNeutralToFull) {
        ramp = secondsFromNeutralToFull;
    }

    public double getRamp(){
        return ramp;
    }

    @Override
    public WPI_TalonSRX getTalon() {
        // Return null because hardware can't be initialized
        return null;
    }

    @Override
    public void set(double speed) {
        this.speed = speed;
    }

    @Override
    public void setPosition(double position) {
        this.position = position;
    }

    @Override
    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public void setCurrent(double current) {
        this.current = current;
    }

    @Override
    public double get() {
        return speed;
    }

    @Override
    public double getCurrent() {
        return current;
    }

    @Override
    public double getVoltage() {
        return voltage;
    }

    @Override
    public void resetEncoder() {
        position = 0;
    }

    @Override
    public double getPosition() {
        return position;
    }

    @Override
    public double getVelocity() {
        return velocity;
    }

    @Override
    public void setInverted(boolean isInverted) {
        this.inverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return inverted;
    }

    @Override
    public void setSensorPhase(boolean sensorPhase) {
        this.sensorPhase = sensorPhase;
    }

    public boolean getSensorPhase(){
        return sensorPhase;
    }

    @Override
    public void disable() {
        speed = 0;
        velocity = 0;
        voltage = 0;
        current = 0;
    }

    @Override
    public void stopMotor() {
        speed = 0;
        velocity = 0;
        voltage = 0;
        current = 0;
    }

    @Override
    public void pidWrite(double output) {
        speed = output;
    }

    @Override
    public void enableCurrentLimit(int peakCurrentAmps, int maintainCurrentAmps) {
        this.peakCurrentAmps = peakCurrentAmps;
        this.maintainCurrentAmps = maintainCurrentAmps;
        currentLimited = true;
    }

    @Override
    public void disableCurrentLimit() {
        this.peakCurrentAmps = 0;
        this.maintainCurrentAmps = 0;
        currentLimited = true;
    }

    public boolean isCurrentLimited(){
        return currentLimited;
    }

    public int getPeakCurrentAmps(){
        return peakCurrentAmps;
    }

    public int getMaintainCurrentAmps(){
        return maintainCurrentAmps;
    }


    @Override
    public void startMotionProfile(BufferedTrajectoryPointStream pointStream) {
        this.trajectory = pointStream;
    }

    public BufferedTrajectoryPointStream getMotionProfile() {
        return trajectory;
    }

    public void setIsMotionProfileFinished(boolean isFinished){
        motionProfileFinished = isFinished;
        if (isFinished){
            trajectory = null;
        }
    }

    @Override
    public boolean isMotionProfileFinished() {
        return motionProfileFinished;
    }
}
