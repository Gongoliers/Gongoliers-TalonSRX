package com.thegongoliers.talonsrx;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class GTalonSRX implements ITalonSRX {

    private WPI_TalonSRX talon;

    /**
     * Create a talon.
     * @param canID The talon's id.
     */
    public GTalonSRX(int canID){
        talon = new WPI_TalonSRX(canID);
        talon.set(ControlMode.PercentOutput, 0.0);
        talon.configFactoryDefault();
        talon.configNeutralDeadband(0.04);
        talon.configNominalOutputForward(0);
        talon.configNominalOutputReverse(0);
        talon.configPeakOutputForward(1.0);
        talon.configPeakOutputReverse(-1.0);
        talon.setInverted(false);
        setSensorPhase(false);
        talon.enableCurrentLimit(false);
        talon.selectProfileSlot(0, 0);
        useCoastMode();
    }

    /**
     * Create a slave talon.
     * @param masterID The master talon's id.
     * @param slaveID The slave talon's id.
     */
    public GTalonSRX(int masterID, int slaveID){
        this(slaveID);
        talon.set(ControlMode.Follower, masterID);
    }

    /**
     * Set the selected sensor of the talon.
     * @param feedbackDevice The type of the sensor.
     */
    @Override
    public void setSensor(FeedbackDevice feedbackDevice){
        talon.configSelectedFeedbackSensor(feedbackDevice, 0, 30);
    }

    /**
     * Set the coefficient of the sensor (can be used for encoder position).
     * @param sensorCoefficient The coefficient of the sensor.
     */
    @Override
    public void setSensorCoefficient(double sensorCoefficient){
        talon.configSelectedFeedbackCoefficient(sensorCoefficient);
    }

    /**
     * Set this talon's sensor to the sensor on another talon.
     * @param remoteTalonID The id of the talon which has the sensor to use selected.
     */
    @Override
    public void setRemoteSensor(int remoteTalonID){
        talon.configRemoteFeedbackFilter(remoteTalonID, RemoteSensorSource.TalonSRX_SelectedSensor, 0);
        setSensor(FeedbackDevice.RemoteSensor0);
    }

    /**
     * Set the neutral deadband (sets motor to 0 if below percent output).
     * @param percentOutput The percent output of the neutral deadband.
     */
    @Override
    public void setNeutralDeadband(double percentOutput){
        talon.configNeutralDeadband(percentOutput);
    }

    /**
     * Use the brake mode when in neutral.
     */
    @Override
    public void useBrakeMode(){
        talon.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * Use the coast mode when in neutral.
     */
    @Override
    public void useCoastMode(){
        talon.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Follow a talon SRX motor.
     * @param master The master talon to follow.
     */
    @Override
    public void follow(ITalonSRX master){
        talon.follow(master.getTalon());
//        talon.set(ControlMode.Follower, master.talon.getDeviceID());
    }

    /**
     * Unfollow any other talon SRXs if currently following.
     */
    @Override
    public void unfollow(){
        talon.set(ControlMode.PercentOutput, 0);
    }


    /**
     * Set the PID of the talon.
     * @param kp The proportional constant.
     * @param ki The integral constant.
     * @param kd The differential constant.
     * @param threshold The error threshold of the PID loop.
     */
    @Override
    public void setPID(double kp, double ki, double kd, int threshold){
        setPIDF(kp, ki, kd, 0, threshold);
    }

    /**
     * Set the PIDF of the talon.
     * @param kp The proportional constant.
     * @param ki The integral constant.
     * @param kd The differential constant.
     * @param kf The feed forward constant.
     * @param threshold The error threshold of the PIDF loop.
     */
    @Override
    public void setPIDF(double kp, double ki, double kd, double kf, int threshold){
        talon.config_kP(0, kp);
        talon.config_kI(0, ki);
        talon.config_kD(0, kd);
        talon.config_kF(0, kf);
        talon.configAllowableClosedloopError(0, threshold);
    }

    /**
     * Get the underlying talon for more advanced features.
     * @return The underlying TalonSRX.
     */
    public WPI_TalonSRX getTalon(){
        return talon;
    }

    /**
     * Sets the open loop ramp of a Talon SRX. This will limit the amount the power is allowed to change per tick.
     * @param secondsFromNeutralToFull The seconds it takes to go from neutral (no power) to full (full power).
     */
    @Override
    public void setRamp(double secondsFromNeutralToFull){
        talon.configOpenloopRamp(secondsFromNeutralToFull);
    }

    /**
     * Sets the percent output of the motor.
     * @param speed the percent output between -1.0 and 1.0, with 0.0 as stopped.
     */
    @Override
    public void set(double speed) {
        talon.set(speed);
    }

    /**
     * Sets the position of the motor.
     * @param position The position in encoder ticks.
     */
    @Override
    public void setPosition(double position){
        set(ControlMode.Position, position);
    }

    /**
     * Sets the change in position over 100ms.
     * @param velocity The velocity (position change / 100ms).
     */
    @Override
    public void setVelocity(double velocity){
        set(ControlMode.Velocity, velocity);
    }

    /**
     * Sets the current of the motor in amps.
     * @param current The current in amps.
     */
    @Override
    public void setCurrent(double current){
        set(ControlMode.Current, current);
    }

    private void set(ControlMode controlMode, double value){
        talon.set(controlMode, value);
    }

    @Override
    public double get() {
        return talon.get();
    }

    /**
     * Gets the output current of the motor.
     * @return The output current in amps.
     */
    @Override
    public double getCurrent() {
        return talon.getOutputCurrent();
    }

    /**
     * Gets the output voltage of the motor.
     * @return The output voltage in volts.
     */
    @Override
    public double getVoltage() {
        return talon.getMotorOutputVoltage();
    }

    /**
     * Reset the encoder position to 0.
     */
    @Override
    public void resetEncoder(){
        talon.getSensorCollection().setQuadraturePosition(0, 30);
    }

    /**
     * Get the position of the motor.
     * @return The position in encoder ticks.
     */
    @Override
    public double getPosition(){
        return talon.getSelectedSensorPosition(0);
    }

    /**
     * Get the velocity of the motor.
     * @return The velocity in encoder ticks / 100ms.
     */
    @Override
    public double getVelocity(){
        return talon.getSelectedSensorVelocity(0);
    }

    @Override
    public void setInverted(boolean isInverted) {
        talon.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return talon.getInverted();
    }

    /**
     * Set the sensor phase (if the sensor is inverted).
     * @param sensorPhase The sensor phase.
     */
    @Override
    public void setSensorPhase(boolean sensorPhase){
        talon.setSensorPhase(sensorPhase);
    }

    @Override
    public void disable() {
        talon.disable();
    }

    @Override
    public void stopMotor() {
        talon.stopMotor();
    }

    @Override
    public void pidWrite(double output) {
        talon.pidWrite(output);
    }

    /**
     * Limit the maximum current of the motor.
     * @param peakCurrentAmps The maximum current in amps.
     * @param maintainCurrentAmps The nominal current.
     */
    @Override
    public void enableCurrentLimit(int peakCurrentAmps, int maintainCurrentAmps){
        talon.configPeakCurrentLimit(peakCurrentAmps);
        talon.configContinuousCurrentLimit(maintainCurrentAmps);
        talon.configPeakCurrentDuration(0);
        talon.enableCurrentLimit(true);
    }

    /**
     * Disable current limiting.
     */
    @Override
    public void disableCurrentLimit(){
        talon.enableCurrentLimit(false);
    }

    /**
     * Start following a motion profile.
     * @param pointStream The motion profile to follow.
     */
    @Override
    public void startMotionProfile(BufferedTrajectoryPointStream pointStream){
        talon.startMotionProfile(pointStream, 10, ControlMode.MotionProfile);
    }

    /**
     * Determines if the motion profile is finished.
     * @return True if the motion profile is finished.
     */
    @Override
    public boolean isMotionProfileFinished(){
        return talon.isMotionProfileFinished();
    }

}
