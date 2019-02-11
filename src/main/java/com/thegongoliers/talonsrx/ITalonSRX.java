package com.thegongoliers.talonsrx;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.SpeedController;

public interface ITalonSRX extends SpeedController {
    void setSensor(FeedbackDevice feedbackDevice);

    void setSensorCoefficient(double sensorCoefficient);

    void setRemoteSensor(int remoteTalonID);

    void setNeutralDeadband(double percentOutput);

    void useBrakeMode();

    void useCoastMode();

    void follow(ITalonSRX master);

    void unfollow();

    void setPID(double kp, double ki, double kd, int threshold);

    void setPIDF(double kp, double ki, double kd, double kf, int threshold);

    void setRamp(double secondsFromNeutralToFull);

    public WPI_TalonSRX getTalon();

    @Override
    void set(double speed);

    void setPosition(double position);

    void setVelocity(double velocity);

    void setCurrent(double current);

    @Override
    double get();

    double getCurrent();

    double getVoltage();

    void resetEncoder();

    double getPosition();

    double getVelocity();

    @Override
    void setInverted(boolean isInverted);

    @Override
    boolean getInverted();

    void setSensorPhase(boolean sensorPhase);

    @Override
    void disable();

    @Override
    void stopMotor();

    @Override
    void pidWrite(double output);

    void enableCurrentLimit(int peakCurrentAmps, int maintainCurrentAmps);

    void disableCurrentLimit();

    void startMotionProfile(BufferedTrajectoryPointStream pointStream);

    boolean isMotionProfileFinished();
}
