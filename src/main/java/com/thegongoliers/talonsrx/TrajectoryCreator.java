package com.thegongoliers.talonsrx;

import com.ctre.phoenix.motion.BufferedTrajectoryPointStream;
import com.ctre.phoenix.motion.TrajectoryPoint;

public class TrajectoryCreator {

    private TrajectoryCreator(){

    }

    /**
     * Creates a trajectory from a profile. Does not assume the units of the profile.
     * @param profile The profile as generated from Pathweaver.py (double[][] of {position, velocity, acceleration, duration}.
     * @param positionToTicks The conversion from units to ticks. This may be the ticks per complete rotation or a conversion from distance per rotation to ticks.
     * @param velocityToTicks The conversion from units to ticks/100ms.
     * @return The motion profile to follow.
     */
    public static BufferedTrajectoryPointStream createTrajectory(double[][] profile, double positionToTicks, double velocityToTicks){
        TrajectoryPoint point = new TrajectoryPoint();
        BufferedTrajectoryPointStream pointStream = new BufferedTrajectoryPointStream();

        for (int i = 0; i < profile.length; i++) {
            double position = profile[i][0];
            double velocity = profile[i][1];
//            double acceleration = profile[i][2];
            int duration = (int) profile[i][3];

            point.timeDur = duration;
            point.position = position * positionToTicks;
            point.velocity = velocity * velocityToTicks;

            point.auxiliaryPos = 0;
            point.auxiliaryVel = 0;
            point.profileSlotSelect0 = 0;
            point.profileSlotSelect1 = 1;
            point.zeroPos = (i == 0);
            point.isLastPoint = (i == profile.length - 1);
            point.arbFeedFwd = 0;

            pointStream.Write(point);
        }

        return pointStream;
    }

    /**
     * Creates a trajectory from a profile. Assumes the profile is in ticks and ticks/100ms.
     * @param profile The profile as generated from Pathweaver.py (double[][] of {position, velocity, acceleration, duration}.
     * @return The motion profile to follow.
     */
    public static BufferedTrajectoryPointStream createTrajectory(double[][] profile){
        return createTrajectory(profile, 1, 1);
    }

}
