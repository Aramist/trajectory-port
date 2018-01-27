package com.team5472.robot.pathfinder.from_c;

import java.util.Arrays;

public class Pathfinder {

    public static Segment[] generate(Waypoint[] waypoints, Trajectory.Config config) {
        TrajectoryCandidate cand = new TrajectoryCandidate();
        Generator.prepare(waypoints, config.fit, config.samples, config.dt, config.max_velocity, config.max_acceleration,
                config.max_jerk, cand);
        Segment[] toReturn = new Segment[cand.length];
        Generator.generate(cand, toReturn);
        return toReturn;
    }

}
