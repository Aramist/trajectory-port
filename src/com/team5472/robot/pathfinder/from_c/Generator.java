package com.team5472.robot.pathfinder.from_c;

import com.team5472.robot.pathfinder.from_c.fit.FitMethod;

public class Generator {

    protected static int prepare(final Waypoint[] path, int path_length, FitMethod fit, int sample_count,
                              double dt, double max_velocity, double max_accel, double max_jerk,
                              TrajectoryCandidate cand){
        if(path_length < 2)
            return -1;

        cand.saptr = new Spline[path_length - 1];
        cand.laptr = new double[path_length - 1];
        double totalLength = 0;

        for(int i = 0; i < path_length - 1; i++){
            Spline s = new Spline();
            fit.fit(path[i], path[i+1], s);
            double dist = Spline.distance(s, sample_count);
            cand.saptr[i] = s;
            cand.laptr[i] = dist;
            totalLength += dist;
        }

        TrajectoryConfig config = new TrajectoryConfig(dt, max_velocity, max_accel, max_jerk, 0, path[0].angle,
                totalLength, 0, path[0].angle, sample_count);
        TrajectoryInfo info = Trajectory.prepare(config);
        int trajectoryLength = info.length;

        cand.totalLength = totalLength;
        cand.length = trajectoryLength;
        cand.path_length = path_length;
        cand.info = info;
        cand.config = config;

        return trajectoryLength;
    }

    protected static int generate(TrajectoryCandidate c, Segment[] segments){
        int trajectory_length = c.length;
        int path_length = c.path_length;

        Spline[] splines = c.saptr;
        double[] splineLengths = c.laptr;

        int trajectory_status = Trajectory.create(c.info, c.config, segments);
        if(trajectory_status < 0)
            return trajectory_status;

        int spline_i = 0;
        double spline_pos_initial = 0, splines_complete = 0;

        for(int i = 0; i < trajectory_length; i++){
            double pos = segments[i].position;

            boolean found = false;
            while(!found){
                double pos_relative = pos - spline_pos_initial;
                if(pos_relative <= splineLengths[spline_i]){
                    Spline si = splines[spline_i];
                    double percentage = Spline.progressForDistance(si, pos_relative, c.config.sampleCount);
                    Coord coords = Spline.interpolatedCoords(si, percentage);
                    segments[i].heading = Spline.angle(si, percentage);
                    segments[i].x = coords.x;
                    segments[i].y = coords.y;
                    found = true;
                } else if(spline_i < path_length - 2) {
                    splines_complete += splineLengths[spline_i];
                    spline_pos_initial = splines_complete;
                    spline_i++;
                } else {
                    Spline si = splines[path_length - 2];
                    segments[i].heading = Spline.angle(si, 1.0);
                    Coord coords = Spline.interpolatedCoords(si, 1.0);
                    segments[i].x = coords.x;
                    segments[i].y = coords.y;
                    found = true;
                }
            }
        }

        return trajectory_length;
    }
}
