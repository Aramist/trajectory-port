package com.team5472.robot.pathfinder.from_c.followers;

import com.team5472.robot.pathfinder.from_c.Segment;

public class Encoder {

    public static double pathfinder_follow_encoder(EncoderConfig c, EncoderFollower follower, Segment[] trajectory,
                                                   int encoder_tick){
        int segment = follower.segment;
        if(segment >= trajectory.length){
            follower.finished = 1;
            follower.output = 0.0;
            Segment last = trajectory[trajectory.length - 1];
            follower.heading = last.heading;
            return 0.0;
        } else {
            return pathfinder_follow_encoder2(c, follower, trajectory[segment], trajectory.length, encoder_tick);
        }
    }

    public static double pathfinder_follow_encoder2(EncoderConfig c, EncoderFollower follower, Segment s,
                                                    int trajectory_length, int encoder_tick){
        double distance_covered = ((double)encoder_tick - (double)c.initial_position) / ((double)c.ticks_per_revolution);
        distance_covered *= c.wheel_circumference;

        if(follower.segment < trajectory_length){
            follower.finished = 0;
            double error = s.position - distance_covered;
            double calculated_value = c.kp * error +
                                      c.kd * ((error - follower.last_error) / s.dt) +
                                      (c.kv * s.velocity + c.ka * s.acceleration);

            follower.last_error = error;
            follower.heading = s.heading;
            follower.output = calculated_value;
            follower.segment++;
            return calculated_value;
        } else {
            follower.finished = 1;
            return 0.0;
        }
    }
}
