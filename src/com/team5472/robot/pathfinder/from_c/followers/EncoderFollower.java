package com.team5472.robot.pathfinder.from_c.followers;

public class EncoderFollower {

    public double last_error;
    public double heading;
    public double output;
    public int segment;
    public int finished;

    public EncoderFollower(double last, double heading, double out, int seg){
        last_error = last;
        this.heading = heading;
        output = out;
        segment = seg;
        finished = 0;
    }

}
