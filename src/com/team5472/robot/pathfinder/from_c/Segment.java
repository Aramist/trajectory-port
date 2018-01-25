package com.team5472.robot.pathfinder.from_c;

public class Segment {

    public double dt;
    public double x;
    public double y;
    public double position;
    public double velocity;
    public double acceleration;
    public double jerk;
    public double heading;

    public Segment(double dt, double x, double y, double position, double velocity, double acceleration, double jerk,
                   double heading){
        this.dt = dt;
        this.x = x;
        this.y = y;
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
        this.jerk = jerk;
        this.heading = heading;
    }

    public Segment clone(){
        return new Segment(dt, x, y, position, velocity, acceleration, jerk, heading);
    }

}
