package com.team5472.robot.pathfinder.from_c.followers;

public class EncoderConfig {

    public int initial_position;
    public int ticks_per_revolution;
    public double wheel_circumference;
    public double kp, ki, kd, kv, ka;

    public EncoderConfig(int ini, int ticks, double wheel, double p, double i, double d, double v, double a){
        initial_position = ini;
        ticks_per_revolution = ticks;
        wheel_circumference = wheel;
        kp = p;
        ki = i;
        kd = d;
        kv = v;
        ka = a;
    }
}
