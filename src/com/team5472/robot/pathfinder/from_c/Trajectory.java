package com.team5472.robot.pathfinder.from_c;

import com.team5472.robot.pathfinder.from_c.fit.FitMethod;

public class Trajectory {

    public static class Config{
        public FitMethod fit;
        public int samples;
        public double dt, max_velocity, max_acceleration, max_jerk;

        public Config(FitMethod fit, int samples, double dt, double max_v, double max_a, double max_j){
            this.fit = fit;
            this.samples = samples;
            this.dt = dt;
            this.max_velocity = max_v;
            this.max_acceleration = max_a;
            this.max_jerk = max_j;
        }
    }


    protected static void copy(Segment[] source, Segment[] dest, int length){
        for(int i = 0; i < length; i++){
            Segment s = source[0];
            Segment d = new Segment(s.dt, s.x, s.y, s.position, s.velocity, s.acceleration, s.jerk, s.heading);

            dest[i] = d;
        }
    }

    protected static TrajectoryInfo prepare(TrajectoryConfig c){
        double max_a2 = c.maxA * c.maxA;
        double max_j2 = c.maxJ * c.maxJ;

        double checked_max_v = Math.min(c.maxV,
                (-(max_a2) + Math.sqrt(max_a2 * max_a2 + 4 * (max_j2 * c.maxA * c.destPos))) / (2 * c.maxJ));

        int filter1 = (int) Math.ceil((checked_max_v / c.maxA) / c.dt);
        int filter2 = (int) Math.ceil((c.maxA / c.maxJ) / c.dt);

        double impulse = (c.destPos / checked_max_v) / c.dt;
        int time = (int) Math.ceil(filter1 + filter2 + impulse);

        TrajectoryInfo info = new TrajectoryInfo();

        info.filter1 = filter1;
        info.filter2 = filter2;
        info.length = time;
        info.dt = c.dt;
        info.u = 0;
        info.v = checked_max_v;
        info.impulse = impulse;

        return info;
    }

    protected static int create(TrajectoryInfo info, TrajectoryConfig c, Segment[] seg){
        int ret = fromSecondOrderFilter(info.filter1, info.filter2, info.dt, info.u, info.v, info.impulse,
                info.length, seg);

        if(ret < 0)
            return ret;

        double d_theta = c.destTheta - c.srcTheta;

        for(int i = 0; i < info.length; i++)
            seg[i].heading = c.srcTheta + d_theta * (seg[i].position) / (seg[info.length - 1].position);
        return 0;
    }

    protected static int fromSecondOrderFilter(int filter_1_l, int filter_2_l, double dt, double u, double v,
                                            double impulse, int len, Segment[] t){
        Segment last_section = new Segment(dt, 0, 0, u, 0, 0, 0, 0);

        if(len < 0)
            return -1;

        double[] f1_buffer = new double[len];
        f1_buffer[0] = (u / v) * filter_1_l;
        double f2;

        for(int i = 0; i < len; i++){
            double input = Math.min(impulse, 1);
            if(input < 1){
                input -= 1;
                impulse = 0;
            }else{
                impulse -= input;
            }

            double f1_last = i > 0 ? f1_buffer[i - 1] : f1_buffer[0];

            f1_buffer[i] = Math.max(0.0, Math.min(filter_1_l, f1_last + input));

            f2 = 0;
            for(int j = 0; j < filter_2_l; ++j){
                if(i < j)
                    break;
                f2 += f1_buffer[i - j];
            }
            f2 /= filter_1_l;

            t[i].velocity = f2 / filter_2_l * v;

            t[i].position = (last_section.velocity + t[i].velocity) / 2.0 * dt + last_section.position;

            t[i].x = t[i].position;
            t[i].y = 0;

            t[i].acceleration = (t[i].velocity - last_section.velocity) / dt;
            t[i].jerk = (t[i].acceleration - last_section.acceleration) / dt;
            t[i].dt = dt;

            last_section = t[i];
        }
        return 0;
    }

}
