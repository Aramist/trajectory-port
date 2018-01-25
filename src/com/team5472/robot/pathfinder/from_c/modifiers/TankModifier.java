package com.team5472.robot.pathfinder.from_c.modifiers;

import com.team5472.robot.pathfinder.from_c.Segment;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class TankModifier {

    public static void pathfinder_modify_tank(Segment[] original, int length, Segment[] leftTraj, Segment[] rightTraj,
                                double wheelbaseWidth){
        double w = wheelbaseWidth / 2.0;

        for(int i = 0; i < length; i++){
            Segment seg = original[i];
            Segment left = seg.clone();
            Segment right = seg.clone();

            double cos_angle = cos(seg.heading);
            double sin_angle = sin(seg.heading);

            left.x = seg.x - (w * sin_angle);
            left.y = seg.y + (w * cos_angle);

            right.x = seg.x + (w * sin_angle);
            right.y = seg.y - (w * cos_angle);

            if(i > 0){
                Segment lastl = leftTraj[i - 1];
                double distancel = sqrt((left.x - lastl.x) * (left.x - lastl.x) + (left.y - lastl.y) * (left.y - lastl.y));

                left.position = lastl.position + distancel;
                left.velocity = distancel / seg.dt;
                left.acceleration = (left.velocity - lastl.velocity) / seg.dt;
                left.jerk = (left.acceleration - lastl.acceleration) / seg.dt;

                Segment lastr = rightTraj[i - 1];
                double distancer = sqrt((right.x - lastr.x) * (right.x - lastr.x) + (right.y - lastr.y) * (right.y - lastr.y));

                right.position = lastr.position + distancer;
                right.velocity = distancer / seg.dt;
                right.acceleration = (right.velocity - lastr.velocity) / seg.dt;
                right.jerk = (right.acceleration - lastr.acceleration) / seg.dt;
            }


            leftTraj[i] = left;
            rightTraj[i] = right;
        }
    }
}
