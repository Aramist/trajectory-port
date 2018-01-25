package com.team5472.robot.pathfinder.from_c.fit;

import com.team5472.robot.pathfinder.from_c.Spline;
import com.team5472.robot.pathfinder.from_c.Waypoint;

import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

import static com.team5472.robot.pathfinder.from_c.MathUtils.boundRadians;

public interface FitMethod {

    void fit(Waypoint a, Waypoint b, Spline s);

    FitMethod HERMITE_CUBIC = (a, b, s) -> {
        FitMethod.HERMITE_PRE.fit(a,b,s);

        double a0_delta = tan(boundRadians(a.angle - s.angleOffset));
        double a1_delta = tan(boundRadians(b.angle - s.angleOffset));

        s.a = 0;
        s.b = 0;
        s.c = (a0_delta + a1_delta) / (s.knotDistance * s.knotDistance);
        s.d = -(2 * a0_delta + a1_delta) / s.knotDistance;
        s.e = a0_delta;
    };

    FitMethod HERMITE_QUINTIC = (a, b, s) -> {
        FitMethod.HERMITE_PRE.fit(a, b, s);

        double a0_delta = tan(boundRadians(a.angle - s.angleOffset));
        double a1_delta = tan(boundRadians(b.angle - s.angleOffset));

        double d = s.knotDistance;

        s.a = -(3 * (a0_delta + a1_delta)) / (d*d*d*d);
        s.b = (8 * a0_delta + 7 * a1_delta) / (d*d*d);
        s.c = -(6 * a0_delta + 4 * a1_delta) / (d*d);
        s.d = 0;
        s.e = a0_delta;
    };

    FitMethod HERMITE_PRE = (a, b, s) -> {
        s.xOffset = a.x;
        s.yOffset = a.y;

        double delta = sqrt((b.x - a.x)*(b.x - a.x) + (b.y - a.y)*(b.y - a.y));
        s.knotDistance = delta;
        s.angleOffset = atan2(b.y - a.y, b.x - a.x);
    };
}
