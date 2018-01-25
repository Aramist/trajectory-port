package com.team5472.robot.pathfinder.from_c;

public class Spline {

    public double a, b, c, d, e;
    public double xOffset;
    public double yOffset;
    public double angleOffset;
    public double knotDistance;
    public double arcLength;

    public static Coord pf_spline_coords(Spline s, double percentage){
        percentage = percentage < 0 ? 0 : percentage > 1 ? 1 : percentage;
        double x = percentage * s.knotDistance;
        double y = (s.a*x + s.b) * (x*x*x*x) + (s.c*x + s.d) * (x*x) + s.e*x;

        double cos_theta = Math.cos(s.angleOffset);
        double sin_theta = Math.sin(s.angleOffset);

        Coord coord = new Coord();
        coord.x = x * cos_theta - y * sin_theta + s.xOffset;
        coord.y = x * sin_theta + y * cos_theta + s.yOffset;
        return coord;
    }

    public static double pf_spline_deriv(Spline s, double percentage){
        double x = percentage * s.knotDistance;
        return (5*s.a*x + 4*s.b) * (x*x*x) + (3*s.c*x + 2*s.d) * x + s.e;
    }

    public static double pf_spline_deriv_2(double a, double b, double c, double d, double e, double k, double p){
        double x = p * k;
        return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
    }

    public static double pf_spline_angle(Spline s, double percentage){
        return MathUtils.boundRadians(Math.atan(pf_spline_deriv(s, percentage)) + s.angleOffset);
    }

    public static double pf_spline_distance(Spline s, int sampleCount) {
        double sample_count_d = (double) sampleCount;
        double a = s.a, b = s.b, c = s.c, d = s.d, e = s.e, knot = s.knotDistance;

        double arc_length = 0, t = 0, dydt = 0;

        double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);

        double integrand = 0;
        double last_integrand = Math.sqrt(1 + deriv0 * deriv0) / sample_count_d;

        for (int i = 0; i <= sampleCount; ++i) {
            t = i / sample_count_d;
            dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
            integrand = Math.sqrt(1 + dydt * dydt) / sample_count_d;
            arc_length += (integrand + last_integrand) / 2;
            last_integrand = integrand;
        }

        double al = knot * arc_length;
        s.arcLength = al;
        return al;
    }

    public static double pf_spline_progress_for_distance(Spline s, double distance, int sample_count){
        double sample_count_d = (double) sample_count;
        double a = s.a, b = s.b, c = s.c, d = s.d, e = s.e, knot = s.knotDistance;

        double arc_length = 0, t = 0, dydt = 0, last_arc_length = 0;

        double deriv0 = pf_spline_deriv_2(a, b, c, d, e, knot, 0);

        double integrand = 0;
        double last_integrand = Math.sqrt(1 + deriv0 * deriv0) / sample_count_d;

        distance /= knot;

        for(int i = 0; i <= sample_count; i++){
            t = i / sample_count_d;
            dydt = pf_spline_deriv_2(a, b, c, d, e, knot, t);
            integrand = Math.sqrt(1 + dydt * dydt) / sample_count_d;
            arc_length += (integrand + last_integrand) / 2;
            if(arc_length > distance) break;
            last_integrand = integrand;
            last_arc_length = arc_length;
        }

        double interpolated = t;
        if(arc_length != last_arc_length)
            interpolated += ((distance - last_arc_length) / (arc_length - last_arc_length) - 1) / sample_count_d;
        return interpolated;
    }
}
