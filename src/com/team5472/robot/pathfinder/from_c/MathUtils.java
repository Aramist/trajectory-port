package com.team5472.robot.pathfinder.from_c;

public class MathUtils {

    public static double TAU = Math.PI * 2;

    public static double boundRadians(double angle){
        double modded = angle % TAU;
        return modded > 0 ? modded : modded + TAU;
    }

}
