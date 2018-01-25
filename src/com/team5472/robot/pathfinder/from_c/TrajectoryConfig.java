package com.team5472.robot.pathfinder.from_c;

public class TrajectoryConfig {

    public double dt;
    public double maxV, maxA, maxJ;
    public double srcV, srcTheta;
    public double destPos, destV, destTheta;
    public int sampleCount;

    public TrajectoryConfig(double dt, double maxV, double maxA, double maxJ, double srcV, double srcTheta,
                            double destPos, double destV, double destTheta, int sampleCount){
        this.dt = dt;
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJ = maxJ;
        this.srcV = srcV;
        this.srcTheta = srcTheta;
        this.destPos = destPos;
        this.destV = destV;
        this.destTheta = destTheta;
        this.sampleCount = sampleCount;
    }

}
