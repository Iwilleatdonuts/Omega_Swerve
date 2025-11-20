package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.teamcode.Utilities.math.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Utilities.math.geometry.Rotation2d;

public class OmegaPose2D {

    private double x;
    private double y;
    private double r;

    public OmegaPose2D(double x, double y, double r) {
        this.x = x;
        this.y = y;
        this.r = r;
    }

    public double x(){
        return x;
    }

    public double y(){
        return y;
    }

    public double r(){
        return r;
    }

    public static Pose2d OmegaPoseToWPIPose(OmegaPose2D omegaPose) {
        return new Pose2d(omegaPose.x(), omegaPose.y(), Rotation2d.fromDegrees(omegaPose.r()));
    }

    public static OmegaPose2D WPIPoseToOmegaPose(Pose2d wpiPose) {
        double r = wpiPose.getRotation().getDegrees();
        r = ((r % 360) + 360) % 360;
        return new OmegaPose2D(wpiPose.getX(), wpiPose.getY(), r);
    }

}
