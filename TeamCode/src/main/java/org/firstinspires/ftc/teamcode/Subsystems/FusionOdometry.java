package org.firstinspires.ftc.teamcode.Subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class FusionOdometry {

    private final EZTelemetry telem;
    private final Follower follower;

    private boolean enableTelemetry;

    private OmegaPose2D currentPose;
    private Pose pedroPose;

    public FusionOdometry(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        follower = Constants.createFollower(hardwareMap);

    }

    public Pose getPedroPose() {
        return pedroPose;
    }

    public OmegaPose2D getCurrentPose() {
        return currentPose;
    }

    public double getHeading() {
        return currentPose.r();
    }

    public void zeroGyro() {
        follower.setPose(new Pose(pedroPose.getX(), pedroPose.getY(), 0));
    }

    public void setPose(OmegaPose2D newPose) {
        follower.setPose(omegaToPedro(newPose));
    }

    public void setLinearPose(OmegaPose2D newPose) {
        Pose newPedroPose = omegaToPedro(newPose);
        follower.setPose(new Pose(newPedroPose.getX(), newPedroPose.getY(), pedroPose.getHeading()));
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        follower.update();

        pedroPose = follower.getPose();
        currentPose = pedroToOmega(pedroPose);

        if(enableTelemetry) {

            telem.putLine("Odometry ");
            telem.putTelemetry("Current X Pose", currentPose.x());
            telem.putTelemetry("Current Y Pose", currentPose.y());
            telem.putTelemetry("Current Heading", currentPose.r());
            telem.putLine();

        }

    }

    public OmegaPose2D pedroToOmega(Pose pedrosPose) {
        return new OmegaPose2D(pedrosPose.getY() * -0.0254, pedrosPose.getX() * 0.0254, Math.toDegrees(pedrosPose.getHeading()));
    }

    public Pose omegaToPedro(OmegaPose2D omegasPose) {
        return new Pose(omegasPose.y() * 39.3701, omegasPose.x() * 39.3701, Math.toRadians(omegasPose.r()));
    }

}
