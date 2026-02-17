package org.firstinspires.ftc.teamcode.Subsystems;


import com.arcrobotics.ftclib.geometry.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.PedroPathing.OmegaCoolConstants;
import org.firstinspires.ftc.teamcode.Utilities.PedroPathing.OmegaCoolLocalizer;

public class FusionOdometry {

    private final EZTelemetry telem;
    private final OmegaCoolLocalizer localizer;

    private boolean enableTelemetry;

    private OmegaPose2D currentPose;
    private Pose pedroPose;

    public FusionOdometry(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

//        follower = Constants.createFollower(hardwareMap);
        localizer = new OmegaCoolLocalizer(hardwareMap, new OmegaCoolConstants(), new Pose());

    }

    public Pose getPedroPose() {
        return pedroPose;
    }

    public OmegaPose2D getCurrentPose() {
        return currentPose;
    }

    public Vector2d getFieldRelativeVelocity() {
        return localizer.getVelocityVector2d();
    }

    public double getAngularVelocity() {
        return localizer.getVelocity().getHeading();
    }

    public double getDistanceFromTarget(boolean areWeWinners) {

        OmegaPose2D targetPose = areWeWinners ? Constants.TurretConstants.redTarget : Constants.TurretConstants.blueTarget;

        return Math.hypot(getCurrentPose().x() - targetPose.x(), getCurrentPose().y() - targetPose.y());
    }

    public double getHeading() {
        return currentPose.r();
    }

    public double getReversedHeading() {
        double reversed = getHeading();
        reversed += 180;
        if(reversed > 360) {
            reversed -= 360;
        }
        return reversed;
    }

    public void zeroGyro() {
        localizer.setPose(new Pose(pedroPose.getX(), pedroPose.getY(), Math.toRadians(90)));
    }

    public void setPose(OmegaPose2D newPose) {
        localizer.setPose(omegaToPedro(newPose));
    }

    public void setLinearPose(OmegaPose2D newPose) {
        Pose newPedroPose = omegaToPedro(newPose);
        localizer.setPose(new Pose(newPedroPose.getX(), newPedroPose.getY(), pedroPose.getHeading()));
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        localizer.update();

        pedroPose = localizer.getPose();
        currentPose = pedroToOmega(pedroPose);

        if(enableTelemetry) {

            telem.putLine("Odometry ");
            telem.putTelemetry("Current X Pose", currentPose.x());
            telem.putTelemetry("Current Y Pose", currentPose.y());
            telem.putTelemetry("Current Heading", getHeading());
            telem.putLine();

            telem.putLine("Pedro ");
            telem.putTelemetry("Pedro X Pose", pedroPose.getX());
            telem.putTelemetry("Pedro Y Pose", pedroPose.getY());
            telem.putTelemetry("Pedro Heading", Math.toDegrees(getPedroPose().getHeading()));
            telem.putLine();

        }
    }

    public OmegaPose2D pedroToOmega(Pose pedrosPose) {
        double heading = Math.toDegrees(pedrosPose.getHeading());
        heading -= 90;
        heading = (heading + 360) % 360;
        return new OmegaPose2D(pedrosPose.getX() * 0.0254, pedrosPose.getY() * 0.0254, heading);
    }

    public Pose omegaToPedro(OmegaPose2D omegasPose) {
        return new Pose(omegasPose.x(), omegasPose.y(), omegasPose.r());
//        double heading = Math.toRadians(omegasPose.r());
//        heading += (Math.PI/2);
//        if(heading > Math.PI) {
//            heading -= Math.PI*2;
//        }
//        if(heading < -Math.PI) {
//            heading += Math.PI*2;
//        }
//        return new Pose(omegasPose.x() * 39.3701, omegasPose.y() * 39.3701, heading);
    }

}
