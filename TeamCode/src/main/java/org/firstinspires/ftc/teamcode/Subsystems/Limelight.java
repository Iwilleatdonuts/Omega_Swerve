package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Kalman;

import java.util.ArrayDeque;

public class Limelight {

    private final EZTelemetry telem;
    private final boolean areWeWinners;

    private final Limelight3A lime;

    private boolean enableTelemetry;

    private LLResult latestResult;

    private final Kalman bearingFilter;
    private final Kalman distanceFilter;

    private double filteredBearing;
    private double filteredDistance;

    private double adjustedBearing;

    private Pose3D limePose;

    private double cameraHeading = 0;

    private final ArrayDeque<Double> distanceWindow = new ArrayDeque<>();
    private static final int WINDOW_SIZE = 5;

//    private double smoothedDistance = 0;

    public Limelight(HardwareMap hardwareMap, EZTelemetry telem, boolean areWeWinners) {

        bearingFilter = new Kalman(1.5, 0.75, 0);
        distanceFilter = new Kalman(0.1, 0.15, 0);

        this.telem = telem;
        this.areWeWinners = areWeWinners;

        lime = hardwareMap.get(Limelight3A.class, "lime");

        lime.pipelineSwitch(areWeWinners? 0 : 1);
//        lime.pipelineSwitch(2);

        latestResult = lime.getLatestResult();

        startLime();

    }

    public void startLime() {
        lime.start();
    }

    public void stopLime() {
        lime.stop();
    }

    public LLStatus getLimeStatus() {
        return lime.getStatus();
    }

    public LLResult getLatestResult() {
        return lime.getLatestResult();
    }

    public double getGoalBearing () {
        return isValidReaing() ? latestResult.getTx() : 0;
    }

    public double getGoalDistance() {
        return isValidReaing() ? (0.31092234) / Math.tan(Math.toRadians(latestResult.getTy() + 20)) : 0;
    }

    public double getFilteredBearing() {
        return filteredBearing;
    }

    public double getFilteredDistance() {
        return filteredDistance;
    }

    public boolean isValidReaing() {
        return latestResult.isValid();
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public double getTagSkew() {
        return isValidReaing() ? latestResult.getFiducialResults().get(0).getSkew() : 0;
    }

    public void updateRobotYawFromGyro(double degrees) {
        cameraHeading = degrees;
        if(cameraHeading > 180) {
            cameraHeading -= 360;
        }
    }

    public Pose3D getLimePose() {
        return limePose != null ? limePose : new Pose3D(new Position(), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    }

//    public double getSmoothedDistance() {
//        return smoothedDistance;
//    }

    public void skadoodle() {

        latestResult = getLatestResult();

        if(isValidReaing()) {
            filteredBearing = bearingFilter.update(getGoalBearing());
            filteredDistance = distanceFilter.update(getGoalDistance());

//            distanceWindow.addLast(filteredDistance);
//
//
//            if (distanceWindow.size() > WINDOW_SIZE) {
//                distanceWindow.removeFirst();
//            }

//            double sum = 0;
//            for (double d : distanceWindow) sum += d;
//            smoothedDistance = sum / distanceWindow.size();
//            lime.updateRobotOrientation(cameraHeading);
//            limePose = latestResult.getBotpose_MT2();
        }

//        telem.putTelemetry("CAMERA HEADING", cameraHeading);

        if(enableTelemetry) {
            telem.putTelemetry("Tag Bearing: ", getGoalBearing());
            telem.putTelemetry("Tag Distance: ", getGoalDistance());
        }

    }

}
