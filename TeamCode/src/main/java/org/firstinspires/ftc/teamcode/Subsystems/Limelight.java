package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Kalman;

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

    public Limelight(HardwareMap hardwareMap, EZTelemetry telem, boolean areWeWinners) {

        bearingFilter = new Kalman(1.5, 0.75, 0);
        distanceFilter = new Kalman(0.1, 0.15, 0);

        this.telem = telem;
        this.areWeWinners = areWeWinners;

        lime = hardwareMap.get(Limelight3A.class, "lime");

        lime.pipelineSwitch(areWeWinners? 0 : 1);

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

    public void skadoodle() {

        latestResult = getLatestResult();

        if(isValidReaing()) {
            filteredBearing = bearingFilter.update(getGoalBearing());
            filteredDistance = distanceFilter.update(getGoalDistance());
        }

        if(enableTelemetry) {
            telem.putTelemetry("Tag Bearing: ", getGoalBearing());
            telem.putTelemetry("Tag Distance: ", getGoalDistance());
        }

    }

}
