package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Limelight {

    private final EZTelemetry telem;
    private final boolean areWeWinners;

    private final Limelight3A lime;

    private boolean enableTelemetry;

    private LLResult latestResult;

    public Limelight(HardwareMap hardwareMap, EZTelemetry telem, boolean areWeWinners) {

        this.telem = telem;
        this.areWeWinners = areWeWinners;

        lime = hardwareMap.get(Limelight3A.class, "lime");

        lime.pipelineSwitch(areWeWinners? 0 : 1);

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

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle() {

        latestResult = getLatestResult();

        if(enableTelemetry) {
            telem.putTelemetry("Tag Bearing: ", latestResult.getTx());
        }

    }

}
