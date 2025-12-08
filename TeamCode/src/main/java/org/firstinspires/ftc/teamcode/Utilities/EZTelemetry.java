package org.firstinspires.ftc.teamcode.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;

import java.util.HashMap;
import java.util.Map;

public class EZTelemetry {

    private final FtcDashboard dashboard;
    private final Telemetry telemetry;
    private final Map<String, Object> dashboardData = new HashMap<>();
    private final Map<String, Object> telemetryData = new HashMap<>();

    public EZTelemetry(Telemetry telemetry) {

        this.dashboard = FtcDashboard.getInstance();
        this.telemetry = telemetry;

    }

    public void putTelemetry(String key, Object value) {
        telemetry.addData(key, value);
    }

    public void putLine(String line) {
        telemetry.addLine(line);
    }

    public void putLine() {
        telemetry.addLine();
    }

    public void enableCameraStrea(CameraStreamSource source, double fps) {
        dashboard.startCameraStream(source, fps);
    }

    public void updateTelemetry() {
//        telemetry.clearAll();
        for(Map.Entry<String, Object> entry : telemetryData.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
        telemetry.update();
        telemetryData.clear();
    }

    public void putDashboard(String key, Object value) {
        dashboardData.put(key, value);
    }

    public void updateDashboard() {
        TelemetryPacket packet = new TelemetryPacket();
        for (Map.Entry<String, Object> entry : dashboardData.entrySet()) {
            packet.put(entry.getKey(), entry.getValue());
        }
        dashboard.sendTelemetryPacket(packet);
        dashboardData.clear();
    }

    public void updateAll() {
        updateTelemetry();
        updateDashboard();
    }
}