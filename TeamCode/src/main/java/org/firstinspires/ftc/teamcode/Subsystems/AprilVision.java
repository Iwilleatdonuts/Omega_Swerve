package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilVision extends SubsystemBase {

    private VisionPortal aprilCamera;
    private AprilTagProcessor aprilTagProcessor;

    private final Telemetry telemetry;
    private boolean enableTelemetry;

    private AprilTagDetection latestDetection;

    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry) {

        configureAprilTagCamera(hardwareMap);

        this.telemetry = telemetry;

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(Constants.VisionConstants.poseCameraPosition, Constants.VisionConstants.poseCameraOrientation)
                .setLensIntrinsics(552.2287565089085, 549.2233357291731, 330.46847362162896, 207.9732802095237)
                .build();

        aprilCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.VisionConstants.poseCameraName))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(false)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public VisionPortal getAprilCamera() {
        return aprilCamera;
    }

    public void periodic(FtcDashboard dashboard) {

        if (!aprilTagProcessor.getDetections().isEmpty()) {
            latestDetection = aprilTagProcessor.getDetections().get(0);
        } else {
            latestDetection = null;
        }

        if(enableTelemetry) {
            TelemetryPacket packet = new TelemetryPacket();
            if (latestDetection != null) {
                packet.addLine("Tag ID: " + latestDetection.id);
                packet.addLine("Robot X Pose: " + latestDetection.robotPose.getPosition().x);
                packet.addLine("Robot Y Pose: " + latestDetection.robotPose.getPosition().y);
                packet.addLine("Robot Yaw: " + latestDetection.robotPose.getOrientation().getYaw());

                telemetry.addLine("Vision");
                telemetry.addData("Robot X Pose:", latestDetection.robotPose.getPosition().x);
                telemetry.addData("Robot Y Pose:", latestDetection.robotPose.getPosition().y);
                telemetry.addData("Robot Yaw:", latestDetection.robotPose.getOrientation().getYaw());
            } else {
                packet.addLine("No AprilTags detected");
            }
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public boolean hasTag() {
        return latestDetection != null;
    }

    public double getAprilX() {
        return latestDetection != null ? latestDetection.ftcPose.x : 0.0;
    }

    public double getAprilY() {
        return latestDetection != null ? latestDetection.ftcPose.y : 0.0;
    }

    public double getAprilZ() {
        return latestDetection != null ? latestDetection.ftcPose.z : 0.0;
    }

    public double getAprilYaw() {
        return latestDetection != null ? latestDetection.ftcPose.yaw : 0.0;
    }

    public double getAprilPitch() {
        return latestDetection != null ? latestDetection.ftcPose.pitch : 0.0;
    }

    public double getAprilRoll() {
        return latestDetection != null ? latestDetection.ftcPose.roll : 0.0;
    }

    public double getAprilRange() {
        return latestDetection != null ? latestDetection.ftcPose.range : 0.0;
    }

    public double getAprilBearing() {
        return latestDetection != null ? latestDetection.ftcPose.bearing : 0.0;
    }

    public double getAprilElevation() {
        return latestDetection != null ? latestDetection.ftcPose.elevation : 0.0;
    }

}
