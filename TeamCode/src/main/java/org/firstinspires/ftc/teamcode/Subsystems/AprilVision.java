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

    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry) {

        configureAprilTagCamera(hardwareMap);

        this.telemetry = telemetry;

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(Constants.VisionConstants.aprilTagCameraPosition, Constants.VisionConstants.aprilTagCameraOrientation)
                .build();

        aprilCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.VisionConstants.AprilTagCameraName))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(320, 240))
                .enableLiveView(true)
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

        if(enableTelemetry) {
            if (!aprilTagProcessor.getDetections().isEmpty()) {
                for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Tag X Pose", detection.ftcPose.x);
                    telemetry.addData("Tag Y Pose", detection.ftcPose.y);
                    telemetry.addData("Tag Z Pose", detection.ftcPose.z);
                    telemetry.addData("Tag Yaw", detection.ftcPose.yaw);
                    telemetry.addData("Tag Pitch", detection.ftcPose.pitch);
                    telemetry.addData("Tag Roll", detection.ftcPose.roll);
                    telemetry.addData("Tag Elevation", detection.ftcPose.elevation);
                    telemetry.addData("Tag Bearing", detection.ftcPose.bearing);
                    telemetry.addData("Tag Range", detection.ftcPose.range);
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
        }

        TelemetryPacket packet = new TelemetryPacket();

        if (!aprilTagProcessor.getDetections().isEmpty()) {
                for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                packet.addLine("Tag ID: " + detection.id);
//                packet.addLine("Tag X Pose: " + detection.ftcPose.x);
//                packet.addLine("Tag Y Pose: " + detection.ftcPose.y);
//                packet.addLine("Tag Z Pose: " + detection.ftcPose.z);
//                packet.addLine("Tag Yaw: " + detection.ftcPose.yaw);
//                packet.addLine("Tag Roll: " + detection.ftcPose.elevation);
//                packet.addLine("Tag Pitch: " + detection.ftcPose.pitch);
//                packet.addLine("Tag Roll: " + detection.ftcPose.roll);
//                packet.addLine("Tag Elevation: " + detection.ftcPose.elevation);
//                packet.addLine("Tag Bearing: " + detection.ftcPose.bearing);
//                packet.addLine("Tag Range: " + detection.ftcPose.range);
                packet.addLine("Robot X Pose: " + detection.robotPose.getPosition().x);
                packet.addLine("Robot Y Pose: " + detection.robotPose.getPosition().y);
                packet.addLine("Robot Yaw: " + detection.robotPose.getOrientation().getYaw());
            }
        }

        dashboard.sendTelemetryPacket(packet);
    }

}
