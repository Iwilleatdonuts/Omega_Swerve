package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilVision extends SubsystemBase {

    private VisionPortal aprilCamera;
    private AprilTagProcessor aprilTagProcessor;

    private final Telemetry telemetry;
    private boolean enableTelemetry;

    private AprilTagDetection latestDetection;
    private AprilTagDetection allianceGoalTag;
    private AprilTagDetection randomPatternTag;

    private final TelemetryPacket packet = new TelemetryPacket();
    private List<AprilTagDetection> detections;
    private Pose3D currentPose;

    private final boolean areWeWinners;


    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry, boolean areWeWinners) {

        configureAprilTagCamera(hardwareMap);

        detections = aprilTagProcessor.getDetections();

        this.telemetry = telemetry;

        this.areWeWinners = areWeWinners;

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
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

    public boolean hasTag() {
        return latestDetection != null;
    }

    public double getGoalBearing() {
        return allianceGoalTag != null ? allianceGoalTag.ftcPose.bearing : 0.0;
    }

    public int getObeliskTarget() {
        return
    }

    @Override
    public void periodic() {

        packet.clearLines();

        detections = aprilTagProcessor.getDetections();
        latestDetection = null;
        allianceGoalTag = null;
        randomPatternTag = null;

        if(!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                int id = tag.id;

                // localization stuff only with good tags
                if (id == 20 || id == 24) {
                    latestDetection = tag;
                    currentPose = tag.robotPose;

                    //find alliance tag
                    if ((areWeWinners && id == 24) || (!areWeWinners && id == 20)) {
                        allianceGoalTag = tag;
                    }
                } else if (id >= 21 && id <= 23) {
                    randomPatternTag = tag;
                }
            }
        }

        if (enableTelemetry) {
            packet.clearLines();

            if (latestDetection != null) {
                packet.put("Tag ID", latestDetection.id);
                packet.put("Robot X Pose", currentPose.getPosition().x);
                packet.put("Robot Y Pose", currentPose.getPosition().y);
                packet.put("Robot Yaw", currentPose.getOrientation().getYaw());
            } else {
                packet.addLine("No Localization Tags");
            }

            if (randomPatternTag != null) {
                packet.put("Random Pattern ID", randomPatternTag.id);
            }

            telemetry.addLine("Vision");
            telemetry.addData("Alliance", areWeWinners ? "Red" : "Blue");
            telemetry.addData("Localization Tag", latestDetection != null ? latestDetection.id : "None");
            telemetry.addData("Random Pattern ID", randomPatternTag != null ? randomPatternTag.id : "None");
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
