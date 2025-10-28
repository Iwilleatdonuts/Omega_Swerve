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
import org.firstinspires.ftc.teamcode.Utilities.Kalman;
import org.firstinspires.ftc.teamcode.Utilities.KalmanTuning;
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

    private double rawBearing;
    private double rawDistance;

    private double filteredBearing;
    private double filteredDistance;

    private final Kalman bearingFilter;
    private final Kalman distanceFilter;

    private final boolean areWeWinners;


    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry, boolean areWeWinners) {

        configureAprilTagCamera(hardwareMap);

        detections = aprilTagProcessor.getDetections();

        this.telemetry = telemetry;

        this.areWeWinners = areWeWinners;

        bearingFilter = new Kalman(KalmanTuning.q1, KalmanTuning.r1, 0);
        distanceFilter = new Kalman(KalmanTuning.q2, KalmanTuning.r2, 0);

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setCameraPose(Constants.VisionConstants.poseCameraPosition, Constants.VisionConstants.poseCameraOrientation)
                .setLensIntrinsics(552.2287565089085, 549.2233357291731, 330.46847362162896, 207.9732802095237)
                .build();

        aprilCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.VisionConstants.aprilCameraName))
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

    public boolean hasGoalTag() {
        return allianceGoalTag != null;
    }

    public double getGoalBearing() {
        return allianceGoalTag != null ? filteredBearing : 0.0;
    }

    public double getGoalDistance() {
        return allianceGoalTag != null ? filteredDistance : 0;
    }

    @Override
    public void periodic() {

        packet.clearLines();

        detections = aprilTagProcessor.getDetections();
        latestDetection = null;
        allianceGoalTag = null;
        randomPatternTag = null;
        rawBearing = 0;
        rawDistance = 0;

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
                        rawBearing = allianceGoalTag.ftcPose.bearing;
                        rawDistance = allianceGoalTag.ftcPose.range;
                        filteredBearing = bearingFilter.update(rawBearing);
                        filteredDistance = distanceFilter.update(rawDistance);
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

            if(allianceGoalTag != null) {
                packet.put("Tag Skew", allianceGoalTag.ftcPose.yaw);
                packet.put("Raw Baering", rawBearing);
                packet.put("Raw Distance", rawDistance);
                packet.put("Filtered Bearing", filteredBearing);
                packet.put("Filtered Distance", filteredDistance);
            }

            telemetry.addLine("Vision");
            telemetry.addData("Alliance", areWeWinners ? "Red" : "Blue");
            telemetry.addData("Localization Tag", latestDetection != null ? latestDetection.id : "None");
            telemetry.addData("Random Pattern ID", randomPatternTag != null ? randomPatternTag.id : "None");
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

}
