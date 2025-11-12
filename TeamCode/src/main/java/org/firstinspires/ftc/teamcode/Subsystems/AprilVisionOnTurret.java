package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Kalman;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilVisionOnTurret {

    private final EZTelemetry telem;
    private VisionPortal aprilCamera;
    private AprilTagProcessor aprilTagProcessor;
    private boolean enableTelemetry;

    private AprilTagDetection latestDetection;
    private AprilTagDetection allianceGoalTag;
    private AprilTagDetection randomPatternTag;
    private List<AprilTagDetection> detections;

    private double rawBearing;
    private double rawDistance;

    private double adjustedBearing;

    private double goalYaw;

    private double filteredBearing;
    private double filteredDistance;

    private final Kalman bearingFilter;
    private final Kalman distanceFilter;

    private final boolean areWeWinners;


    public AprilVisionOnTurret(HardwareMap hardwareMap, EZTelemetry telem, boolean areWeWinners) {

        configureAprilTagCamera(hardwareMap);

        detections = aprilTagProcessor.getDetections();

        this.telem = telem;

        this.areWeWinners = areWeWinners;
//
//        bearingFilter = new Kalman(KalmanTuning.q1, KalmanTuning.r1, 0);
//        distanceFilter = new Kalman(KalmanTuning.q2, KalmanTuning.r2, 0);
        bearingFilter = new Kalman(1.5, 0.75, 0);
        distanceFilter = new Kalman(0.1, 0.15, 0);

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .setLensIntrinsics(552.2287565089085, 549.2233357291731, 330.46847362162896, 207.9732802095237)
                .setLensIntrinsics(560.210886526574,568.5837973643032,341.4406064182799,174.0920003890196)
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
        return allianceGoalTag != null ? rawBearing : 0.0;
    }

    public double getGoalYaw() {
        return allianceGoalTag != null ? goalYaw : 0.0;
    }

    public double getAdjustedGoalBearing() {
        return allianceGoalTag != null ? adjustedBearing : 0;
    }

    public double getCameraFPS() {
        return aprilCamera != null ? aprilCamera.getFps(): 0;
    }

    public boolean isCameraOn() {
        if(aprilCamera!= null) {
            if(aprilCamera.getFps() != 0) {
                return true;
            } else {
                return false;
            }
        } else {
            return  false;
        }
    }


    public double getGoalDistance() {
        return allianceGoalTag != null ? rawDistance : 0;
    }

    private void setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
//
//        // Wait for the camera to be open
        if (aprilCamera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (aprilCamera.getCameraState() != VisionPortal.CameraState.STREAMING) {
                System.out.println("I am currentl erroring because my silly stream is off");
            }
        }

        if(aprilCamera != null) {
            ExposureControl exposureControl = aprilCamera.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

            // Set Gain.
            GainControl gainControl = aprilCamera.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

        }
    }

    public void skadoodle() {

        detections = aprilTagProcessor.getDetections();
        latestDetection = null;
        allianceGoalTag = null;
        randomPatternTag = null;
        rawBearing = 0;
        rawDistance = 0;
        goalYaw = 0;

        if(!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                int id = tag.id;

                // localization stuff only with good tags
                if (id == 20 || id == 24) {
                    latestDetection = tag;

                    //find alliance tag
                    if ((areWeWinners && id == 24) || (!areWeWinners && id == 20)) {
                        try{
                            allianceGoalTag = tag;
                            rawBearing = allianceGoalTag.ftcPose.bearing;
                            rawDistance = allianceGoalTag.ftcPose.range;
                            filteredBearing = bearingFilter.update(rawBearing);
                            filteredDistance = distanceFilter.update(rawDistance);
                            goalYaw = allianceGoalTag.ftcPose.yaw;
                            adjustedBearing = getGoalBearing() + (getGoalYaw() * 0.133333333333);
                        } catch(Exception ignored) {

                        }
                    }
                } else if (id >= 21 && id <= 23) {
                    randomPatternTag = tag;
                }
            }
        }
        telem.putTelemetry("FPS", getCameraFPS());
        telem.putTelemetry("Camera State", isCameraOn());

        telem.putDashboard("FPS", getCameraFPS());
        telem.putDashboard("Camera State", isCameraOn());

    }

}
