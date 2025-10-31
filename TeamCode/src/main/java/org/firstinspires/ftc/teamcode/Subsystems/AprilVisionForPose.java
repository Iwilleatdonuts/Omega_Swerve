package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.Kalman;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilVisionForPose extends SubsystemBase {

    private VisionPortal aprilCamera;
    private AprilTagProcessor aprilTagProcessor;

    private final Telemetry telemetry;
    private boolean enableTelemetry;

    private AprilTagDetection latestDetection;
    private AprilTagDetection randomPatternTag;

    private List<AprilTagDetection> detections;

    private SparkFunOTOS.Pose2D rawPose;
    private SparkFunOTOS.Pose2D filteredPose;

    private final Kalman xFilter;
    private final Kalman yFilter;
    private final Kalman hFilter;

    public AprilVisionForPose(HardwareMap hardwareMap, Telemetry telemetry) {

        configureAprilTagCamera(hardwareMap);

        detections = aprilTagProcessor.getDetections();

        this.telemetry = telemetry;

//
//        bearingFilter = new Kalman(KalmanTuning.q1, KalmanTuning.r1, 0);
//        distanceFilter = new Kalman(KalmanTuning.q2, KalmanTuning.r2, 0);
        xFilter = new Kalman(1.5, 0.75, 0);
        yFilter = new Kalman(1.5, 0.75, 0);
        hFilter = new Kalman(1.5, 0.75, 0);

        rawPose = new SparkFunOTOS.Pose2D(0, 0, 0);
        filteredPose = rawPose;

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawTagOutline(false)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setLensIntrinsics(552.2287565089085, 549.2233357291731, 330.46847362162896, 207.9732802095237)
                .setOutputUnits(DistanceUnit.METER, AngleUnit.DEGREES)
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

    public SparkFunOTOS.Pose2D getRawPose() {
        return rawPose;
    }

    public SparkFunOTOS.Pose2D getFilteredPose() {
        return filteredPose;
    }

    public double getCameraFPS() {
        return aprilCamera != null ? aprilCamera.getFps(): 0;
    }


    private void setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
//
//        // Wait for the camera to be open
        if (aprilCamera.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (aprilCamera.getCameraState() != VisionPortal.CameraState.STREAMING) {
                System.out.println("I am currentl erroring because my silly stream is off");
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
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


    @Override
    public void periodic() {

        detections = aprilTagProcessor.getDetections();
        latestDetection = null;
        randomPatternTag = null;

        if(!detections.isEmpty()) {
            for (AprilTagDetection tag : detections) {
                int id = tag.id;

                // localization stuff only with good tags
                if (id == 20 || id == 24) {
                    latestDetection = tag;

                    double rawX = latestDetection.robotPose.getPosition().x;
                    double rawY = latestDetection.robotPose.getPosition().y;
                    double rawH = latestDetection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    rawPose = new SparkFunOTOS.Pose2D(rawX, rawY, rawH);

                    double filteredX = xFilter.update(rawX);
                    double filteredY = yFilter.update(rawY);
                    double filteredH = hFilter.update(rawH);

                    filteredPose = new SparkFunOTOS.Pose2D(filteredX, filteredY, filteredH);

                } else if (id >= 21 && id <= 23) {
                    randomPatternTag = tag;
                }
            }
        }
        telemetry.addData("FPS", getCameraFPS());
        telemetry.addData("Robot X", getFilteredPose().x);
        telemetry.addData("Robot Y", getFilteredPose().y);
        telemetry.addData("Robot H", getFilteredPose().h);
        telemetry.update();
    }


}
