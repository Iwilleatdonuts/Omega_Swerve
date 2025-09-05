package org.firstinspires.ftc.teamcode.Subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class AprilVision extends SubsystemBase {

    private VisionPortal aprilCamera;
    private AprilTagProcessor aprilTagProcessor;

    private final Telemetry telemetry;

    public AprilVision(HardwareMap hardwareMap, Telemetry telemetry) {

        configureAprilTagCamera(hardwareMap);

        this.telemetry = telemetry;

    }

    public void configureAprilTagCamera(HardwareMap hardwareMap) {

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .setCameraPose(Constants.Vision.aprilTagCameraPosition, Constants.Vision.aprilTagCameraOrientation)
                .build();

        aprilCamera = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, Constants.Vision.AprilTagCameraName))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(320, 240))
                .enableLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();

    }

    public void update() {
        if (!aprilTagProcessor.getDetections().isEmpty()) {
            telemetry.addData("Detected Tags", aprilTagProcessor.getDetections().size());
            telemetry.addData("First Tag ID", aprilTagProcessor.getDetections().get(0).id);
            telemetry.addData("X (in)", aprilTagProcessor.getDetections().get(0).ftcPose.x);
            telemetry.addData("Y (in)", aprilTagProcessor.getDetections().get(0).ftcPose.y);
            telemetry.addData("Yaw (deg)", aprilTagProcessor.getDetections().get(0).ftcPose.yaw);
        } else {
            telemetry.addLine("No AprilTags detected");
        }
    }


}
