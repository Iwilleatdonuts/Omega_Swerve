package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.Pose2D;

public class OTOSSensor {

    private final EZTelemetry telem;

    private final SparkFunOTOS otos;

    private boolean isReady;

    private boolean enableTelemetry;

    private volatile boolean disabled = false;

    public OTOSSensor(HardwareMap hardwareMap, EZTelemetry telem){

        this.telem = telem;

        otos = hardwareMap.get(SparkFunOTOS.class, Constants.DriveTrainConstants.OTOS.sparkfun);

        isReady = false;

        enableTelemetry = false;

    }

    public void disable() {
        disabled = true;
    }

    public void configureOTOS(SparkFunOTOS.Pose2D currentPose) {
        if (disabled) return;

        otos.setLinearUnit(DistanceUnit.METER);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // left and right is - + x
        // back and forth is - + y
        // counter clockwise is positive degrees


        SparkFunOTOS.Pose2D offset = Constants.DriveTrainConstants.OTOS.sensorOffset;
        otos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        otos.setLinearScalar(1.03562700333);
        otos.setAngularScalar(0.99936984179);

        otos.calibrateImu();

        otos.resetTracking();

        if (disabled) return;

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(currentPose.x, currentPose.y, currentPose.h);
        otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        isReady = true;
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    }

    public Pose2D getPose() {
        if (disabled) return new Pose2D(0,0,0);
        SparkFunOTOS.Pose2D otosPose = otos.getPosition();
        double heading = otosPose.h;
        heading = (heading + 360) % 360;
        return new Pose2D(otosPose.x, otosPose.y, heading);
    }

    public SparkFunOTOS.Pose2D normiePoseToSparkyPose(Pose2D normiePose) {
        double heading = normiePose.r();
        if(heading > 180) {
            heading-=360;
        }
        return new SparkFunOTOS.Pose2D(normiePose.x(), normiePose.y(), heading);
    }

    public double getHeading() {
        if (disabled) return 0;
        double rotation = otos.getPosition().h;

        rotation = (rotation+360)%360;

        return rotation;
    }

    public void zeroGyro() {
        if (disabled) return;
        otos.setPosition(new SparkFunOTOS.Pose2D(otos.getPosition().x, otos.getPosition().y, 0));
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        if (disabled || !enableTelemetry) return;
            telem.putTelemetry("X Position \t", getPose().x());
            telem.putTelemetry("Y Position \t", getPose().y());
            telem.putTelemetry("OTOS Rotation \t", getHeading());

            telem.putDashboard("X Position \t", getPose().x());
            telem.putDashboard("Y Position \t", getPose().y());
            telem.putDashboard("OTOS Rotation \t", getHeading());

    }

}
