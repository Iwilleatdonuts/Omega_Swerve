package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class OTOSSensor extends SubsystemBase {

    private final Telemetry telemetry;

    private final SparkFunOTOS otos;

    private boolean isReady;

    public OTOSSensor(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;

        otos = hardwareMap.get(SparkFunOTOS.class, Constants.DriveTrainConstants.sparkfun);

        isReady = false;

    }

    public void configureOTOS() {

        telemetry.addLine("OTOS is Ready: " + isReady);
        telemetry.update();

        otos.setLinearUnit(DistanceUnit.MM);
        otos.setAngularUnit(AngleUnit.DEGREES);

        // left and right is - + x
        // back and forth is - + y
        // counter clockwise is positive degrees

        //Currently the offset is in mm, if the offset seems weird then try setting all units to meters instead of mm
        //also if driving is backwards, change the h to 180
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, -48, 0);
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
        otos.setLinearScalar(1.0);
        otos.setAngularScalar(1.0);

        otos.calibrateImu();

        otos.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);

        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        otos.getVersionInfo(hwVersion, fwVersion);

        isReady = true;
        telemetry.addLine("OTOS is Ready: " + isReady);
        telemetry.update();
//        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    }

    public SparkFunOTOS.Pose2D getPose() {
        return otos.getPosition();
    }

    public SparkFunOTOS.Pose2D getVelocity() {
        return otos.getVelocity();
    }

    public SparkFunOTOS.Pose2D getPoseSTD() {
        return otos.getPositionStdDev();
    }

    public SparkFunOTOS.Pose2D getVelocitySTD() {
        return otos.getVelocityStdDev();
    }

    public void update(){
        telemetry.addLine("OTOS");
        telemetry.addData("X Position: ", getPose().x);
        telemetry.addData("Y Position: ", getPose().y);
        telemetry.addData("Rotation: ", getPose().h);
        telemetry.addData("X Velocity: ", getVelocity().x);
        telemetry.addData("Y Velocity: ", getVelocity().y);
        telemetry.addData("R Velocity: ", getVelocity().h);
    }

}
