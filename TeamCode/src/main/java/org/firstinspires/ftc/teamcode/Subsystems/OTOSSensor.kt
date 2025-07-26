package org.firstinspires.ftc.teamcode.Subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.Constants

class OTOSSensor(hardwareMap: HardwareMap, private val telemetry: Telemetry) : SubsystemBase() {
    private val otos: SparkFunOTOS

    private var isReady = false

    private val imu: IMU

    init {
        otos = hardwareMap.get<SparkFunOTOS>(
            SparkFunOTOS::class.java,
            Constants.DriveTrainConstants.OTOS.sparkfun
        )

        val logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.DOWN
        val usbDirection = UsbFacingDirection.BACKWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logoDirection, usbDirection)

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get<IMU>(IMU::class.java, "imu")
        imu.initialize(IMU.Parameters(orientationOnRobot))
    }

    fun configureOTOS() {
        telemetry.addLine("OTOS is Ready: " + isReady)
        telemetry.update()

        otos.setLinearUnit(DistanceUnit.MM)
        otos.setAngularUnit(AngleUnit.DEGREES)

        // left and right is - + x
        // back and forth is - + y
        // counter clockwise is positive degrees
        val offset = Constants.DriveTrainConstants.OTOS.sensorOffset
        otos.setOffset(offset)

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
        otos.setLinearScalar(1.0)
        otos.setAngularScalar(1.0)

        otos.calibrateImu()

        otos.resetTracking()

        val currentPosition = SparkFunOTOS.Pose2D(0.0, 0.0, 0.0)
        otos.setPosition(currentPosition)

        val hwVersion = SparkFunOTOS.Version()
        val fwVersion = SparkFunOTOS.Version()
        otos.getVersionInfo(hwVersion, fwVersion)

        isReady = true
        telemetry.addLine("OTOS is Ready: " + isReady)
        telemetry.update()
        //        telemetry.addLine();
//        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
//        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
    }

    val pose: SparkFunOTOS.Pose2D?
        get() = otos.getPosition()

    val heading: Double
        get() {
            var rotation = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES)

            rotation = (rotation + 360) % 360

            return rotation
        }

    fun zeroGyro() {
        imu.resetYaw()
    }

    val velocity: SparkFunOTOS.Pose2D?
        get() = otos.getVelocity()

    val poseSTD: SparkFunOTOS.Pose2D?
        get() = otos.getPositionStdDev()

    val velocitySTD: SparkFunOTOS.Pose2D?
        get() = otos.getVelocityStdDev()

    fun update() {
        telemetry.addLine("OTOS")
        telemetry.addData("X Position \t", this.pose!!.x)
        telemetry.addData("Y Position \t", this.pose!!.y)
        telemetry.addData("Rotation \t", this.heading)
        telemetry.addData("X Velocity \t", this.velocity!!.x)
        telemetry.addData("Y Velocity \t", this.velocity!!.y)
        telemetry.addData("R Velocity \t", this.velocity!!.h)
        telemetry.addLine()
    }
}
