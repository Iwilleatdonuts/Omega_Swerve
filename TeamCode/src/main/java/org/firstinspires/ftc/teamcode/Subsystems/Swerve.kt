package org.firstinspires.ftc.teamcode.Subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.Constants
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

class Swerve(hardwareMap: HardwareMap, telemetry: Telemetry) {
    private val telemetry: Telemetry?

    private val mod0: SwerveModule
    private val mod1: SwerveModule
    private val mod2: SwerveModule
    private val mod3: SwerveModule

    private val otos: OTOSSensor

    init {
        this.telemetry = telemetry

        mod0 = SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants)
        mod1 = SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants)
        mod2 = SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants)
        mod3 = SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants)

        otos = OTOSSensor(hardwareMap, telemetry)
    }

    fun drive(xVal: Double, yVal: Double, rVal: Double, fieldRelative: Boolean) {
        var x = xVal
        var y = yVal
        var r = rVal

        if (abs(x) < 0.1) {
            x = 0.0
        }
        if (abs(y) < 0.1) {
            y = 0.0
        }
        if (abs(r) < 0.1) {
            r = 0.0
        }

        if (fieldRelative) {
            val robotHeading = Math.toRadians(otos.heading)

            x = x * cos(robotHeading) + y * sin(robotHeading)
            y = -x * sin(robotHeading) + y * cos(robotHeading)
        }

        val widthVector = r * Constants.DriveTrainConstants.widthRotation
        val lengthVector = r * Constants.DriveTrainConstants.lengthRotation

        val aVector = x - widthVector
        val bVector = x + widthVector
        val cVector = y - lengthVector
        val dVector = y + lengthVector

        var mod0Speed = hypot(bVector, dVector)
        var mod1Speed = hypot(bVector, cVector)
        var mod2Speed = hypot(aVector, dVector)
        var mod3Speed = hypot(aVector, cVector)

        var max = max(abs(mod0Speed), abs(mod1Speed))
        max = max(max, abs(mod2Speed))
        max = max(max, abs(mod3Speed))

        if (max > 1.0) {
            mod0Speed /= max
            mod1Speed /= max
            mod2Speed /= max
            mod3Speed /= max
        }

        var mod0Angle = Math.toDegrees(atan2(-bVector, dVector))
        var mod1Angle = Math.toDegrees(atan2(-bVector, cVector))
        var mod2Angle = Math.toDegrees(atan2(-aVector, dVector))
        var mod3Angle = Math.toDegrees(atan2(-aVector, cVector))
        mod0Angle = (mod0Angle + 360) % 360
        mod1Angle = (mod1Angle + 360) % 360
        mod2Angle = (mod2Angle + 360) % 360
        mod3Angle = (mod3Angle + 360) % 360

        mod0.setDrivePower(mod0Speed)
        mod1.setDrivePower(mod1Speed)
        mod2.setDrivePower(mod2Speed)
        mod3.setDrivePower(mod3Speed)

        if (abs(xVal) > 0.1 || abs(yVal) > 0.1 || abs(rVal) > 0.1) {
            mod0.setModuleSetpoint(mod0Angle)
            mod1.setModuleSetpoint(mod1Angle)
            mod2.setModuleSetpoint(mod2Angle)
            mod3.setModuleSetpoint(mod3Angle)
        }

        mod0.setModulePosition()
        mod1.setModulePosition()
        mod2.setModulePosition()
        mod3.setModulePosition()
    }

    fun update() {
        telemetry!!.addLine("Swerve")
        telemetry.addData("X Position ", otos.pose?.x)
        telemetry.addData("Y Position ", otos.pose?.y)
        telemetry.addData("Heading ", otos.heading)
        telemetry.addData("OTOS Heading ", otos.pose?.h)
        telemetry.addLine()
    }
}
