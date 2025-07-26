package org.firstinspires.ftc.teamcode.OpModes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.sin

@TeleOp(name = "Janky Swerve")
class SwerveOpMode : LinearOpMode() {
    override fun runOpMode() {
        val m_DriverOp = GamepadEx(gamepad1)
        val m_OperatorOp = GamepadEx(gamepad2)

        val s_Mod0 =
            SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants)
        val s_Mod1 =
            SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants)
        val s_Mod2 =
            SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants)
        val s_Mod3 =
            SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants)

        val s_Sparky = OTOSSensor(hardwareMap, telemetry)

        val runtime = ElapsedTime()

        s_Sparky.configureOTOS()

        val hyp = hypot(
            Constants.DriveTrainConstants.trackWidth / 2,
            Constants.DriveTrainConstants.wheelbase / 2
        )

        waitForStart()
        runtime.reset()


        while (opModeIsActive()) {
            m_DriverOp.readButtons()
            m_OperatorOp.readButtons()

            var xJoy = m_DriverOp.getLeftX()
            var yJoy = m_DriverOp.getLeftY()

            if (abs(xJoy) < 0.1) {
                xJoy = 0.0
            }
            if (abs(yJoy) < 0.1) {
                yJoy = 0.0
            }

            val robotHeading = Math.toRadians(s_Sparky.heading)

            val x = xJoy * cos(robotHeading) + yJoy * sin(robotHeading)
            val y = -xJoy * sin(robotHeading) + yJoy * cos(robotHeading)

            var r = -m_DriverOp.getRightX()

            if (abs(r) < 0.1) {
                r = 0.0
            }

            val rotVec = r * (Constants.DriveTrainConstants.wheelbase / hyp)

            val aVec = x - rotVec
            val bVec = x + rotVec
            val cVec = y - rotVec
            val dVec = y + rotVec

            var mod0Speed = hypot(bVec, dVec)
            var mod1Speed = hypot(bVec, cVec)
            var mod2Speed = hypot(aVec, dVec)
            var mod3Speed = hypot(aVec, cVec)

            var max = max(abs(mod0Speed), abs(mod1Speed))
            max = max(max, abs(mod2Speed))
            max = max(max, abs(mod3Speed))

            if (max > 1.0) {
                mod0Speed /= max
                mod1Speed /= max
                mod2Speed /= max
                mod3Speed /= max
            }

            var mod0Angle = Math.toDegrees(atan2(-bVec, dVec))
            var mod1Angle = Math.toDegrees(atan2(-bVec, cVec))
            var mod2Angle = Math.toDegrees(atan2(-aVec, dVec))
            var mod3Angle = Math.toDegrees(atan2(-aVec, cVec))
            mod0Angle = (mod0Angle + 360) % 360
            mod1Angle = (mod1Angle + 360) % 360
            mod2Angle = (mod2Angle + 360) % 360
            mod3Angle = (mod3Angle + 360) % 360

            if (s_Mod0.atRoughSepoint() && s_Mod1.atRoughSepoint() && s_Mod2.atRoughSepoint() && s_Mod3.atRoughSepoint()) {
                s_Mod0.setDrivePower(mod0Speed)
                s_Mod1.setDrivePower(mod1Speed)
                s_Mod2.setDrivePower(mod2Speed)
                s_Mod3.setDrivePower(mod3Speed)
            }

            if (abs(m_DriverOp.getLeftY()) > 0.1 || abs(m_DriverOp.getRightX()) > 0.1 || abs(
                    m_DriverOp.getLeftX()
                ) > 0.1
            ) {
                s_Mod0.setModuleSetpoint(mod0Angle)
                s_Mod1.setModuleSetpoint(mod1Angle)
                s_Mod2.setModuleSetpoint(mod2Angle)
                s_Mod3.setModuleSetpoint(mod3Angle)
            }

            s_Mod0.setModulePosition()
            s_Mod1.setModulePosition()
            s_Mod2.setModulePosition()
            s_Mod3.setModulePosition()

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.START)) {
                s_Sparky.zeroGyro()
            }

            s_Mod0.update()
            s_Mod1.update()
            s_Mod2.update()
            s_Mod3.update()
            s_Sparky.update()
            //TODO make sure that the heading is 0-360, CCW position, 0 is forwards
            telemetry.addData(
                "Left Joystick Angle \t",
                Math.toDegrees(atan2(-m_DriverOp.getLeftX(), m_DriverOp.getLeftY()))
            )
            telemetry.addData("Right X \t", m_DriverOp.getRightX())
            telemetry.update()
        }
    }
}
