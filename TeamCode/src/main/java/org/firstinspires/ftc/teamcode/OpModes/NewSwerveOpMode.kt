package org.firstinspires.ftc.teamcode.OpModes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor
import org.firstinspires.ftc.teamcode.Subsystems.Swerve

@TeleOp(name = "New Swerve")
class NewSwerveOpMode : LinearOpMode() {
    override fun runOpMode() {
        val m_DriverOp = GamepadEx(gamepad1)
        val m_OperatorOp = GamepadEx(gamepad2)

        val s_Swerve = Swerve(hardwareMap, telemetry)

        val s_Sparky = OTOSSensor(hardwareMap, telemetry)

        val runtime = ElapsedTime()

        s_Sparky.configureOTOS()

        waitForStart()
        runtime.reset()

        while (opModeIsActive()) {
            m_DriverOp.readButtons()
            m_OperatorOp.readButtons()

            s_Swerve.drive(
                m_DriverOp.getLeftY(),
                m_DriverOp.getLeftX(),
                m_DriverOp.getRightX(),
                true
            )

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.START)) {
                s_Sparky.zeroGyro()
            }

            s_Swerve.update()
            telemetry.update()
        }
    }
}
