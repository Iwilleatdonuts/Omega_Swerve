package org.firstinspires.ftc.teamcode.OpModes

import com.arcrobotics.ftclib.gamepad.GamepadEx
import com.arcrobotics.ftclib.gamepad.GamepadKeys
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule

@TeleOp(name = "Swerve Module")
class SwerveModuleTestOpMode : LinearOpMode() {
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

        waitForStart()
        runtime.reset()


        while (opModeIsActive()) {
            m_DriverOp.readButtons()
            m_OperatorOp.readButtons()

            s_Mod0.setDrivePower(m_DriverOp.getLeftY())
            s_Mod1.setDrivePower(m_DriverOp.getLeftY())
            s_Mod2.setDrivePower(m_DriverOp.getLeftY())
            s_Mod3.setDrivePower(m_DriverOp.getLeftY())

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                if (m_DriverOp.isDown(GamepadKeys.Button.A)) {
                    s_Mod0.setModuleSetpoint(0.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.B)) {
                    s_Mod1.setModuleSetpoint(0.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.X)) {
                    s_Mod2.setModuleSetpoint(0.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.Y)) {
                    s_Mod3.setModuleSetpoint(0.0)
                }
            }

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (m_DriverOp.isDown(GamepadKeys.Button.A)) {
                    s_Mod0.setModuleSetpoint(90.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.B)) {
                    s_Mod1.setModuleSetpoint(90.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.X)) {
                    s_Mod2.setModuleSetpoint(90.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.Y)) {
                    s_Mod3.setModuleSetpoint(90.0)
                }
            }

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                if (m_DriverOp.isDown(GamepadKeys.Button.A)) {
                    s_Mod0.setModuleSetpoint(180.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.B)) {
                    s_Mod1.setModuleSetpoint(180.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.X)) {
                    s_Mod2.setModuleSetpoint(180.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.Y)) {
                    s_Mod3.setModuleSetpoint(180.0)
                }
            }

            if (m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (m_DriverOp.isDown(GamepadKeys.Button.A)) {
                    s_Mod0.setModuleSetpoint(270.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.B)) {
                    s_Mod1.setModuleSetpoint(270.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.X)) {
                    s_Mod2.setModuleSetpoint(270.0)
                }
                if (m_DriverOp.isDown(GamepadKeys.Button.Y)) {
                    s_Mod3.setModuleSetpoint(270.0)
                }
            }

            //            if(m_DriverOp.isDown(GamepadKeys.Button.A)) {
//                s_Mod0.setTurnSpeed(0.5);
//                s_Mod1.setTurnSpeed(0.5);
//                s_Mod2.setTurnSpeed(0.5);
//                s_Mod3.setTurnSpeed(0.5);
//            } else {
//                s_Mod0.setTurnSpeed(0);
//            s_Mod1.setTurnSpeed(0);
//            s_Mod2.setTurnSpeed(0);
//            s_Mod3.setTurnSpeed(0);
//            }
            s_Mod0.update()
            s_Mod1.update()
            s_Mod2.update()
            s_Mod3.update()
            s_Sparky.update()
            telemetry.update()
        }
    }
}
