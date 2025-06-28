package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Test")
public class SwerveModuleTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        SwerveModule s_Module = new SwerveModule(hardwareMap, telemetry);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();


            s_Module.update();
            telemetry.update();

        }
    }
}
