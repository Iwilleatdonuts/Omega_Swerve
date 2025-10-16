package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Thingy test Joystick")
public class Joystick extends LinearOpMode {


    @Override
    public void runOpMode() {
    GamepadEx m_DriverOp = new GamepadEx(gamepad1);

    ElapsedTime runtime = new ElapsedTime();

    waitForStart();
    runtime.reset();


    while (opModeIsActive()) {

        telemetry.addData("Joystick X", m_DriverOp.getLeftX());
        telemetry.addData("Joystick Y", m_DriverOp.getLeftY());
        double angle = Math.toDegrees(Math.atan2(-m_DriverOp.getLeftX(), m_DriverOp.getLeftY()));
        angle = (angle + 360) % 360;
        telemetry.addData("Joystick Angle", angle);

        telemetry.update();

    }
    }
}
