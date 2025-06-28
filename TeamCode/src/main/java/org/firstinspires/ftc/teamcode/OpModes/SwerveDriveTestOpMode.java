package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

@TeleOp(name = "Swerve Test")
public class SwerveDriveTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        Swerve s_Swerve = new Swerve(hardwareMap);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Swerve.setPower(0, m_DriverOp.getLeftY());
            s_Swerve.setPower(1, m_DriverOp.getLeftY());
            s_Swerve.setPower(2, m_DriverOp.getLeftY());
            s_Swerve.setPower(3, m_DriverOp.getLeftY());

            if(m_DriverOp.isDown(GamepadKeys.Button.A)){
                s_Swerve.setModuleSpeed(0, m_DriverOp.getRightX());
            } else {
                s_Swerve.setModuleSpeed(0, 0);
            }

            if(m_DriverOp.isDown(GamepadKeys.Button.B)){
                s_Swerve.setModuleSpeed(1, m_DriverOp.getRightX());
            } else {
                s_Swerve.setModuleSpeed(1, 0);
            }

            if(m_DriverOp.isDown(GamepadKeys.Button.X)){
                s_Swerve.setModuleSpeed(2, m_DriverOp.getRightX());
            } else {
                s_Swerve.setModuleSpeed(2, 0);
            }

            if(m_DriverOp.isDown(GamepadKeys.Button.Y)){
                s_Swerve.setModuleSpeed(3, m_DriverOp.getRightX());
            } else {
                s_Swerve.setModuleSpeed(3, 0);
            }

            telemetry.addData("Module 0 Rotation", s_Swerve.getModuleRotation(0));
            telemetry.addData("Module 1 Rotation", s_Swerve.getModuleRotation(1));
            telemetry.addData("Module 2 Rotation", s_Swerve.getModuleRotation(2));
            telemetry.addData("Module 3 Rotation", s_Swerve.getModuleRotation(3));
            telemetry.update();

        }
    }
}
