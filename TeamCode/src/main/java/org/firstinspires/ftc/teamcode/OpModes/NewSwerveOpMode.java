package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

//http://192.168.49.1:8080/dash
@TeleOp(name = "New Swerve")
public class NewSwerveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        Swerve s_Swerve = new Swerve(hardwareMap, telemetry);

        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telemetry);

        ElapsedTime runtime = new ElapsedTime();

        s_Sparky.configureOTOS();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Swerve.drive(
                    m_DriverOp.getLeftX(),
                    m_DriverOp.getLeftY(),
                    m_DriverOp.getRightX(),
                    true
            );

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Sparky.zeroGyro();
            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.addLine("OTOS");

            SparkFunOTOS.Pose2D pose = s_Sparky.getPose();

            packet.put("X Position: \t", pose.x);
            packet.put("Y Position: \t", pose.y);
            packet.put("Heading: \t", s_Sparky.getHeading());

            dashboard.sendTelemetryPacket(packet);

//            s_Swerve.update();
//            telemetry.update();

        }
    }
}
