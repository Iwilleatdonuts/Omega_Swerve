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

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
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

        boolean slowMode = false;
        boolean dashboardDriving = false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            double leftXVal = m_DriverOp.getLeftX();
            double leftYVal = m_DriverOp.getLeftY();

            if(dashboardDriving) {
                leftXVal = m_DriverOp.getLeftY();
                leftYVal = m_DriverOp.getLeftX();
            }

            s_Swerve.drive(
                    leftXVal,
                    leftYVal,
                    m_DriverOp.getRightX(),
                    true,
                    slowMode
            );

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Sparky.zeroGyro();
            }

            if(m_DriverOp.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) || m_DriverOp.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON)){
                slowMode = true;
            } else {
                slowMode = false;
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
                dashboardDriving = !dashboardDriving;
            }

            TelemetryPacket packet = new TelemetryPacket();

            SparkFunOTOS.Pose2D pose = s_Sparky.getPose();

            packet.put("X Position: \t", pose.x);
            packet.put("Y Position: \t", pose.y);
            packet.put("Heading: \t", s_Sparky.getHeading());
            packet.put("SlowMode Enabled: \t", slowMode);
            packet.putAll(s_Swerve.getMotorCurrents());

            dashboard.sendTelemetryPacket(packet);

        }
    }
}
