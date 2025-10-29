package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Turret Test with Low CLT")
public class TurretMode extends CommandOpMode {

    private Swerve s_Swerve;
    private Turret s_Turret;
    private AprilVisionOnTurret s_Vision;

    private FtcDashboard dashboard;

    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private Button zeroGyroButton;

    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);

        s_Swerve = new Swerve(hardwareMap, telemetry);
        s_Turret = new Turret(hardwareMap, telemetry);
        s_Vision = new AprilVisionOnTurret(hardwareMap, telemetry, true);
        dashboard.startCameraStream(s_Vision.getAprilCamera(), 30);

        s_Vision.toggleTelemetry();

        s_Turret.setDefaultCommand(new TurretToApril(s_Swerve, s_Turret, s_Vision, dashboard, m_Operator));
//        s_Shooter.setDefaultCommand(new RunCommand(() -> {
//
//            m_Operator.readButtons();
//
//            if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
//                shootersGunnaShoot = !shootersGunnaShoot;
//            }
//
//            if(!shootersGunnaShoot) {
//                output = 0;
//            } else {
//                output = shooterSpeed;
//            }
//
//            if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                shooterSpeed += 0.01;
//            }
//            if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                shooterSpeed -= 0.01;
//            }
//
//            s_Shooter.setShooterSpeed(output);
//
//            telemetry.addData("Distance", s_Vision.getGoalDistance());
//            telemetry.addData("Shooter Target Percentage", shooterSpeed);
//
//            telemetry.update();
//        }, s_Shooter));
        s_Vision.setDefaultCommand(new RunCommand(() -> {
            s_Vision.periodic();
            telemetry.addData("Distance to Target", s_Vision.getGoalDistance());
            }, s_Vision));


    }

}
