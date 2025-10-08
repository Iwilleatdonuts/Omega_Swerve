package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ManualCommands.JoystickTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Ginger Driving Core")
public class RileysFunkyDriveMode extends CommandOpMode {

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
//    private AprilVision s_Vision;
    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private FtcDashboard dashboard;

    private Button zeroGyroButton;

    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);

        s_Swerve = new Swerve(hardwareMap, telemetry);
        s_Intake = new Intake(hardwareMap, telemetry);
        s_Feeder = new Feeder(hardwareMap, telemetry);
        s_Turret = new Turret(hardwareMap, telemetry);
        s_Shooter = new Shooter(hardwareMap, telemetry);
//        s_Vision = new AprilVision(hardwareMap, telemetry);
//        dashboard.startCameraStream(s_Vision.getAprilCamera(), 30);

        s_Swerve.setDefaultCommand(new TurnToPointDrive(telemetry, s_Swerve, m_Driver));
        s_Intake.setDefaultCommand(new SmartIntake(s_Intake, s_Feeder, m_Driver, dashboard));
        s_Turret.setDefaultCommand(new JoystickTurret(s_Swerve, s_Turret, s_Shooter, m_Operator, dashboard));
//        s_Vision.setDefaultCommand(new RunCommand(() -> s_Vision.periodic(), s_Vision));

        zeroGyroButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));
    }

}
