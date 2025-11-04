package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
import org.firstinspires.ftc.teamcode.Commands.DriveToSwervePoint;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Normie Driving Core Sad", group = "Main")
public class NormalFunctionModeButLoser extends CommandOpMode {

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private AprilVisionOnTurret s_Vision;
    private OTOSSensor s_Sparky;

    private FtcDashboard dashboard;

    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private Button zeroGyroButton;
    private Button autoDriveButton;

    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
        autoDriveButton = new GamepadButton(m_Driver, GamepadKeys.Button.Y);

        s_Swerve = new Swerve(hardwareMap, telemetry);
        s_Intake = new Intake(hardwareMap, telemetry);
        s_Feeder = new Feeder(hardwareMap, telemetry);
        s_Turret = new Turret(hardwareMap, telemetry);
        s_Shooter = new Shooter(hardwareMap, telemetry);
        s_Sparky = new OTOSSensor(hardwareMap, telemetry);
        s_Vision = new AprilVisionOnTurret(hardwareMap, telemetry, true);

        s_Swerve.setDefaultCommand(new TeleOpDrive(telemetry, dashboard, s_Swerve, m_Driver, m_Operator));
        s_Intake.setDefaultCommand(new SmartIntake(s_Intake, s_Feeder, m_Driver, dashboard));
        s_Turret.setDefaultCommand(new TurretToApril(s_Swerve, s_Turret, s_Vision, dashboard, m_Operator));
        s_Shooter.setDefaultCommand(new CoolShooters(s_Shooter, s_Vision, m_Driver, m_Operator, telemetry));
        s_Sparky.setDefaultCommand(new RunCommand(() -> s_Sparky.periodic(), s_Sparky));
        s_Vision.setDefaultCommand(new RunCommand(() -> {
            s_Vision.periodic();
        }, s_Vision));

        zeroGyroButton.whenPressed(new InstantCommand(() -> {
            s_Swerve.zeroGyro();
        }, s_Swerve, s_Sparky));
        autoDriveButton.whenHeld(new DriveToSwervePoint(s_Swerve, s_Sparky));

    }

}
