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
import org.firstinspires.ftc.teamcode.Commands.DriveToPoint;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Normie Driving Core Sad", group = "Main")
public class NormalFunctionModeButLoser extends CommandOpMode {

    private final EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private AprilVisionOnTurret s_Vision;
    private OTOSSensor s_Sparky;

    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private Button zeroGyroButton;
    private Button autoDriveButton;

    @Override
    public void initialize() {

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
        autoDriveButton = new GamepadButton(m_Driver, GamepadKeys.Button.Y);

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Vision = new AprilVisionOnTurret(hardwareMap, telem, false);

        s_Swerve.setDefaultCommand(new TeleOpDrive(telem, s_Swerve, m_Driver, m_Operator));
        s_Intake.setDefaultCommand(new SmartIntake(s_Intake, s_Feeder, m_Driver));
        s_Turret.setDefaultCommand(new TurretToApril(s_Swerve, s_Turret, s_Vision, m_Operator));
        s_Shooter.setDefaultCommand(new CoolShooters(s_Shooter, s_Vision, m_Driver, m_Operator, telem));
        s_Sparky.setDefaultCommand(new RunCommand(() -> s_Sparky.periodic(), s_Sparky));
        s_Vision.setDefaultCommand(new RunCommand(() -> {
            s_Vision.periodic();
        }, s_Vision));

        zeroGyroButton.whenPressed(new InstantCommand(() -> {
            s_Swerve.zeroGyro();
        }, s_Swerve, s_Sparky));
        autoDriveButton.whenHeld(new DriveToPoint(s_Swerve, s_Sparky));

        schedule(new RunCommand(telem::updateAll));

    }

}
