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

import org.firstinspires.ftc.teamcode.Commands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Ginger Driving Core")
public class RileysFunkyDriveMode extends CommandOpMode {

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Turret s_Turret;
    private AprilVision s_Vision;
    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private FtcDashboard dashboard;

    private Button zeroGyroButton;
    private Button shooterLockButton;

    @Override
    public void initialize() {

        dashboard = FtcDashboard.getInstance();

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
        shooterLockButton = new GamepadButton(m_Driver, GamepadKeys.Button.A);

        s_Swerve = new Swerve(hardwareMap, telemetry);
        s_Intake = new Intake(hardwareMap, telemetry);
        s_Turret = new Turret(hardwareMap, telemetry);
        s_Vision = new AprilVision(hardwareMap, telemetry);

        s_Swerve.setDefaultCommand(new TurnToPointDrive(telemetry, s_Swerve, m_Driver, m_Operator));
        s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.setSpeed(m_Driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_Driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)), s_Intake));
//        s_Vision.setDefaultCommand(new RunCommand(() -> s_Vision.periodic(), s_Vision));

        zeroGyroButton.whenPressed(new InstantCommand(() -> s_Swerve.zeroGyro(), s_Swerve));

        shooterLockButton.whenHeld(new TurretToApril(s_Turret, s_Vision));

    }

}
