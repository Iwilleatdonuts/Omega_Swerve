package org.firstinspires.ftc.teamcode.OpModes.TestingModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToSwervePoint;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
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
@TeleOp(name = "Shooter Manual - Blue", group = "Testing")
public class BlueShooterTestMode extends CommandOpMode {

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
    private boolean shootersGunnaShoot = false;

    private double shooterSpeed = 0;
    private double output = 0;

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
        s_Vision = new AprilVisionOnTurret(hardwareMap, telemetry, false);

        s_Swerve.setDefaultCommand(new TurnToPointDrive(telemetry, s_Swerve, s_Sparky, m_Driver, m_Operator));
        s_Intake.setDefaultCommand(new SmartIntake(s_Intake, s_Feeder, m_Driver, dashboard));
        s_Turret.setDefaultCommand(new TurretToApril(s_Swerve, s_Turret, s_Vision, dashboard, m_Operator));
        s_Shooter.setDefaultCommand(new RunCommand(() -> {

            m_Operator.readButtons();

            if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)){
                shootersGunnaShoot = !shootersGunnaShoot;
            }

            if(!shootersGunnaShoot) {
                output = 0;
            } else {
                output = shooterSpeed;
            }

            if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                shooterSpeed += 0.01;
            }
            if(m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                shooterSpeed -= 0.01;
            }

            s_Shooter.setShooterSpeed(output);

            telemetry.addData("Distance", s_Vision.getGoalDistance());
            telemetry.addData("Shooter Target Percentage", shooterSpeed);

            telemetry.update();
        }, s_Shooter));
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
