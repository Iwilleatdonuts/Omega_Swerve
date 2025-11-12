package org.firstinspires.ftc.teamcode.OpModes.TestingModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToPoint;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Constants;
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
@TeleOp(name = "Shooter Manual - Blue", group = "Testing")
public class BlueShooterTestMode extends LinearOpMode {

    private EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private AprilVisionOnTurret s_Vision;
    private AprilVisionOnTurret.VisionReadings visionReadings;
    private OTOSSensor s_Sparky;

    private GamepadEx m_Driver;
    private GamepadEx m_Operator;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private TurretToApril turretCommand;

    private Button zeroGyroButton;
    private Button autoDriveButton;
    private boolean shootersGunnaShoot = false;

    private double shooterSpeed = 0;
    private double shooterAngle = 1;
    private double output = 0;

    @Override
    public void runOpMode() {

        m_Driver = new GamepadEx(gamepad1);
        m_Operator = new GamepadEx(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
        autoDriveButton = new GamepadButton(m_Driver, GamepadKeys.Button.Y);

        visionReadings = new AprilVisionOnTurret.VisionReadings();

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Vision = new AprilVisionOnTurret(hardwareMap, telem, false, visionReadings);

        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, m_Driver, m_Operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, m_Driver, telem);
        turretCommand = new TurretToApril(s_Swerve, s_Turret, s_Vision, m_Operator);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();

        waitForStart();

        while (opModeIsActive()) {

            s_Sparky.skadoodle();
            s_Vision.skadoodle();

            driveCommand.execute();
            intakeCommand.execute();
            turretCommand.execute();

                if (m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    shootersGunnaShoot = !shootersGunnaShoot;
                }

                if (!shootersGunnaShoot) {
                    output = 0;
                } else {
                    output = shooterSpeed;
                }

                if (m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                    shooterSpeed += 0.01;
                }
                if (m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                    shooterSpeed -= 0.01;
                }

                if (m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                    shooterAngle = Constants.ShooterConstants.closeAngle;
                }

                if (m_Operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                    shooterAngle = Constants.ShooterConstants.farAngle;
                }

                if (s_Vision.hasGoalTag()) {
                    shooterAngle = s_Shooter.getShooterAngleFromDistance(s_Vision.getGoalDistance());
                }

                s_Shooter.setShooterAngle(shooterAngle);
                s_Shooter.setShooterSpeed(output);


                if(m_Driver.wasJustPressed(GamepadKeys.Button.START)) {
                    s_Sparky.zeroGyro();
                    s_Swerve.zeroGyro();
                }

            telem.putTelemetry("Distance", s_Vision.getGoalDistance());
            telem.putTelemetry("Shooter Target Percentage", shooterSpeed);

            telem.updateTelemetry();

        }
    }
}
