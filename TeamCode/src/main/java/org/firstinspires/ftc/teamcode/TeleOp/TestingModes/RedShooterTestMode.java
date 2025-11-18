package org.firstinspires.ftc.teamcode.TeleOp.TestingModes;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.LimeTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Commands.TurretToApril;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.Controller.GamepadButton;
import org.firstinspires.ftc.teamcode.Utilities.Controller.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Shooter Manual - Red", group = "Testing")
public class RedShooterTestMode extends LinearOpMode {

    private EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private Limelight s_Lime;
    private OTOSSensor s_Sparky;

    private OmegaController m_Driver;
    private OmegaController m_Operator;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private LimeTurret turretCommand;

    private Button zeroGyroButton;
    private Button autoDriveButton;
    private boolean shootersGunnaShoot = false;

    private double shooterSpeed = 0;
    private double shooterAngle = 1;
    private double output = 0;

    @Override
    public void runOpMode() {

        m_Driver = new OmegaController(gamepad1);
        m_Operator = new OmegaController(gamepad2);

        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
        autoDriveButton = new GamepadButton(m_Driver, GamepadKeys.Button.Y);

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Lime = new Limelight(hardwareMap, telem, true);

        s_Sparky.configureOTOS(new SparkFunOTOS.Pose2D(0, 0, 0));

        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, m_Driver, m_Operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Lime, m_Driver, m_Operator, telem);
        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, m_Operator);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();

        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());
        telem.updateTelemetry();

        waitForStart();

        if(isStopRequested()) {
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            s_Sparky.skadoodle();
            s_Lime.skadoodle();

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

            if (s_Lime.isValidReaing()) {
                shooterAngle = s_Shooter.getShooterAngleFromDistance(s_Lime.getGoalDistance());
            }

            s_Shooter.setShooterAngle(shooterAngle);
            s_Shooter.setShooterSpeed(output);


            if(m_Driver.wasJustPressed(GamepadKeys.Button.START)) {
                s_Sparky.zeroGyro();
                s_Swerve.zeroGyro();
            }

            telem.putTelemetry("Distance", s_Lime.getGoalDistance());
            telem.putTelemetry("Shooter Target Percentage", shooterSpeed);

            telem.updateTelemetry();

        }
        s_Sparky.disable();
    }
}
