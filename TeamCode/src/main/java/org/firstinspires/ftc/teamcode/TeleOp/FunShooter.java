package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ManualCommands.JoystickTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.math.MathUtil;

@TeleOp(name = "Fun", group = "Main")
public class FunShooter extends OpMode {

    private EZTelemetry telem;
    private OmegaController driver, operator;

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;

    private TeleOpDrive driveCommand;
    private JoystickTurret turretCommand;

    private double shooterAngle;
    private double shooterSpeed;

    @Override
    public void init() {

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        driveCommand = new TeleOpDrive(telem, s_Swerve, driver, operator);
        turretCommand = new JoystickTurret(s_Swerve, s_Turret, operator);

        driveCommand.initialize();
        turretCommand.initialize();

        shooterAngle = Constants.ShooterConstants.closeAngle;
        shooterSpeed = 0;

    }

    @Override
    public void loop() {

        long loopStart = System.nanoTime();

        driveCommand.execute();

        s_Intake.setSpeed(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        if(driver.isDown(GamepadKeys.Button.A) || operator.isDown(GamepadKeys.Button.A)) {
            s_Intake.setSpeed(1);
            s_Feeder.setFeederSpeed(1);
            s_Feeder.openGate();
        } else if (driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) != 0) {
            s_Feeder.openGate();
            s_Feeder.setFeederSpeed(-1);
        } else {
            s_Feeder.setFeederSpeed(0);
            s_Feeder.closeGate();
        }

        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            if(driver.isDown(GamepadKeys.Button.X)) {
                shooterSpeed -= 0.2;
            }

            if(driver.isDown(GamepadKeys.Button.Y)) {
                shooterAngle += 0.2;
            }
        }


        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            if(driver.isDown(GamepadKeys.Button.X)) {
                shooterSpeed += 0.2;
            }

            if(driver.isDown(GamepadKeys.Button.Y)) {
                shooterAngle -= 0.2;
            }
        }

        shooterSpeed = MathUtil.clamp(1, 0, shooterSpeed);
        shooterAngle = MathUtil.clamp(1, 0, shooterAngle);

        turretCommand.execute();

        s_Shooter.setShooterPower(shooterSpeed);
        s_Shooter.setShooterAngle(shooterAngle);

        if (driver.wasJustPressed(GamepadKeys.Button.BACK)) {
            s_Swerve.zeroGyro();
        }

        telem.updateTelemetry();

        long sleepTime = 20 - ((System.nanoTime() - loopStart) / 1_000_000);
        if (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }
}
