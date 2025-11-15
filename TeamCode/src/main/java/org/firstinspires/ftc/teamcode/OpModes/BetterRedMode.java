package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
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

@TeleOp(name = "Ginger Drive Core", group = "Main")
public class BetterRedMode extends LinearOpMode {

    private class VisionThread implements Runnable {
        private volatile boolean runVisionThread = true;
        private final AprilVisionOnTurret s_Vision;
        private final long sleepTime;

        public VisionThread(AprilVisionOnTurret s_Vision, long sleepTime) {
            this.s_Vision = s_Vision;
            this.sleepTime = sleepTime;
        }

        public void stop() {
            runVisionThread = false;
        }

        @Override
        public void run(){

            if(s_Vision == null){
                return;
            }

            try {
                while(runVisionThread && !Thread.currentThread().isInterrupted()) {
                    s_Vision.skadoodle();
                    Thread.sleep(sleepTime);
                }
            } catch (InterruptedException e) {
                telem.putTelemetry("vision go bye bye", " hahahah");
                telem.updateTelemetry();
            }

        }
    }

    private EZTelemetry telem;
    private GamepadEx driver;
    private GamepadEx operator;

    private AprilVisionOnTurret s_Vision;
    private OTOSSensor s_Sparky;

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private TurretToApril turretCommand;
    private CoolShooters shooterCommand;

    @Override
    public void runOpMode(){

        driver = new GamepadEx(gamepad1);
        operator = new GamepadEx(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Vision = new AprilVisionOnTurret(hardwareMap, telem, true);

        s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Sparky.toggleTelemetry();
        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.gateLineupTeleop));

        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, driver, operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Vision, driver, operator, telem);
        turretCommand = new TurretToApril(s_Swerve, s_Turret, s_Vision, operator);
        shooterCommand = new CoolShooters(s_Shooter, s_Vision, driver, operator, telem);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();
        shooterCommand.initialize();

        VisionThread visionRunnable = new VisionThread(s_Vision, 15);
        Thread visionThread = new Thread(visionRunnable, "Vision Thread");
        visionThread.start();

        telem.putTelemetry("FPS", s_Vision.getCameraFPS());
        telem.updateTelemetry();

        waitForStart();

        if(isStopRequested()) {
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            long loopStart = System.nanoTime();

            s_Sparky.skadoodle();

            driveCommand.execute();
            intakeCommand.execute();
            turretCommand.execute();
            shooterCommand.execute();

            if(driver.wasJustPressed(GamepadKeys.Button.START)) {
                s_Swerve.zeroGyro();
                s_Sparky.zeroGyro();
            }

            telem.updateTelemetry();

            long mainThreadSleep = 20 - ((System.nanoTime() - loopStart) / 1000000);

            if(mainThreadSleep > 0) {
                try {
                    Thread.sleep(mainThreadSleep);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }
        }
        s_Sparky.disable();
        visionRunnable.stop();

    }

}
