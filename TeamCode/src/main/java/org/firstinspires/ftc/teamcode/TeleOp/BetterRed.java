package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
import org.firstinspires.ftc.teamcode.Commands.LimeTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

@TeleOp(name = "Red TeleOp", group = "Main")
public class BetterRed extends OpMode {

    private class VisionThread implements Runnable {
        private volatile boolean runVisionThread = true;
        private final Limelight s_Lime;
        private final long sleepTime;

        public VisionThread(Limelight lime, long sleepTime) {
            this.s_Lime = lime;
            this.sleepTime = sleepTime;
        }

        public void stop() {
            runVisionThread = false;
        }

        @Override
        public void run() {
            if (s_Lime == null) return;

            try {
                while (runVisionThread && !Thread.currentThread().isInterrupted()) {
                    s_Lime.skadoodle();
                    Thread.sleep(sleepTime);
                }
            } catch (InterruptedException e) {
                telem.putTelemetry("vision go bye bye", "hahahah");
                telem.updateTelemetry();
            }
        }
    }

    private EZTelemetry telem;
    private OmegaController driver, operator;

    private Limelight s_Lime;
    private OTOSSensor s_Sparky;

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private LimeTurret turretCommand;
    private CoolShooters shooterCommand;

    private VisionThread visionRunnable;
    private Thread visionThread;

    private long lastLoopTime;

    private boolean areWeWinners = true;

    @Override
    public void init() {

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        s_Lime.toggleTelemetry();
        s_Lime.startLime();

        s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Sparky.toggleTelemetry();
        s_Sparky.configureOTOS(new SparkFunOTOS.Pose2D(0, 0, 0));

        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, driver, operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Lime, driver, operator, telem);
        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, s_Sparky, operator, driver, telem, areWeWinners);
        shooterCommand = new CoolShooters(s_Shooter, s_Lime, s_Sparky, driver, operator, telem, areWeWinners);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();
        shooterCommand.initialize();

        visionRunnable = new VisionThread(s_Lime, 15);
        visionThread = new Thread(visionRunnable, "Vision Thread");
        visionThread.start();
    }

    @Override
    public void init_loop() {

        if(driver.wasJustPressed(GamepadKeys.Button.B)) {
            s_Sparky.configureOTOS(
                    s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.mediumShotPositionForTeleop)
            );
        }

        if(driver.wasJustPressed(GamepadKeys.Button.Y)) {
            s_Sparky.configureOTOS(
                    s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.gateLineupTeleop)
            );
        }

        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());

        telem.putTelemetry("X Position", s_Sparky.getPose().x());
        telem.putTelemetry("Y Position", s_Sparky.getPose().y());
        telem.putTelemetry("Heading", s_Sparky.getHeading());

        telem.updateTelemetry();

    }

    @Override
    public void start() {
        lastLoopTime = System.nanoTime();
    }

    @Override
    public void loop() {

        long loopStart = System.nanoTime();

        driveCommand.execute();
        intakeCommand.execute();
        turretCommand.execute();
        shooterCommand.execute();
        s_Lime.skadoodle();

        if (driver.wasJustPressed(GamepadKeys.Button.BACK)) {
            s_Swerve.zeroGyro();
            s_Sparky.zeroGyro();
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

    @Override
    public void stop() {
        s_Sparky.disable();
        s_Lime.stopLime();

        if (visionRunnable != null) visionRunnable.stop();
        if (visionThread != null) visionThread.interrupt();
    }
}
