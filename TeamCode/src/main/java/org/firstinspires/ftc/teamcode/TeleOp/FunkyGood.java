package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
import org.firstinspires.ftc.teamcode.Commands.LimeTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

@TeleOp(name = "Red Ginger", group = "Main")
public class FunkyGood extends LinearOpMode {

    private class VisionThread implements Runnable {
        private volatile boolean runVisionThread = true;
        private final Limelight s_Lime;
        private final long sleepTime;

        public VisionThread(Limelight s_Lime, long sleepTime) {
            this.s_Lime = s_Lime;
            this.sleepTime = sleepTime;
        }

        public void stop() {
            runVisionThread = false;
        }

        @Override
        public void run(){

            if(s_Lime == null){
                return;
            }

            try {
                while(runVisionThread && !Thread.currentThread().isInterrupted()) {
                    s_Lime.skadoodle();
                    Thread.sleep(sleepTime);
                }
            } catch (InterruptedException e) {
                telem.putTelemetry("vision go bye bye", " hahahah");
                telem.updateTelemetry();
            }

        }
    }

    private class LemonThread implements Runnable {
        private volatile boolean runLemonThread = true;
        private final FusionOdometry s_Lemon;
        private final long sleepTime;

        public LemonThread(FusionOdometry s_Lemon, long sleepTime) {
            this.s_Lemon = s_Lemon;
            this.sleepTime = sleepTime;
        }

        public void stop() {
            runLemonThread = false;
        }

        @Override
        public void run(){

            if(s_Lemon == null){
                return;
            }

            try {
                while(runLemonThread && !Thread.currentThread().isInterrupted()) {
                    s_Lemon.skadoodle();
                    Thread.sleep(sleepTime);
                }
            } catch (InterruptedException e) {
                telem.putTelemetry("lemons are sour now", " :(((");
                telem.updateTelemetry();
            }

        }
    }

    private EZTelemetry telem;
    private OmegaController driver;
    private OmegaController operator;
    private Limelight s_Lime;

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private FusionOdometry s_Lemon;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private LimeTurret turretCommand;
    private CoolShooters shooterCommand;

    @Override
    public void runOpMode(){
        boolean areWeWinners = true;

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        s_Lime.startLime();

        s_Lemon = new FusionOdometry(hardwareMap, telem);
//        s_Lemon.toggleTelemetry();

        s_Swerve = new Swerve(hardwareMap, telem, s_Lemon);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        driveCommand = new TurnToPointDrive(telem, s_Swerve, driver, operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Lime, driver, operator, telem);
        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, s_Lemon, operator, driver, telem, areWeWinners);
        shooterCommand = new CoolShooters(s_Shooter, s_Lime, s_Lemon, driver, operator, telem, areWeWinners);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();
        shooterCommand.initialize();

        VisionThread visionRunnable = new VisionThread(s_Lime, 15);
        Thread visionThread = new Thread(visionRunnable, "Vision Thread");
        visionThread.start();

        LemonThread lemonRunnable = new LemonThread(s_Lemon, 15);
        Thread lemonThread = new Thread(lemonRunnable, "Lemon Thread");
        lemonThread.start();

        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());
        telem.putLine();
//        telem.putTelemetry("X Pose", s_Lemon.getCurrentPose().x());
//        telem.putTelemetry("Y Pose", s_Lemon.getCurrentPose().y());
//        telem.putTelemetry("Heading Pose", s_Lemon.getCurrentPose().r());
        telem.updateAll();

        waitForStart();

        if(isStopRequested()) {
            s_Lime.stopLime();
        }

        while (opModeIsActive()) {

            long loopStart = System.nanoTime();

            driveCommand.execute();
            intakeCommand.execute();
            turretCommand.execute();
            shooterCommand.execute();
            s_Swerve.skadoodle();

            if(driver.wasJustPressed(GamepadKeys.Button.BACK)) {
                s_Swerve.zeroGyro();
            }

            telem.putLine("Odometry");
            OmegaPose2D currentPose = s_Lemon.getCurrentPose();
            telem.putTelemetry("X Position ", currentPose.x());
            telem.putTelemetry("Y Position ", currentPose.y());
            telem.putTelemetry("Heading ", currentPose.r());
            telem.putLine();

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
        s_Lime.stopLime();
        visionRunnable.stop();
        lemonRunnable.stop();
    }

}
