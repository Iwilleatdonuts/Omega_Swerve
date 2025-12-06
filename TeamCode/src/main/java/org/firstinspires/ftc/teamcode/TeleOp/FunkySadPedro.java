package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.util.DashboardPoseTracker;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;

@TeleOp(name = "Pedro Blue", group = "Main")
public class FunkySadPedro extends LinearOpMode {

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

    private final Follower follower;

    private EZTelemetry telem;
    private OmegaController driver;
    private OmegaController operator;

    private Limelight s_Lime;

    private Swerve s_Swerve;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;

    private TurnToPointDrive driveCommand;
    private SmartIntake intakeCommand;
    private LimeTurret turretCommand;
    private CoolShooters shooterCommand;

    private PoseUpdater poseUpdater;
    private DashboardPoseTracker dashboardPoseTracker;

    private Telemetry telemetryA;

    @Override
    public void runOpMode(){

        follower =
        boolean areWeWinners = false;

        poseUpdater = new PoseUpdater(hardwareMap);
        dashboardPoseTracker = new DashboardPoseTracker(poseUpdater);

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);

        s_Lime.startLime();

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Lime.toggleTelemetry();

        driveCommand = new TurnToPointDrive(telem, s_Swerve, driver, operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Lime, driver, operator, telem);
//        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, s_Sparky, operator, driver, telem, areWeWinners);
//        shooterCommand = new CoolShooters(s_Shooter, s_Lime, s_Sparky, driver, operator, telem, areWeWinners);

        driveCommand.initialize();
        intakeCommand.initialize();
//        turretCommand.initialize();
//        shooterCommand.initialize();

        VisionThread visionRunnable = new VisionThread(s_Lime, 15);
        Thread visionThread = new Thread(visionRunnable, "Vision Thread");
        visionThread.start();

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will print your robot's position to telemetry while "
                + "allowing robot control through a basic mecanum drive on gamepad 1.");
        telemetryA.update();

        Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
        Drawing.sendPacket();

//        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());
//        telem.updateAll();

        waitForStart();

        if(isStopRequested()) {
            s_Lime.stopLime();
        }

        while (opModeIsActive()) {

            long loopStart = System.nanoTime();

            poseUpdater.update();
            dashboardPoseTracker.update();

            driveCommand.execute();
            intakeCommand.execute();
//            turretCommand.execute();
//            shooterCommand.execute();
            s_Lime.skadoodle();

            if(driver.wasJustPressed(GamepadKeys.Button.BACK)) {
                s_Swerve.zeroGyro();
            }

            telemetryA.addData("x", poseUpdater.getPose().getX());
            telemetryA.addData("y", poseUpdater.getPose().getY());
            telemetryA.addData("heading", poseUpdater.getPose().getHeading());
            telemetryA.addData("total heading", poseUpdater.getTotalHeading());
            telemetryA.update();

            Drawing.drawPoseHistory(dashboardPoseTracker, "#4CAF50");
            Drawing.drawRobot(poseUpdater.getPose(), "#4CAF50");
            Drawing.sendPacket();

//            telem.updateTelemetry();

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

    }

}
