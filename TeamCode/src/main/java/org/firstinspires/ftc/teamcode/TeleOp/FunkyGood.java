package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Utilities.PedroPathing.Tuning.follower;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
import org.firstinspires.ftc.teamcode.Commands.LimeTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.PedroPathing.PedroConstants;

@TeleOp(name = "Ginger Drive Core", group = "Main")
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

    private Follower follower;

    @Override
    public void runOpMode(){
        boolean areWeWinners = true;

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        s_Lime.toggleTelemetry();

        s_Lime.startLime();

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        follower = PedroConstants.createFollower(hardwareMap, s_Swerve);
        follower.setStartingPose(new Pose(72,72));
//        s_Sparky.toggleTelemetry();
//        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.mediumShotPositionForTeleop));

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

//        telem.enableCameraStrea(s_Lime.);

        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());
        telem.updateAll();

        waitForStart();

        if(isStopRequested()) {
            s_Lime.stopLime();
        }

        while (opModeIsActive()) {

//            s_Lime.updateRobotYawFromGyro(s_Sparky.getHeading());
//
//            telem.putTelemetry("Tag X", s_Lime.getLimePose().getPosition().x);
//            telem.putTelemetry("Tag Y", s_Lime.getLimePose().getPosition().y);
//            telem.putTelemetry("Tag R", s_Lime.getLimePose().getOrientation().getYaw(AngleUnit.DEGREES));

            long loopStart = System.nanoTime();

            driveCommand.execute();
            intakeCommand.execute();
//            turretCommand.execute();
//            shooterCommand.execute();
            s_Lime.skadoodle();
            s_Swerve.skadoodle();

            follower.update();
            Pose currentPose = follower.getPose();
            telem.putTelemetry("1 X Pose", currentPose.getX());
            telem.putTelemetry("1 Y Pose", currentPose.getY());
            telem.putTelemetry("1 Heading", Math.toDegrees(currentPose.getHeading()));

            if(driver.wasJustPressed(GamepadKeys.Button.BACK)) {
                s_Swerve.zeroGyro();
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
        s_Lime.stopLime();
        visionRunnable.stop();

    }

}
