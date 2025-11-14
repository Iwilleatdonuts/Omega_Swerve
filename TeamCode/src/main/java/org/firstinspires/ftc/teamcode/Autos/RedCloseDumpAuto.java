package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoTurret;
import org.firstinspires.ftc.teamcode.AutoCommands.DriveToAutoPoint;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

@Autonomous(name = "Red Close Dump")
public class RedCloseDumpAuto extends LinearOpMode {

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

            while(runVisionThread && !Thread.currentThread().isInterrupted()) {

                try {
                    s_Vision.skadoodle();
                } catch (Exception e) {
                    telem.putTelemetry("Vision Thread say bye bye", " LMAO");
                    telem.updateTelemetry();
                }

                try {
                    Thread.sleep(sleepTime);
                } catch (InterruptedException ie) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

        }
    }

    private EZTelemetry telem;

    private AprilVisionOnTurret s_Vision;
    private OTOSSensor s_Sparky;
    private Swerve s_Swerve;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private Intake s_Intake;
    private Feeder s_Feeder;

    private DriveToAutoPoint driveCommand;
    private AutoTurret turretCommand;

    private int phase;

    private double timestamp;

    @Override
    public void runOpMode() {

        telem = new EZTelemetry(telemetry);

        s_Vision = new AprilVisionOnTurret(hardwareMap, telem, true);
        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Swerve = new Swerve(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);

        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.closeStart));
        s_Sparky.toggleTelemetry();

        driveCommand = new DriveToAutoPoint(s_Swerve, s_Sparky, telem);
        turretCommand = new AutoTurret(s_Turret, s_Vision);

        phase = 0;

        VisionThread visionRunnable = new VisionThread(s_Vision, 15);
        Thread visionThread = new Thread(visionRunnable, "Vision Thread");
        visionThread.start();

        telem.putTelemetry("FPS", s_Vision.getCameraFPS());
        telem.updateTelemetry();

        waitForStart();

        while(opModeIsActive()){

            long loopStart = System.nanoTime();

            telem.putTelemetry("Phase", phase);
            s_Sparky.skadoodle();
            telem.updateTelemetry();
            switch(phase) {
                case 0:
                    driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                    phase++;
                    break;
                case 1:
                    driveCommand.execute();
                    break;
//                case 0:
//                    driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
//                    turretCommand.execute();
//                    s_Shooter.setShooterSpeed(0.36);
//                    phase++;
//                    break;
//                case 1:
//
//                    s_Shooter.setShooterSpeed(0.36);
//                    driveCommand.execute();
//                    turretCommand.execute();
//
//                    if(driveCommand.isAtRoughSetpoint() && s_Shooter.shooterAtSpeed()){
//                        s_Swerve.drive(0, 0, 0, true, true);
//                        timestamp = System.nanoTime();
//                        phase++;
//                    }
//                    break;
//                case 2:
//
//                    s_Intake.setSpeed(1);
//                    s_Feeder.setFeederSpeed(1);
//                    s_Feeder.openGate();
//
//                    if((System.nanoTime() - timestamp)/1000000 > 1.5){
//                        s_Feeder.setFeederSpeed(0);
//                        s_Feeder.closeGate();
//                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeBallLineup);
//                        phase++;
//                    }
//                    break;
//                case 3:
//                    driveCommand.execute();
//
//                    if(driveCommand.isAtSetpoint()){
//                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeBallPickup);
//                        phase++;
//                    }
//                    break;
//                case 4:
//                    driveCommand.execute();
//
//                    if(driveCommand.isAtRoughSetpoint()){
//                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
//                        phase++;
//                    }
//                    break;
//                case 5:
//                    driveCommand.execute();
//
//                    if(driveCommand.isAtRoughSetpoint() && s_Shooter.shooterAtSpeed()){
//                        s_Swerve.drive(0, 0, 0, true, true);
//                        timestamp = System.nanoTime();
//                        phase++;
//                    }
//                    break;
//                case 6:
//                    s_Intake.setSpeed(1);
//                    s_Feeder.setFeederSpeed(1);
//                    s_Feeder.openGate();
//
//                    if((System.nanoTime() - timestamp)/1000000 > 1.5){
//                        s_Feeder.setFeederSpeed(0);
//                        s_Feeder.closeGate();
//                        driveCommand.initialize(Constants.AutoConstants.RedConstants.mediumBallLineup);
//                        phase++;
//                    }
//                    break;
            }


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

        s_Vision.stopCamera();

        visionRunnable.stop();
        visionThread.interrupt();
        try {
            visionThread.join(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }
}
