package org.firstinspires.ftc.teamcode.Autos;

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

//    private class VisionThread implements Runnable {
//        private volatile boolean runVisionThread = true;
//        private final AprilVisionOnTurret s_Vision;
//        private final long sleepTime;
//
//        public VisionThread(AprilVisionOnTurret s_Vision, long sleepTime) {
//            this.s_Vision = s_Vision;
//            this.sleepTime = sleepTime;
//        }
//
//        public void stop() {
//            runVisionThread = false;
//        }
//
//        @Override
//        public void run(){
//
//            if(s_Vision == null){
//                return;
//            }
//
//            try {
//                while(runVisionThread && !Thread.currentThread().isInterrupted()) {
//                    s_Vision.skadoodle();
//                    Thread.sleep(sleepTime);
//                }
//            } catch (InterruptedException e) {
//                telem.putTelemetry("vision go bye bye", " hahahah");
//                telem.updateTelemetry();
//            }
//
//        }
//    }

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
        s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        s_Shooter = new Shooter(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);

        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.closeStart));
        s_Sparky.toggleTelemetry();

        driveCommand = new DriveToAutoPoint(s_Swerve, s_Sparky, telem);
        turretCommand = new AutoTurret(s_Turret, s_Vision);

        phase = 0;

//        VisionThread visionRunnable = new VisionThread(s_Vision, 15);
//        Thread visionThread = new Thread(visionRunnable, "Vision Thread");
//        visionThread.start();

        telem.putTelemetry("FPS", s_Vision.getCameraFPS());
        telem.updateTelemetry();

        waitForStart();

        while(opModeIsActive()){

//            long loopStart = System.nanoTime();

            telem.putTelemetry("Phase", phase);
            s_Sparky.skadoodle();
            s_Vision.skadoodle();
            telem.updateTelemetry();

            turretCommand.execute();
            s_Shooter.setShooterSpeed(0.36);

            switch(phase) {
                case 0:
                    driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                    phase++;
                    break;
                case 1:
                    driveCommand.execute();
                    s_Shooter.setShooterAngle(Constants.ShooterConstants.closeAngle);
                    if(driveCommand.isAtRoughSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 2:
                    if(s_Shooter.shooterAtSpeed()){
                        s_Feeder.openGate();
                        s_Feeder.setFeederSpeed(1);
                        s_Intake.setSpeed(1);
                    } else {
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                    }
                    if(System.nanoTime() - timestamp > 2.5e9){
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeBallLineup);
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 3:
                    s_Swerve.drive(-0.8, -0.8, 0, true, false);
                    if(System.nanoTime() - timestamp > 0.4e9){
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 4:
                    driveCommand.execute();
                    if(driveCommand.isAtSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 5:
                    s_Intake.setSpeed(1);
                    s_Swerve.drive(0.2, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 3e9){
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 6:
                    s_Swerve.drive(-0.8, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 0.3e9){
                        timestamp = System.nanoTime();
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.gateLineup);
                        phase++;
                    }
                    break;
                case 7:
                    driveCommand.execute();
                    if(driveCommand.isAtRoughSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 8:
                    s_Swerve.drive(0.2, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 2e9){
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                        phase++;
                    }
                    break;
                case 9:
                    driveCommand.execute();
                    s_Shooter.setShooterAngle(Constants.ShooterConstants.closeAngle);
                    if(driveCommand.isAtRoughSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 10:
                    if(s_Shooter.shooterAtSpeed()){
                        s_Feeder.openGate();
                        s_Feeder.setFeederSpeed(1);
                        s_Intake.setSpeed(1);
                    } else {
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                    }
                    if(System.nanoTime() - timestamp > 4e9){
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.mediumBallLineup);
                        phase++;
                    }
                    break;
                case 11:
                    s_Swerve.drive(-0.8, -0.8, 0, true, false);
                    if(System.nanoTime() - timestamp > 0.8e9){
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 12:
                    driveCommand.execute();
                    if(driveCommand.isAtSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 13:
                    s_Intake.setSpeed(1);
                    s_Swerve.drive(0.2, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 3e9){
                        timestamp = System.nanoTime();
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                        phase++;
                    }
                    break;
                case 14:
                    s_Intake.setSpeed(1);
                    s_Swerve.drive(-0.6, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 1e9){
                        timestamp = System.nanoTime();
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                        phase++;
                    }
                    break;
                case 15:
                    driveCommand.execute();
                    s_Shooter.setShooterAngle(Constants.ShooterConstants.closeAngle);
                    if(driveCommand.isAtRoughSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 16:
                    if(s_Shooter.shooterAtSpeed()){
                        s_Feeder.openGate();
                        s_Feeder.setFeederSpeed(1);
                        s_Intake.setSpeed(1);
                    } else {
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                    }
                    if(System.nanoTime() - timestamp > 4e9){
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.farBallLineup);
                        phase++;
                    }
                    break;
                case 17:
                    s_Swerve.drive(-0.8, -0.8, 0, true, false);
                    if(System.nanoTime() - timestamp > 0.8e9){
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 18:
                    driveCommand.execute();
                    if(driveCommand.isAtSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 19:
                    s_Intake.setSpeed(1);
                    s_Swerve.drive(0.2, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 3e9){
                        timestamp = System.nanoTime();
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                        phase++;
                    }
                    break;
                case 20:
                    s_Intake.setSpeed(1);
                    s_Swerve.drive(-0.6, 0, 0, true, false);
                    if(System.nanoTime() - timestamp > 1e9){
                        timestamp = System.nanoTime();
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.closeShot);
                        phase++;
                    }
                    break;
                case 21:
                    driveCommand.execute();
                    s_Shooter.setShooterAngle(Constants.ShooterConstants.closeAngle);
                    if(driveCommand.isAtRoughSetpoint()){
                        s_Swerve.stop();
                        timestamp = System.nanoTime();
                        phase++;
                    }
                    break;
                case 22:
                    if(s_Shooter.shooterAtSpeed()){
                        s_Feeder.openGate();
                        s_Feeder.setFeederSpeed(1);
                        s_Intake.setSpeed(1);
                    } else {
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                    }
                    if(System.nanoTime() - timestamp > 4e9){
                        s_Feeder.closeGate();
                        s_Feeder.setFeederSpeed(0);
                        s_Intake.setSpeed(0);
                        driveCommand.initialize(Constants.AutoConstants.RedConstants.gateLineupTeleop);
                        phase++;
                    }
                    break;
                case 23:
                    driveCommand.execute();
                    s_Intake.setSpeed(0);
                    s_Feeder.closeGate();
                    s_Feeder.setFeederSpeed(0);
                    s_Shooter.setShooterSpeed(0);
                    break;
            }


//            long mainThreadSleep = 20 - ((System.nanoTime() - loopStart) / 1000000);
//
//            if(mainThreadSleep > 0) {
//                try {
//                    Thread.sleep(mainThreadSleep);
//                } catch (InterruptedException ie) {
//                    Thread.currentThread().interrupt();
//                    break;
//                }
//            }

        }

//        visionRunnable.stop();

    }
}
