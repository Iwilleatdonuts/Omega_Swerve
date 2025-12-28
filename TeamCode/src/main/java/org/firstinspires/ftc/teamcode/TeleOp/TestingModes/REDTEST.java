//package org.firstinspires.ftc.teamcode.TeleOp.TestingModes;
//
//import com.arcrobotics.ftclib.command.button.Button;
//import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
//import org.firstinspires.ftc.teamcode.Commands.LimeTurret;
//import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
//import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TurnToPointDrive;
//import org.firstinspires.ftc.teamcode.Constants;
//import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
//import org.firstinspires.ftc.teamcode.Subsystems.Intake;
//import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
//import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
//import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
//import org.firstinspires.ftc.teamcode.Subsystems.Turret;
//import org.firstinspires.ftc.teamcode.Utilities.OmegaController.GamepadButton;
//import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
//import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
//import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
//
////http://192.168.43.1:8080/dash
////adb connect 192.168.43.1:5555
//@TeleOp(name = "Shooter Manual - Red", group = "Testing")
//public class RedShooterTestMode extends LinearOpMode {
//
//    private EZTelemetry telem = new EZTelemetry(telemetry);
//
//    private Swerve s_Swerve;
//    private Intake s_Intake;
//    private Feeder s_Feeder;
//    private Turret s_Turret;
//    private Shooter s_Shooter;
//    private Limelight s_Lime;
//    private OTOSSensor s_Sparky;
//
//    private OmegaController m_Driver;
//    private OmegaController m_Operator;
//
//    private TurnToPointDrive driveCommand;
//    private SmartIntake intakeCommand;
//    private LimeTurret turretCommand;
//
//    private Button zeroGyroButton;
//    private Button autoDriveButton;
//    private boolean shootersGunnaShoot = false;
//
//    private double shooterSpeed = 0;
//    private double shooterAngle = 1;
//    private double output = 0;
//
//    @Override
//    public void runOpMode() {
//        boolean areWeWinners = true;
//
//        m_Driver = new OmegaController(gamepad1);
//        m_Operator = new OmegaController(gamepad2);
//
//        zeroGyroButton = new GamepadButton(m_Driver, GamepadKeys.Button.START);
//        autoDriveButton = new GamepadButton(m_Driver, GamepadKeys.Button.Y);
//
//        s_Swerve = new Swerve(hardwareMap, telem);
//        s_Intake = new Intake(hardwareMap, telem);
//        s_Feeder = new Feeder(hardwareMap, telem);
//        s_Turret = new Turret(hardwareMap, telem);
//        s_Shooter = new Shooter(hardwareMap, telem);
//        s_Sparky = new OTOSSensor(hardwareMap, telem);
//        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
//
//        s_Sparky.configureOTOS(new SparkFunOTOS.Pose2D(0, 0, 0));
//
//        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, m_Driver, m_Operator);
//        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Lime, m_Driver, m_Operator, telem);
//        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, s_Sparky, m_Operator, m_Driver, telem, areWeWinners);
//
//        driveCommand.initialize();
//        intakeCommand.initialize();
//        turretCommand.initialize();
//
//        telem.putTelemetry("FPS", s_Lime.getLimeStatus().getFps());
//        telem.updateTelemetry();
//
//        waitForStart();
//
//        if(isStopRequested()) {
//            s_Sparky.disable();
//        }
//
//        while (opModeIsActive()) {
//
//            s_Sparky.skadoodle();
//            s_Lime.skadoodle();
//
//            driveCommand.execute();
//            intakeCommand.execute();
//            turretCommand.execute();
//
//            if (m_Driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//                shootersGunnaShoot = !shootersGunnaShoot;
//            }
//
//            if (!shootersGunnaShoot) {
//                output = 0;
//            } else {
//                output = shooterSpeed;
//            }
//
//            if (m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
//                shooterSpeed += 0.01;
//            }
//            if (m_Operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
//                shooterSpeed -= 0.01;
//            }
//
//            if (m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//                shooterAngle = Constants.ShooterConstants.closeAngle;
//            }
//
//            if (m_Operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                shooterAngle = Constants.ShooterConstants.farAngle;
//            }
//
//            if (s_Lime.isValidReaing()) {
//                shooterAngle = s_Shooter.getShooterAngleFromDistance(s_Lime.getGoalDistance());
//            }
//
//            s_Shooter.setShooterAngle(shooterAngle);
//            s_Shooter.setShooterSpeed(output);
//
//
//            if(m_Driver.wasJustPressed(GamepadKeys.Button.BACK)) {
//                s_Sparky.zeroGyro();
//                s_Swerve.zeroGyro();
//            }
//
//            OmegaPose2D currentPose = s_Sparky.getPose();
//            telem.putTelemetry("X pose", currentPose.x());
//            telem.putTelemetry("Y pose", currentPose.y());
//            telem.putTelemetry("Distance", s_Lime.getFilteredDistance());
//            telem.putTelemetry("Shooter Target Percentage", shooterSpeed);
//
//            telem.updateTelemetry();
//
//        }
//        s_Sparky.disable();
//    }
//}
package org.firstinspires.ftc.teamcode.TeleOp.TestingModes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.Commands.TestTurret;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

@TeleOp(name = "RED TEST", group = "Testing")
public class REDTEST extends LinearOpMode {

    private EZTelemetry telem;
    private OmegaController driver;
    private OmegaController operator;

    private Swerve s_Swerve;
    private FusionOdometry s_Lemon;
    private Intake s_Intake;
    private Feeder s_Feeder;
    private Turret s_Turret;
    private Shooter s_Shooter;

    private boolean shootersGunnaShoot = false;

    private double shooterSpeed = 0;
    private double shooterAngle = 1;
    private double output = 0;

    @Override
    public void runOpMode(){

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Lemon = new FusionOdometry(hardwareMap, telem);

        s_Swerve = new Swerve(hardwareMap, telem, s_Lemon);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Lemon.toggleTelemetry();
        s_Lemon.setPose(new OmegaPose2D(0, 0, 0));

        telem.updateAll();

        waitForStart();

        while (opModeIsActive()) {

            driver.readButtons();
            operator.readButtons();

            s_Lemon.skadoodle();

            if (driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                shootersGunnaShoot = !shootersGunnaShoot;
            }

            if (!shootersGunnaShoot) {
                output = 0;
            } else {
                output = shooterSpeed;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                shooterSpeed += 0.01;
            }
            if (operator.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                shooterSpeed -= 0.01;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                shooterAngle = Constants.ShooterConstants.closeAngle;
            }

            if (operator.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                shooterAngle = Constants.ShooterConstants.farAngle;
            }

            if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                s_Turret.setSetpoint(s_Turret.getSetpoint() + 0.5);
            }

            if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                s_Turret.setSetpoint(s_Turret.getSetpoint() - 0.5);
            }

            s_Turret.runToSetpoint();
            s_Shooter.setShooterAngle(shooterAngle);
            s_Shooter.setShooterSpeed(output);

            if(driver.wasJustPressed(GamepadKeys.Button.BACK)) {
                s_Swerve.zeroGyro();
                s_Lemon.zeroGyro();
            }

            if(driver.wasJustPressed(GamepadKeys.Button.A)) {
                s_Intake.setSpeed(1);
                s_Feeder.setFeederSpeed(1);
                s_Feeder.openGate();
            }

            if(driver.wasJustPressed(GamepadKeys.Button.B)) {
                s_Intake.setSpeed(1);
                s_Feeder.closeGate();
                s_Feeder.setFeederSpeed(0);
            }

            if(driver.wasJustPressed(GamepadKeys.Button.Y)) {
                s_Intake.setSpeed(0);
                s_Feeder.closeGate();
                s_Feeder.setFeederSpeed(0);
            }

            telem.putTelemetry("Shooter Target Percentage", shooterSpeed);
            telem.putDashboard("Shooter Speed", s_Shooter.getShooterVelocity());
            telem.updateAll();

            }

    }

}
