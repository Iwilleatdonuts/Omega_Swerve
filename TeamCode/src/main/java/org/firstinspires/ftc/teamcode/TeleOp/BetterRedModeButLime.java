package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
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
import org.firstinspires.ftc.teamcode.Utilities.Controller.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

@TeleOp(name = "Ginger Drive Core But Green", group = "Main")
public class BetterRedModeButLime extends LinearOpMode {

    private EZTelemetry telem;
    private OmegaController driver;
    private OmegaController operator;

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

    @Override
    public void runOpMode(){

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Lime = new Limelight(hardwareMap, telem, true);

        s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Sparky.toggleTelemetry();
        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.gateLineupTeleop));

        driveCommand = new TurnToPointDrive(telem, s_Swerve, s_Sparky, driver, operator);
//        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, s_Vision, driver, operator, telem);
        turretCommand = new LimeTurret(s_Swerve, s_Turret, s_Lime, operator);
//        shooterCommand = new CoolShooters(s_Shooter, s_Vision, driver, operator, telem);

        driveCommand.initialize();
//        intakeCommand.initialize();
        turretCommand.initialize();
//        shooterCommand.initialize();

        s_Lime.startLime();

        waitForStart();

        if(isStopRequested()) {
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            long loopStart = System.nanoTime();

            driveCommand.execute();
            turretCommand.execute();

            if(driver.wasJustPressed(GamepadKeys.Button.START)) {
                s_Swerve.zeroGyro();
                s_Sparky.zeroGyro();
            }

            telem.updateTelemetry();
        }

        s_Sparky.disable();
        s_Lime.stopLime();

    }

}
