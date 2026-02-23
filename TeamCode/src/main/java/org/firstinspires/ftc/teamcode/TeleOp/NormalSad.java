package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.CoolShooters;
import org.firstinspires.ftc.teamcode.Commands.LemonTurret;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.SmartIntake;
import org.firstinspires.ftc.teamcode.Commands.ManualCommands.TeleOpDrive;
import org.firstinspires.ftc.teamcode.Constants;
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

@TeleOp(name = "Blue Normie", group = "Main")
public class NormalSad extends LinearOpMode {

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

    private TeleOpDrive driveCommand;
    private SmartIntake intakeCommand;
    private LemonTurret turretCommand;
    private CoolShooters shooterCommand;

    @Override
    public void runOpMode(){
        boolean areWeWinners = false;

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        s_Lime.startLime();

        s_Lemon = new FusionOdometry(hardwareMap, telem);

        s_Swerve = new Swerve(hardwareMap, telem, s_Lemon);
        s_Intake = new Intake(hardwareMap, telem);
        s_Feeder = new Feeder(hardwareMap, telem);
        s_Turret = new Turret(hardwareMap, telem);
        s_Shooter = new Shooter(hardwareMap, telem);

        s_Turret.toggleTelemetry();

        driveCommand = new TeleOpDrive(telem, s_Swerve, driver, operator);
        intakeCommand = new SmartIntake(s_Intake, s_Feeder, s_Shooter, s_Turret, driver, operator, telem);
        turretCommand = new LemonTurret(s_Swerve, s_Turret, s_Lemon, operator, driver, telem, areWeWinners);
        shooterCommand = new CoolShooters(s_Shooter, s_Lemon, driver, operator, telem, areWeWinners);

        driveCommand.initialize();
        intakeCommand.initialize();
        turretCommand.initialize();
        shooterCommand.initialize();

        telem.putLine("TS IS READY TO RUMBAH");
        telem.putLine();
        s_Lemon.setPose(Constants.NewAutoConstants.BlueConstants.finalCloseShotTeleopPose);
        telem.updateAll();

        waitForStart();

        if(isStopRequested()) {
            s_Lime.stopLime();
        }

        while (opModeIsActive()) {

            s_Lemon.skadoodle();

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
        }
        s_Lime.stopLime();
    }

}
