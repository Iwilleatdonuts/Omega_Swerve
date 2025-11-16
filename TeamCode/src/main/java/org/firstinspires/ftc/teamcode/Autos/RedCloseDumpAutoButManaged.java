package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoCloseShot;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoGate;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoTurret;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVisionOnTurret;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.AutoManager;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Close Dump Clean")
public class RedCloseDumpAutoButManaged extends LinearOpMode {

    @Override
    public void runOpMode() {

        boolean areWeWinners = true;

        EZTelemetry telem = new EZTelemetry(telemetry);

        AprilVisionOnTurret s_Vision = new AprilVisionOnTurret(hardwareMap, telem, areWeWinners);
        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.RedConstants.closeStart));
        s_Sparky.toggleTelemetry();

        Swerve s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        Shooter s_Shooter = new Shooter(hardwareMap, telem);
        Turret s_Turret = new Turret(hardwareMap, telem);
        Intake s_Intake = new Intake(hardwareMap, telem);
        Feeder s_Feeder = new Feeder(hardwareMap, telem);

        AutoCloseShot autoShootCommand = new AutoCloseShot(s_Swerve, s_Shooter, s_Intake, s_Feeder, s_Sparky, telem, areWeWinners);
        AutoIntake intakeCommand = new AutoIntake(s_Swerve, s_Intake, s_Sparky, telem, areWeWinners, 1);
        AutoGate gateCommand = new AutoGate(s_Swerve, s_Sparky, telem, areWeWinners);
        AutoTurret turretCommand = new AutoTurret(s_Turret, s_Vision);

        int phase = 0;

        List<AutoManager> autoCommands = Arrays.asList(
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(1); return true;},
                intakeCommand::runCommand,
                () -> {gateCommand.reset(false); return true;},
                gateCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(2); return true;},
                intakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(3); return true;},
                intakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {gateCommand.reset(true); return true;},
                gateCommand::runCommand


        );

        telem.putTelemetry("FPS", s_Vision.getCameraFPS());
        telem.updateTelemetry();

        waitForStart();

        if(isStopRequested()){
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            s_Sparky.skadoodle();
            s_Vision.skadoodle();
            turretCommand.execute();
            telem.updateAll();

            if(phase < autoCommands.size()) {
                boolean isFinished = autoCommands.get(phase).run();
                if(isFinished) {
                    phase++;
                }
            }

        }

        s_Sparky.disable();

    }
}