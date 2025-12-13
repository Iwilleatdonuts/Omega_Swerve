package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoCornerIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoDirectIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoFarShot;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoSwoopyIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoTurret;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoWaitCommand;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.AutoManager;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Red Far Support Spike")
public class RedFarSupportSpike extends LinearOpMode {

    @Override
    public void runOpMode() {

        boolean areWeWinners = true;

        EZTelemetry telem = new EZTelemetry(telemetry);

        Limelight s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        FusionOdometry s_Lemon = new FusionOdometry(hardwareMap, telem);
        s_Lemon.setPose(Constants.AutoConstants.RedConstants.farStart);
        s_Lemon.toggleTelemetry();

        Swerve s_Swerve = new Swerve(hardwareMap, telem, s_Lemon);
        Shooter s_Shooter = new Shooter(hardwareMap, telem);
        Turret s_Turret = new Turret(hardwareMap, telem);
        Intake s_Intake = new Intake(hardwareMap, telem);
        Feeder s_Feeder = new Feeder(hardwareMap, telem);

        AutoFarShot autoShootCommand = new AutoFarShot(s_Swerve, s_Shooter, s_Turret, s_Intake, s_Feeder, s_Lime, s_Lemon, telem, areWeWinners);
        AutoCornerIntake cornerIntakeCommand = new AutoCornerIntake(s_Swerve, s_Intake, s_Feeder, s_Lemon, telem, areWeWinners);
        AutoSwoopyIntake swoopyIntakeCommand = new AutoSwoopyIntake(s_Swerve, s_Intake, s_Feeder, s_Lemon, telem, areWeWinners);
        AutoTurret turretCommand = new AutoTurret(s_Turret, s_Lime, areWeWinners, true);
        AutoDirectIntake intakeCommand = new AutoDirectIntake(s_Swerve, s_Intake, s_Feeder, s_Lemon, telem, areWeWinners, 1);
        AutoWaitCommand waitCommand = new AutoWaitCommand();

        int phase = 0;

        List<AutoManager> autoCommands = Arrays.asList(
//                () -> {waitCommand.reset(25e9); return true;},
//                waitCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(3); return true;},
                intakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {cornerIntakeCommand.reset(); return true;},
                cornerIntakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {swoopyIntakeCommand.reset(); return true;},
                swoopyIntakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand,
                () -> {swoopyIntakeCommand.reset(); return true;},
                swoopyIntakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand
        );

        telem.updateTelemetry();

        waitForStart();

        while (opModeIsActive()) {

            s_Lemon.skadoodle();
            s_Lime.skadoodle();
            turretCommand.execute();
            telem.updateAll();

            if (phase < autoCommands.size()) {
                boolean isFinished = autoCommands.get(phase).run();
                if (isFinished) {
                    phase++;
                }
            }

        }

    }
}