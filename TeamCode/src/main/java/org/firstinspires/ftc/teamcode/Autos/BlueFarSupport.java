package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoCornerIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoFarShot;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoSwoopyIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoTurret;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Feeder;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.AutoManager;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Blue Far Support")
public class BlueFarSupport extends LinearOpMode {

    @Override
    public void runOpMode() {

        boolean areWeWinners = false;

        EZTelemetry telem = new EZTelemetry(telemetry);

        Limelight s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telem);
        s_Sparky.configureOTOS(s_Sparky.normiePoseToSparkyPose(Constants.AutoConstants.BlueConstants.farStart));
        s_Sparky.toggleTelemetry();

        Swerve s_Swerve = new Swerve(hardwareMap, telem, s_Sparky);
        Shooter s_Shooter = new Shooter(hardwareMap, telem);
        Turret s_Turret = new Turret(hardwareMap, telem);
        Intake s_Intake = new Intake(hardwareMap, telem);
        Feeder s_Feeder = new Feeder(hardwareMap, telem);

        AutoFarShot autoShootCommand = new AutoFarShot(s_Swerve, s_Shooter, s_Intake, s_Feeder, s_Lime, s_Sparky, telem, areWeWinners);
        AutoCornerIntake cornerIntakeCommand = new AutoCornerIntake(s_Swerve, s_Intake, s_Feeder, s_Sparky, telem, areWeWinners);
        AutoSwoopyIntake swoopyIntakeCommand = new AutoSwoopyIntake(s_Swerve, s_Intake, s_Feeder, s_Sparky, telem, areWeWinners);
        AutoTurret turretCommand = new AutoTurret(s_Turret, s_Lime, areWeWinners, true);

        int phase = 0;

        List<AutoManager> autoCommands = Arrays.asList(
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
                autoShootCommand::runCommand,
                () -> {swoopyIntakeCommand.reset(); return true;},
                swoopyIntakeCommand::runCommand,
                () -> {autoShootCommand.reset(); return true;},
                autoShootCommand::runCommand
        );

        telem.updateTelemetry();

        waitForStart();

        if(isStopRequested()){
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            s_Sparky.skadoodle();
            s_Lime.skadoodle();
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