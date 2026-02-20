package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoCloseShot;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoDirectIntake;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoGate;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoMediumShot;
import org.firstinspires.ftc.teamcode.AutoCommands.AutoTurret;
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

@Autonomous(name = "Blue Close 12 Overflow")
public class BlueClose12Overflow extends LinearOpMode {

    @Override
    public void runOpMode() {

        boolean areWeWinners = false;
        int motif = 0;

        EZTelemetry telem = new EZTelemetry(telemetry);

        Limelight s_Lime = new Limelight(hardwareMap, telem, areWeWinners);
        FusionOdometry s_Lemon = new FusionOdometry(hardwareMap, telem);
        s_Lemon.setPose(Constants.NewAutoConstants.BlueConstants.closeStart);
        s_Lemon.toggleTelemetry();

        Swerve s_Swerve = new Swerve(hardwareMap, telem, s_Lemon);
        Shooter s_Shooter = new Shooter(hardwareMap, telem);
        Turret s_Turret = new Turret(hardwareMap, telem);
        Intake s_Intake = new Intake(hardwareMap, telem);
        Feeder s_Feeder = new Feeder(hardwareMap, telem);

        AutoCloseShot autoShootCommand = new AutoCloseShot(s_Swerve, s_Shooter, s_Intake, s_Feeder, s_Lemon, telem, areWeWinners);
        AutoMediumShot autoMediumShot = new AutoMediumShot(s_Swerve, s_Turret, s_Shooter, s_Intake, s_Feeder, s_Lime, s_Lemon, telem, areWeWinners);
        AutoDirectIntake intakeCommand = new AutoDirectIntake(s_Swerve, s_Intake, s_Feeder, s_Lemon, telem, areWeWinners, 1);
        AutoGate gateCommand = new AutoGate(s_Swerve, s_Lemon, telem, areWeWinners);
        AutoTurret turretCommand = new AutoTurret(s_Turret, s_Lemon, s_Lime, areWeWinners, false);

        int phase = 0;

        int[][] intakeMap = new int[][]{
                {1, 2, 3},
                {2, 1, 3},
                {3, 1, 2},
        };

        List<AutoManager> autoCommands = Arrays.asList(
                () -> {autoShootCommand.reset(false); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(intakeMap[1][0]); return true;},
                intakeCommand::runCommand,
                () -> {autoShootCommand.reset(false); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(intakeMap[1][1]); return true;},
                intakeCommand::runCommand,
                () -> {autoShootCommand.reset(false); return true;},
                autoShootCommand::runCommand,
                () -> {intakeCommand.reset(intakeMap[1][2]); return true;},
                intakeCommand::runCommand,
                () -> {autoMediumShot.reset(); return true;},
                autoMediumShot::runCommand
        );

        telem.putLine("RED CLOSE 12 IS READY");
        telem.updateTelemetry();

        while(opModeInInit()) {

            s_Lime.skadoodle();

            if(s_Lime.isValidReaing()) {
                motif = s_Lime.getTagID();
                final int intakeTarget;
                switch (motif){
                    case 21:
                        intakeTarget = 2;
                        break;
                    case 22:
                        intakeTarget = 1;
                        break;
                    default:
                        intakeTarget = 0;
                        break;
                }

                autoCommands = Arrays.asList(
                        () -> {autoShootCommand.reset(false); return true;},
                        autoShootCommand::runCommand,
                        () -> {intakeCommand.reset(intakeMap[intakeTarget][0]); return true;},
                        intakeCommand::runCommand,
                        () -> {autoShootCommand.reset(true); return true;},
                        autoShootCommand::runCommand,
                        () -> {intakeCommand.reset(intakeMap[intakeTarget][1]); return true;},
                        intakeCommand::runCommand,
                        () -> {autoShootCommand.reset(false); return true;},
                        autoShootCommand::runCommand,
                        () -> {intakeCommand.reset(intakeMap[intakeTarget][2]); return true;},
                        intakeCommand::runCommand,
                        () -> {autoMediumShot.reset(); return true;},
                        autoMediumShot::runCommand
                );
            }

            telem.putTelemetry("Motif Target", motif);
            telem.putTelemetry("Turret Heading", s_Turret.getDegrees());
            telem.updateTelemetry();

        }

        waitForStart();

        while (opModeIsActive()) {

            s_Lemon.skadoodle();
            turretCommand.execute();
            telem.updateAll();

            if(phase < autoCommands.size()) {
                boolean isFinished = autoCommands.get(phase).run();
                if(isFinished) {
                    phase++;
                }
            }

        }

    }
}