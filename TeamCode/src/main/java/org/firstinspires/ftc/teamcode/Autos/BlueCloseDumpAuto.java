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
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

@Autonomous(name = "Blue Close Dump")
public class BlueCloseDumpAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        boolean areWeWinners = false;

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

            switch (phase) {
                case 0:
                    autoShootCommand.reset();
                    phase++;
                    break;
                case 1:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(1);
                        phase++;
                    }
                    break;
                case 2:
                    intakeCommand.execute();
                    if(intakeCommand.isFinished()){
                        gateCommand.reset(false);
                        phase++;
                    }
                    break;
                case 3:
                    gateCommand.execute();
                    if(gateCommand.isFinished()){
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 4:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(2);
                        phase++;
                    }
                    break;
                case 5:
                    intakeCommand.execute();
                    if(intakeCommand.isFinished()){
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 6:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(3);
                        phase++;
                    }
                    break;
                case 7:
                    intakeCommand.execute();
                    if(intakeCommand.isFinished()){
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 8:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        gateCommand.reset(true);
                        phase++;
                    }
                    break;
                case 9:
                    gateCommand.execute();
                    break;
            }
        }

        s_Sparky.disable();

    }
}