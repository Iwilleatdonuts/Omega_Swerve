package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.AutoCommands.AutoCloseShot;
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

@Autonomous(name = "New Red Close Dump")
public class NewRedCloseDumpAuto extends LinearOpMode {

    private EZTelemetry telem;

    private AprilVisionOnTurret s_Vision;
    private OTOSSensor s_Sparky;
    private Swerve s_Swerve;
    private Turret s_Turret;
    private Shooter s_Shooter;
    private Intake s_Intake;
    private Feeder s_Feeder;

    private AutoCloseShot autoShootCommand;
    private AutoIntake intakeCommand;
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

        autoShootCommand = new AutoCloseShot(s_Swerve, s_Shooter, s_Intake, s_Feeder, s_Sparky, telem, true);
        intakeCommand = new AutoIntake(s_Swerve, s_Intake, s_Sparky, telem, true, 1);
        turretCommand = new AutoTurret(s_Turret, s_Vision);

        phase = 0;

        telem.putTelemetry("FPS", s_Vision.getCameraFPS());
        telem.updateTelemetry();

        waitForStart();

        if (isStopRequested()) {
            s_Sparky.disable();
        }

        while (opModeIsActive()) {

            s_Sparky.skadoodle();
            s_Vision.skadoodle();

            turretCommand.execute();
            s_Shooter.setShooterSpeed(0.36);

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
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 3:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(2);
                        phase++;
                    }
                    break;
                case 4:
                    intakeCommand.execute();
                    if(intakeCommand.isFinished()){
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 5:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(3);
                        phase++;
                    }
                    break;
                case 6:
                    intakeCommand.execute();
                    if(intakeCommand.isFinished()){
                        autoShootCommand.reset();
                        phase++;
                    }
                    break;
                case 7:
                    autoShootCommand.execute();
                    if (autoShootCommand.isFinished()) {
                        intakeCommand.reset(3);
                        phase++;
                    }
                    break;

            }

            s_Sparky.disable();

        }
    }
}