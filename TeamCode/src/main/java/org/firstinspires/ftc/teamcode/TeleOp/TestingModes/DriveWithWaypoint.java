package org.firstinspires.ftc.teamcode.TeleOp.TestingModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToDashboardPoint;
import org.firstinspires.ftc.teamcode.Commands.DriveToDashboardPointWithSwerb;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Auto Drive Test", group = "Testing")
public class DriveWithWaypoint extends LinearOpMode {

    private final EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private Intake s_Intake;
    private OTOSSensor s_Sparky;

    private DriveToDashboardPointWithSwerb driveCommand;

    @Override
    public void runOpMode() {

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);

        s_Sparky.toggleTelemetry();
        s_Sparky.configureOTOS(new SparkFunOTOS.Pose2D(0, 0, 0));

        driveCommand = new DriveToDashboardPointWithSwerb(s_Swerve, s_Intake, s_Sparky, telem);

        driveCommand.initialize();
        s_Sparky.zeroGyro();
        s_Swerve.zeroGyro();

        waitForStart();

        if(isStopRequested()) {
            s_Sparky.disable();
        }

        while(opModeIsActive()){

            driveCommand.execute();

            s_Sparky.skadoodle();

            telem.updateAll();
        }
        s_Sparky.disable();
    }
}
