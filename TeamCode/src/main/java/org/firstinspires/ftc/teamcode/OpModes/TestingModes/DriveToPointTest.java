package org.firstinspires.ftc.teamcode.OpModes.TestingModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToDashboardPoint;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Auto Drive Test", group = "Testing")
public class DriveToPointTest extends LinearOpMode {

    private final EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private OTOSSensor s_Sparky;

    private DriveToDashboardPoint driveCommand;

    @Override
    public void runOpMode() {

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);

        s_Sparky.toggleTelemetry();
        s_Sparky.configureOTOS(new SparkFunOTOS.Pose2D(0, 0, 0));

        driveCommand = new DriveToDashboardPoint(s_Swerve, s_Sparky, telem);

        driveCommand.initialize();

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
