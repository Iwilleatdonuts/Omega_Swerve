package org.firstinspires.ftc.teamcode.TeleOp.TestingModes;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToDashboardPoint;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Auto Drive Test", group = "Testing")
public class DriveToPointTest extends LinearOpMode {

    private OmegaController controller;

    private final EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private Intake s_Intake;
    private FusionOdometry s_Lemon;

    private DriveToDashboardPoint driveCommand;

    @Override
    public void runOpMode() {

        controller = new OmegaController(gamepad1);

        s_Swerve = new Swerve(hardwareMap, telem);
        s_Intake = new Intake(hardwareMap, telem);
        s_Lemon = new FusionOdometry(hardwareMap, telem);

        s_Lemon.toggleTelemetry();
        s_Lemon.setPose(new OmegaPose2D(0, 0, 0));

        driveCommand = new DriveToDashboardPoint(s_Swerve, s_Intake, s_Lemon, telem, controller);

        driveCommand.initialize();
        s_Swerve.zeroGyro();

        waitForStart();

        while(opModeIsActive()){

            s_Lemon.skadoodle();

            driveCommand.execute();

            telem.updateAll();
        }
    }
}
