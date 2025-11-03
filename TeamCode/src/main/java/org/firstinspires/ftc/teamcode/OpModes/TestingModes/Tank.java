package org.firstinspires.ftc.teamcode.OpModes.TestingModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "Tank", group = "Testing")
@Disabled
public class Tank extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("drive0");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("drive2");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("drive1");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("drive3");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double yLeft = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double yRight = gamepad1.right_stick_y;

            frontLeftMotor.setPower(yLeft);
            backLeftMotor.setPower(yRight);
            frontRightMotor.setPower(yLeft);
            backRightMotor.setPower(yRight);
        }
    }
}