package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {

    private final DcMotorEx intakeMotor;
    private final Telemetry telemetry;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.intakeMotor);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    public double getSpeed() {
        return intakeMotor.getVelocity();
    }


    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    @Override
    public void periodic(){

        if(enableTelemetry) {
            telemetry.addLine("Intake");
            telemetry.addData("Intake Speed", getSpeed());
            telemetry.addLine();
        }

    }

}
