package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Intake extends SubsystemBase {

    private final EZTelemetry telem;

    private final DcMotorEx intakeMotor;

    private boolean enableTelemetry;

    public Intake(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.intakeMotor);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enableTelemetry = false;

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

            telem.putTelemetry("Intake Speed", getSpeed());

            telem.putDashboard("Intake Speed", getSpeed());
        }

    }

}
