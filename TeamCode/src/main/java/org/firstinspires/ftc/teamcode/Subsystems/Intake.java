package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.math.MathUtil;

public class Intake {

    private final EZTelemetry telem;

    private final DcMotorEx intakeMotor;

    private boolean enableTelemetry;

    private double targetSpeed;

    public Intake(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.intakeMotor);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enableTelemetry = false;

        targetSpeed = 0;

    }

    public void setTargetSpeed(double target) {
        targetSpeed = MathUtil.clamp(target, -1, 1);
    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }


    public double getIntakeCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }


    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        if(enableTelemetry) {
            telem.putTelemetry("Intake Current", getIntakeCurrent());

            telem.putDashboard("Intake Current", getIntakeCurrent());
        }

    }

}
