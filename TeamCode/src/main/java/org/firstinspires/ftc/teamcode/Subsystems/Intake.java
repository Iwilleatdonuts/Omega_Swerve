package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Intake extends SubsystemBase {

    private final EZTelemetry telem;

    private final DcMotorEx intakeMotor;
    private final DigitalChannel ballSensor;

    private boolean enableTelemetry;

    public Intake(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        intakeMotor = hardwareMap.get(DcMotorEx.class, Constants.IntakeConstants.intakeMotor);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ballSensor = hardwareMap.get(DigitalChannel.class, "ballSensor");
        ballSensor.setMode(DigitalChannel.Mode.INPUT);

        enableTelemetry = false;

    }

    public void setSpeed(double speed) {
        intakeMotor.setPower(speed);
    }

    public double getSpeed() {
        return intakeMotor.getVelocity();
    }

    public double getIntakeCurrent() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }


    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        if(enableTelemetry) {

            telem.putTelemetry("Intake Speed", getSpeed());
            telem.putTelemetry("Intake Current", getIntakeCurrent());

            telem.putTelemetry("Ball Sensor Value", ballSensor .getState());

            telem.putDashboard("Intake Speed", getSpeed());
            telem.putDashboard("Intake Current", getIntakeCurrent());
        }

    }

}
