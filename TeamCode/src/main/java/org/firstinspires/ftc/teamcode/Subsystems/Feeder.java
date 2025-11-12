package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Feeder {

    private final EZTelemetry telem;
    private final CRServoImplEx feederServo;
    private final ServoImplEx feederGateServo;

    private boolean enableTelemetry;

    public Feeder(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        feederServo = hardwareMap.get(CRServoImplEx.class, Constants.IntakeConstants.feederServo);
        feederGateServo = hardwareMap.get(ServoImplEx.class, Constants.IntakeConstants.gateServo);

        feederServo.setDirection(DcMotorSimple.Direction.FORWARD);
        feederGateServo.setDirection(Servo.Direction.FORWARD);

        feederGateServo.setPosition(Constants.IntakeConstants.gateClosed);

        enableTelemetry = false;

    }


    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void setFeederSpeed(double speed) {
        feederServo.setPower(speed);
    }

    public void openGate() {
        feederGateServo.setPosition(Constants.IntakeConstants.gateOpen);
    }

    public void closeGate() {
        feederGateServo.setPosition(Constants.IntakeConstants.gateClosed);
    }

    public void skadoodle(){

        if(enableTelemetry) {

            telem.putTelemetry("Feeder Speed", feederServo.getPower());

            telem.putDashboard("Feeder Speed", feederServo.getPower());

        }

    }

}
