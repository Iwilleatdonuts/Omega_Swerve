package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Feeder extends SubsystemBase {
    private final CRServoImplEx feederServo;
    private final ServoImplEx feederGateServo;
    private final Telemetry telemetry;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    public Feeder(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        feederServo = hardwareMap.get(CRServoImplEx.class, Constants.IntakeConstants.feederServo);
        feederGateServo = hardwareMap.get(ServoImplEx.class, Constants.IntakeConstants.gateServo);

        feederServo.setDirection(DcMotorSimple.Direction.FORWARD);
        feederGateServo.setDirection(Servo.Direction.FORWARD);

        feederGateServo.setPosition(Constants.IntakeConstants.gateClosed);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

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

    @Override
    public void periodic(){

        if(enableTelemetry) {
            telemetry.addLine("Feeder");
            telemetry.addData("Feeder Speed", feederServo.getPower());
            telemetry.addLine();
        }

    }

}
