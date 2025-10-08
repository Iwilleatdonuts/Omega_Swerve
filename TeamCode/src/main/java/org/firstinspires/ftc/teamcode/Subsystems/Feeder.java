package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Feeder extends SubsystemBase {
    private final CRServoImplEx feederServo;
    private final Telemetry telemetry;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    public Feeder(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        feederServo = hardwareMap.get(CRServoImplEx.class, Constants.IntakeConstants.feederServo);

        feederServo.setDirection(DcMotorSimple.Direction.FORWARD);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

    }


    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void setFeederSpeed(double speed) {
        feederServo.setPower(speed);
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
