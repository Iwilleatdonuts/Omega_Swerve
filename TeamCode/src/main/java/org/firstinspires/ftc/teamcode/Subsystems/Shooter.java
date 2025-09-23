package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Shooter extends SubsystemBase {

    private final DcMotorEx upperShooterMotor;
    private final DcMotorEx lowerShooterMotor;

    private final Telemetry telemetry;
    private boolean enableTelemetry;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        upperShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.upperMotor);
        lowerShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.lowerMotor);

        upperShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upperShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowerShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        upperShooterMotor.setVelocityPIDFCoefficients(1, 0, 0, 0);
        lowerShooterMotor.setVelocityPIDFCoefficients(1, 0, 0, 0);

        enableTelemetry = false;

    }

    //input from 0-1
    public void setShooterSpeed(double speed) {
//        double velocity = speed * 6000;
//        upperShooterMotor.setVelocity(velocity);
//        lowerShooterMotor.setVelocity(velocity);
        upperShooterMotor.setPower(speed);
        lowerShooterMotor.setPower(speed);
    }

    public double getUpperVelocity() {
        return upperShooterMotor.getVelocity();
    }

    public double getLowerVelocity() {
        return lowerShooterMotor.getVelocity();
    }

    public double getAverageVelocity(){
        return (getUpperVelocity()+getLowerVelocity())/2;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void periodic() {

        if(enableTelemetry) {
            telemetry.addLine("Shooter");
            telemetry.addData("RPM", getAverageVelocity());
        }

    }

}
