package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;

public class Shooter extends SubsystemBase {

    private final DcMotorEx upperShooterMotor;
    private final DcMotorEx lowerShooterMotor;
    private final ServoImplEx angleServo;

    private final Telemetry telemetry;
    private boolean enableTelemetry;

    private final PIDController shooterController;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        shooterController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);

        upperShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.upperMotor);
        lowerShooterMotor = hardwareMap.get(DcMotorEx.class, Constants.ShooterConstants.lowerMotor);
        angleServo = hardwareMap.get(ServoImplEx.class, Constants.ShooterConstants.angleServo);

        upperShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        lowerShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        upperShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upperShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lowerShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        angleServo.setDirection(Servo.Direction.FORWARD);
        angleServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        enableTelemetry = false;

        angleServo.setPosition(0);

    }

    //input from 0-1
    public void setShooterSpeed(double speed) {
//        double output = shooterController.calculate(getUpperVelocity(), speed * 6000);
        double output = speed;
        output = Math.max(-1.0, Math.min(1.0, output));
        upperShooterMotor.setPower(output);
        lowerShooterMotor.setPower(output);
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

    public void setShooterAngle(double degrees) {
        angleServo.setPosition(degrees);
    }

    public double getShooterAngle(){
        return angleServo.getPosition();
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void periodic() {

        if(enableTelemetry) {
            telemetry.addLine("Shooter");
            telemetry.addData("RPM", getAverageVelocity());
            telemetry.addData("Shooter Angle", getShooterAngle());
        }

    }

}
