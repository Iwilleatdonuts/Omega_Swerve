package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class Shooter {

    private final EZTelemetry telem;

    private final DcMotorEx upperShooterMotor;
    private final DcMotorEx lowerShooterMotor;
    private final ServoImplEx angleServo;

    private boolean enableTelemetry;

    private final PIDController shooterController;

    private final SimpleMotorFeedforward shooterFF;

    private double targetVelocity;

    private double speedConstant;

    public Shooter(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        shooterController = new PIDController(0.008, 0.0005, 0);
//        shooterController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);

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

        angleServo.setPosition(Constants.ShooterConstants.closeAngle);

        shooterFF = new SimpleMotorFeedforward(0.005,1/Constants.ShooterConstants.MAX_TICKS_PER_SEC, 0);

        speedConstant = 0.282878;
        targetVelocity = 0;

    }

    //input from 0-1
    public void setShooterSpeed(double speed) {
        targetVelocity = speed * 2100;
        double ff = shooterFF.calculate(targetVelocity);

        double PID = shooterController.calculate(getShooterVelocity(), targetVelocity);

        double output = ff + PID;

        lowerShooterMotor.setPower(output);
        upperShooterMotor.setPower(output);
    }

    public double getShooterVelocity() {
        return lowerShooterMotor.getVelocity();
    }
    public boolean shooterAtSpeed() {
        return Math.abs(getShooterVelocity() - targetVelocity) < 40;
    }

    public boolean shooterAtRoughSpeed() {
        return Math.abs(getShooterVelocity() - targetVelocity) < 80;
    }

    public void setShooterAngle(double degrees) {
        double limited = Math.max(Math.min(1, degrees), 0);
        angleServo.setPosition(limited);
    }

    public double getShooterAngle(){
        return angleServo.getPosition();
    }

    public void incrementSpeedConstant() {
        speedConstant += 0.001;
    }

    public void decrementSpeedConstant() {
        speedConstant -= 0.001;
    }

    public double getShooterConstant() {
        return speedConstant;
    }

    public double getShooterSpeedFromDistance(double distance) {
        if(distance > 76){
            return 0.000000451637 * distance * distance * distance - 0.000129113 * distance * distance + 0.013132 * distance + 0.0580873;
        } else if (distance > 44) {
            return 0.0000492152 * distance * distance - 0.00121422 * distance + 0.317641;
        } else {
            return 0.36;
        }
    }

    public double getShooterAngleFromDistance(double distance) {
        return -0.0216685 * distance + 1.65005;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle() {

        if(enableTelemetry) {

            telem.putTelemetry("Target Velocity", targetVelocity);
            telem.putTelemetry("Shooter Velocity", getShooterVelocity());
            telem.putTelemetry("Shooter Constant", getShooterConstant());
//
            telem.putDashboard("Target Velocity", targetVelocity);
            telem.putDashboard("Shooter Velocity", getShooterVelocity());
            telem.putDashboard("Shooter Constant", getShooterConstant());

        }

    }

}
