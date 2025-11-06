package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class Shooter extends SubsystemBase {

    private final EZTelemetry telem;

    private final DcMotorEx upperShooterMotor;
    private final DcMotorEx lowerShooterMotor;
    private final ServoImplEx angleServo;

    private boolean enableTelemetry;

    private final PIDController shooterController;

    private final double kS = 0.01;      // static friction term
    private final double kV = 1/Constants.ShooterConstants.MAX_TICKS_PER_SEC;    //ticks per second
    private final double kA = 0.0;

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

        shooterFF = new SimpleMotorFeedforward(0.5,1/Constants.ShooterConstants.MAX_TICKS_PER_SEC, 0);

        speedConstant = 0.295;
        targetVelocity = 0;

    }

    //input from 0-1
    public void setShooterSpeed(double speed) {
        targetVelocity = speed * Constants.ShooterConstants.MAX_TICKS_PER_SEC;
        double ff = shooterFF.calculate(targetVelocity);

        double PID = shooterController.calculate(getShooterVelocity(), targetVelocity);

        double output = ff + PID;

        lowerShooterMotor.setPower(output);
        upperShooterMotor.setPower(output);
    }

    public double getShooterVelocity() {
        return lowerShooterMotor.getVelocity();
    }

    public void setShooterAngle(double degrees) {
        angleServo.setPosition(degrees);
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
//        return 0.00185564 * distance +0.380296;
                return 0.00185564 * distance + speedConstant;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void periodic() {

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
