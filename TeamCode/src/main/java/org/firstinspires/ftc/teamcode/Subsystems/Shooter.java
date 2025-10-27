package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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

    private final FtcDashboard dashboard;

    private final double kS = 0.01;      // static friction term
    private final double kV = 1/Constants.ShooterConstants.MAX_TICKS_PER_SEC;    //ticks per second
    private final double kA = 0.0;

    private final SimpleMotorFeedforward shooterFF;

    public Shooter(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

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

        angleServo.setPosition(Constants.ShooterConstants.aimerDown);

        dashboard = FtcDashboard.getInstance();

        shooterFF = new SimpleMotorFeedforward(PIDTuning.kF,1/Constants.ShooterConstants.MAX_TICKS_PER_SEC, 0);

    }

    //input from 0-1
    public void setShooterSpeed(double speed) {
//        double output = shooterController.calculate(getLowerVelocity(), speed * Constants.ShooterConstants.maxSpeed);
////        double output = speed;
//        output = Math.max(-1.0, Math.min(1.0, output));
//        upperShooterMotor.setPower(output);
//        lowerShooterMotor.setPower(output);
        double targetVelocity = speed * Constants.ShooterConstants.MAX_TICKS_PER_SEC;
        double ff = shooterFF.calculate(targetVelocity);

        double PID = shooterController.calculate(getLowerVelocity(), targetVelocity);

        double output = ff + PID;

        if(enableTelemetry){
            telemetry.addData("Target Velocity:", targetVelocity);
            telemetry.addData("shooter speed", getLowerVelocity());
        }


        lowerShooterMotor.setPower(output);
        upperShooterMotor.setPower(output);
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

    public double getShooterSpeedFromDistance(double distance) {
        return 0.00185564 * distance +0.380296;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void periodic() {

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Shooter Ticks per second", getLowerVelocity());
        dashboard.sendTelemetryPacket(packet);


        if(enableTelemetry) {
            telemetry.addLine("Shooter");
            telemetry.addData("RPM", getAverageVelocity());
            telemetry.addData("Shooter Angle", getShooterAngle());
        }

    }

}
