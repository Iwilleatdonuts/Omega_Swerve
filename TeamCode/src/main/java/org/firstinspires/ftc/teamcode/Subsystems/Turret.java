package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;

public class Turret extends SubsystemBase {

    private final DcMotorEx turretMotor;
    private final Telemetry telemetry;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    private double setpoint;

    private final PIDController turretController;

    public Turret(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        turretMotor = hardwareMap.get(DcMotorEx.class, Constants.TurretConstants.turretMotor);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();


        turretController = new PIDController(0.02, 0.01, 0.0003);

        setSetpoint(getDegrees());
    }

    public void setSpeed(double speed) {
        if(getDegrees() < Constants.TurretConstants.lowerRotationLimit){
            speed = Math.max(speed, 0);
        }
        if(getDegrees() > Constants.TurretConstants.upperRotationLimit){
            speed = Math.min(speed, 0);
        }
        turretMotor.setPower(speed);
    }

    public double getSpeed() {
        return turretMotor.getVelocity();
    }

    public double getRawPosition() {
        return turretMotor.getCurrentPosition();
    }

    public double getDegrees() {
        return getRawPosition() * 0.18725617685;
    }

    public void resetTurretPosition() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void setSetpoint(double newSetpoint){

        double currentRotation = getDegrees();
        double lowerLimit = Constants.TurretConstants.lowerRotationLimit;
        double upperLimit = Constants.TurretConstants.upperRotationLimit;

        double nearestAngle = currentRotation;
        double smallestError = Double.MAX_VALUE;


        int minMultiple = (int)Math.floor((lowerLimit - newSetpoint) / 360.0);
        int maxMultiple = (int)Math.ceil((upperLimit - newSetpoint) / 360.0);

        for(int i = minMultiple; i < maxMultiple; i++) {
            double angle = newSetpoint + 360.0 * i;
            if (angle >= lowerLimit && angle <= upperLimit) {
                double error = Math.abs(angle - currentRotation);
                if (error < smallestError) {
                    smallestError = error;
                    nearestAngle = angle;
                }
            }
        }

        setpoint = nearestAngle;

    }

    public double getSetpoint() {
        return setpoint;
    }

    public void runToSetpoint() {

        double output = turretController.calculate(getDegrees(), getSetpoint());

        output = Math.max(-1.0, Math.min(1.0, output));
        setSpeed(output);
    }

    @Override
    public void periodic(){

        if(enableTelemetry) {
            telemetry.addLine("Turret");
            telemetry.addData("Turret Raw Position", getRawPosition());
            telemetry.addData("Setpoint", getSetpoint());
            telemetry.addData("Turret Degrees", getDegrees());
            telemetry.addLine();
        }

    }

}
