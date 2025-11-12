package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Turret {

    private final EZTelemetry telem;

    private final DcMotorEx turretMotor;

    private boolean enableTelemetry;

    private double setpoint;

    private final PIDController turretController;

    public Turret(HardwareMap hardwareMap, EZTelemetry telem) {

        this.telem = telem;

        turretMotor = hardwareMap.get(DcMotorEx.class, Constants.TurretConstants.turretMotor);

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        enableTelemetry = false;

        turretController = new PIDController(0.02, 0.01, 0);
        turretController.setTolerance(5);

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
            double angle = newSetpoint + (360.0 * i);
            if (angle >= lowerLimit && angle <= upperLimit) {
                double error = Math.abs(angle - currentRotation);
                if (error < smallestError) {
                    smallestError = error;
                    nearestAngle = angle;
                }
            }
        }

        nearestAngle = Math.min(Constants.TurretConstants.upperRotationLimit, Math.max(Constants.TurretConstants.lowerRotationLimit, nearestAngle));

        setpoint = nearestAngle;

    }

    public double getSetpoint() {
        return setpoint;
    }

    public void runToSetpoint() {

        double output = 0;

        if(Math.abs(getDegrees() - getSetpoint()) > 1){
            output = turretController.calculate(getDegrees(), getSetpoint());
        }

        output = Math.max(-1.0, Math.min(1.0, output));
        setSpeed(output);
    }

    public boolean atSetpoint() {
        return Math.abs(getDegrees() - getSetpoint()) < 1;
    }

    public boolean isOutOfBounds() {
        double currentAngle = getDegrees();
        return currentAngle < Constants.TurretConstants.lowerRotationLimit || currentAngle > Constants.TurretConstants.upperRotationLimit;
    }

    public void skadoodle(){

        if(enableTelemetry) {

            telem.putTelemetry("Turret Angle", getDegrees());

            telem.putDashboard("Turret Angle", getDegrees());

        }

    }

}
