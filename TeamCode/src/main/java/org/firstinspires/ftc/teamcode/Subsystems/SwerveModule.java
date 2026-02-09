package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuner;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;

public class SwerveModule {

    private final EZTelemetry telem;
    private final int modNumber;

    private final DcMotorEx drive;
    private final CRServoImplEx angle;
    private final AnalogInput moduleHeading;
    private final PIDController angleController;
    private final SimpleMotorFeedforward ffController;

    private final double moduleOffset;
    private final double headingMaxVoltage;

    private boolean isModuleBackwards = false;
    private boolean enableTelemetry = false;

    private double lastVelocity = 0.0;
    private double targetVelocityTicksPerSec;

    private final double kS = 0.05;
    private final double kA = 0.0;
    private final double kV = 1.0 / Constants.DriveTrainConstants.MAX_TICKS_PER_SEC;

    private final double moduleFF;

    private final String keyRaw, keyDeg;

    private double moduleSetpoint;

    public SwerveModule(HardwareMap hardwareMap, EZTelemetry telem, SwerveModuleConstants moduleConstants) {

        this.telem = telem;
        this.modNumber = moduleConstants.modNumber;

        moduleFF = moduleConstants.kF;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServoImplEx.class, moduleConstants.angleServo);

        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        this.headingMaxVoltage = moduleHeading.getMaxVoltage(); // cache permanently

        moduleOffset = moduleConstants.moduleOffset;

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        angleController = new PIDController(
                Constants.DriveTrainConstants.angleKP,
                Constants.DriveTrainConstants.angleKI,
                Constants.DriveTrainConstants.angleKD
        );

//        angleController = new PIDController(
//                PIDTuner.PIDTuner1.kP,
//                PIDTuner.PIDTuner1.kI,
//                PIDTuner.PIDTuner1.kD
//        );
        angleController.enableContinuousInput(0, 360);
        angleController.setIZone(30);
        angleController.setIntegratorRange(-0.8, 0.8);

        moduleSetpoint = getDegrees(true);

        ffController = new SimpleMotorFeedforward(0, 0, 0);

        keyRaw = "Module " + modNumber + " Raw Angle";
        keyDeg = "Module " + modNumber + " Degrees";
    }

    private static double clamp(double x) {
        return (x < -1.0) ? -1.0 : (x > 1.0 ? 1.0 : x);
    }

    public void setDrivePower(double power) {

        targetVelocityTicksPerSec =
                power * Constants.DriveTrainConstants.MAX_MOTOR_RPM *
                        Constants.DriveTrainConstants.TICKS_PER_REV / 60.0;

        if (isModuleBackwards) {
            targetVelocityTicksPerSec = -targetVelocityTicksPerSec;
        }

        double currentVelocity = drive.getVelocity();
        double acceleration = (currentVelocity - lastVelocity) * 50;  // 1/0.02 = 50
        lastVelocity = currentVelocity;

        double ff =
                (kS * Math.signum(targetVelocityTicksPerSec)) +
                        (kV * targetVelocityTicksPerSec) +
                        (kA * acceleration);

        drive.setPower(clamp(ff));
    }

//    public void setDrivePower(double power) {
//
//        targetVelocityTicksPerSec =
//                power * Constants.DriveTrainConstants.MAX_MOTOR_RPM *
//                        Constants.DriveTrainConstants.TICKS_PER_REV / 60.0;
//
//        if (isModuleBackwards) {
//            targetVelocityTicksPerSec = -targetVelocityTicksPerSec;
//        }
//
//        double currentVelocity = drive.getVelocity();
//        double acceleration = (currentVelocity - lastVelocity) * 50;  // 1/0.02 = 50
//        lastVelocity = currentVelocity;
//
//        double ff =
//                (kS * Math.signum(targetVelocityTicksPerSec)) +
//                        (kV * targetVelocityTicksPerSec) +
//                        (kA * acceleration);
//
//        double output = clamp(ff);
//        double currentCurrentLMAO = getMotorCurrent();
//
//        if(Math.abs(currentCurrentLMAO) > 40) {
//            output *= (40 / currentCurrentLMAO);
//        }
//
//        drive.setPower(output);
//    }

    public double getVelocity() {
        return drive.getVelocity();
    }

    public double getVelocityTarget() {
        return targetVelocityTicksPerSec;
    }

    public double getVelocityError() {
        return Math.abs(drive.getVelocity() - targetVelocityTicksPerSec);
    }

    private double fastWrap360(double x) {
        x %= 360.0;
        return (x < 0) ? x + 360 : x;
    }

    private double fastWrap180(double x) {
        x = fastWrap360(x);
        return (x > 180) ? x - 360 : x;
    }

    public double getRawAngle() {
        return moduleHeading.getVoltage();
    }

    public void setTurnSpeed(double speed) {

        angle.setPower(speed * 0.5);

    }

    public double getDegrees(boolean applyOffset) {
        double rawDeg = (moduleHeading.getVoltage() / headingMaxVoltage) * 360.0;

        if (!applyOffset) return rawDeg;

        double foo = fastWrap360(360.0 - (rawDeg - moduleOffset));
        return foo;
    }

    public double getWrappedError(double setpoint, double measurement) {
        return fastWrap180(setpoint - measurement);
    }

    public void setModuleSetpoint(double setpoint) {

        double error = Math.abs(getWrappedError(setpoint, getDegrees(true)));

        if (error > 90.0) {
            moduleSetpoint = fastWrap360(setpoint + 180.0);
            isModuleBackwards = true;
        } else {
            moduleSetpoint = fastWrap360(setpoint);
            isModuleBackwards = false;
        }
    }

    public double getModuleSetpoint() {
        return moduleSetpoint;
    }

    public void setModulePosition() {

        double measurement = getDegrees(true);
        double output = angleController.calculate(measurement, moduleSetpoint);

        double ff = Math.signum(output) * 0.045;

        output += ff;

        angle.setPower(0.5 * clamp(output));  // half scaling kept
    }

    public boolean isWithinDegrees(double degrees) {
        return Math.abs(getWrappedError(moduleSetpoint, getDegrees(true))) < degrees;
    }

    public double getMotorCurrent() {
        return drive.getCurrent(CurrentUnit.AMPS);
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle() {

        if (!enableTelemetry) return;

        double raw = getDegrees(false);
        double deg = getDegrees(true);

        telem.putTelemetry(keyRaw, raw);
        telem.putTelemetry(keyDeg, deg);
        telem.putTelemetry("Module " + modNumber +" Voltage", moduleHeading.getVoltage());

        telem.putDashboard(keyRaw, raw);
        telem.putDashboard(keyDeg, deg);
    }

}
