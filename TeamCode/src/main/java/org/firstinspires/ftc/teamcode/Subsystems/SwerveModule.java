package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

    private final int modNumber;
    private final DcMotorEx drive;
    private final CRServoImplEx angle;
    private final AnalogInput moduleHeading;
    private final PIDFController controller;
    private final double moduleOffset;
    private final Telemetry telemetry;
    private double moduleSetpoint;

    private final double feedforward;

    //Idk if this is how ur supposed to make a swervy drive but I'm gonna
    // put a boolean to tell the module when it is backwards and so the
    // drive motor should be backwards because im rly smart definitely yes yes i can spell
    private boolean isModuleBackwards;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry, SwerveModuleConstants moduleConstants) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServoImplEx.class, moduleConstants.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20, 2, 0, 3));

        angle.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        controller = new PIDFController(Constants.DriveTrainConstants.angleKP, Constants.DriveTrainConstants.angleKI, Constants.DriveTrainConstants.angleKD, 0);

        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        moduleOffset = moduleConstants.moduleOffset;

        moduleSetpoint = getDegrees(true);

        modNumber = moduleConstants.modNumber;

        feedforward = moduleConstants.kF;

    }

    public void setDrivePower(double power) {

        double newPower = power;

            if(isModuleBackwards) {
                newPower = -newPower;
            }

        drive.setVelocity(newPower * 2800);

    }

    //take a value from -1 to 1
    public void setTurnSpeed(double speed) {

        angle.setPower(speed * 0.5);

    }

    public double getRawAngle() {
        return moduleHeading.getVoltage();
    }

    //withOffset set to true will return real angle, else will return raw angle
    public double getDegrees(boolean withOffset) {

        double rawAngle = (getRawAngle() / moduleHeading.getMaxVoltage()) * 360;

        double realAngle = rawAngle - moduleOffset;

        realAngle = (realAngle + 360) % 360;

        double realRealAngle = (360 - realAngle) % 360;

        return withOffset ? realRealAngle : rawAngle;
    }

    public double getWrappedError(double setpoint, double measurement) {
        double error = setpoint - measurement;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    public double getModuleSetpoint() {
        return moduleSetpoint;
    }

    public void setModuleSetpoint(double setpoint) {

        double newSetpoint = setpoint;

        double error = Math.abs(getWrappedError(newSetpoint, getDegrees(true)));

        if(error > 90){
            newSetpoint  = (newSetpoint + 180) % 360;
            isModuleBackwards = true;
        } else {
            isModuleBackwards = false;
        }

        moduleSetpoint = newSetpoint;
    }

    public void setModulePosition() {
        double error = getWrappedError(moduleSetpoint, getDegrees(true));

        double placeholder = moduleSetpoint - error;

        double servoOutput = controller.calculate(placeholder, moduleSetpoint);

        if(error < 0){
            servoOutput -= feedforward;
        }

        if(error > 0) {
            servoOutput += feedforward;
        }

        setTurnSpeed(servoOutput);
    }

    public double getMotorCurrent() {
        return drive.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isWithinDegrees(double degrees) {
        return Math.abs(getDegrees(true) - getModuleSetpoint()) < degrees;
    }

    public void update(boolean useTelemetry) {

        if(useTelemetry) {
            telemetry.addLine("Module " + modNumber);
            telemetry.addData("Degrees \t", getDegrees(true));
            telemetry.addData("Raw Angle \t", getDegrees(false));
            telemetry.addData("Setpoint \t", getModuleSetpoint());
            telemetry.addData("Drive speed \t", drive.getVelocity());
            telemetry.addData("Angular Error \t", getWrappedError(moduleSetpoint, getDegrees(true)));
            telemetry.addLine();
        }

    }

}
