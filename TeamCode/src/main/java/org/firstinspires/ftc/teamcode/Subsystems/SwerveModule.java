package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private final PIDController angleController;
    private final double moduleOffset;
    private final Telemetry telemetry;
    private double moduleSetpoint;

    private final double angularFeedforward;
    private final double velocityFeedforward = 0.1;

    //Idk if this is how ur supposed to make a swervy drive but I'm gonna
    // put a boolean to tell the module when it is backwards and so the
    // drive motor should be backwards because im rly smart definitely yes yes i can spell
    private boolean isModuleBackwards;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry, SwerveModuleConstants moduleConstants) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServoImplEx.class, moduleConstants.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        angleController = new PIDController(Constants.DriveTrainConstants.angleKP, Constants.DriveTrainConstants.angleKI, Constants.DriveTrainConstants.angleKD);

        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        moduleOffset = moduleConstants.moduleOffset;

        moduleSetpoint = getDegrees(true);

        modNumber = moduleConstants.modNumber;

        angularFeedforward = moduleConstants.kF;

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

    }

    public void setDrivePower(double power) {

//        double velocityTarget = power;
//
//        if (isModuleBackwards) {
//            velocityTarget = -velocityTarget;
//        }
//
//        double currentVelocity = drive.getVelocity()/2800;
//
//        double feedforward = velocityFeedforward * Math.signum(velocityTarget);
//
//        double output = driveController.calculate(currentVelocity, velocityTarget) + feedforward;
//        output = Math.max(-1.0, Math.min(1.0, output));
//
//        drive.setPower(output);
        if (isModuleBackwards) {
            power = -power;
        }
        drive.setPower(power);
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

        double servoOutput = angleController.calculate(placeholder, moduleSetpoint);

        if(error < 0){
            servoOutput -= angularFeedforward;
        }

        if(error > 0) {
            servoOutput += angularFeedforward;
        }

        servoOutput = Math.max(-1.0, Math.min(1.0, servoOutput));
        setTurnSpeed(servoOutput);
    }

    public double getMotorCurrent() {
        return drive.getCurrent(CurrentUnit.AMPS);
    }

    public boolean isWithinDegrees(double degrees) {
        return Math.abs(getDegrees(true) - getModuleSetpoint()) < degrees;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    @Override
    public void periodic(){

        if(enableTelemetry) {
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
