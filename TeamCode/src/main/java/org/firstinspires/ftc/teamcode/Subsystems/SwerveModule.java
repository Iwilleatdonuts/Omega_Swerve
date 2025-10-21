package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
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
import org.firstinspires.ftc.teamcode.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;

public class SwerveModule extends SubsystemBase {

    private final int modNumber;
    private final DcMotorEx drive;
    private final CRServoImplEx angle;
    private final AnalogInput moduleHeading;
    private final PIDController angleController;
    private final double moduleOffset;
    private final Telemetry telemetry;
    private double moduleSetpoint;

    private final double velocityFeedforward = 0.1;

    //Idk if this is how ur supposed to make a swervy drive but I'm gonna
    // put a boolean to tell the module when it is backwards and so the
    // drive motor should be backwards because im rly smart definitely yes yes i can spell
    private boolean isModuleBackwards;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    private double lastVelocity = 0.0;

    private double targetVelocityTicksPerSec;

    private final double kS = 0.05;      // static friction term
    private final double kV = 1/Constants.DriveTrainConstants.MAX_TICKS_PER_SEC;    //ticks per second
    private final double kA = 0.0;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry, SwerveModuleConstants moduleConstants) {

        this.telemetry = telemetry;

        modNumber = moduleConstants.modNumber;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServoImplEx.class, moduleConstants.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);
        angle.setPwmRange(new PwmControl.PwmRange(500, 2500));

        angleController = new PIDController(Constants.DriveTrainConstants.angleKP, Constants.DriveTrainConstants.angleKI, Constants.DriveTrainConstants.angleKD);
//        angleController = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kD);
        angleController.enableContinuousInput(0, 360);
        angleController.setIZone(30);

        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        moduleOffset = moduleConstants.moduleOffset;

        moduleSetpoint = getDegrees(true);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

    }

    public void setDrivePower(double power) {

        double targetMotorRpm = Constants.DriveTrainConstants.MAX_MOTOR_RPM * power;
        targetVelocityTicksPerSec = (targetMotorRpm * Constants.DriveTrainConstants.TICKS_PER_REV) / 60.0;

        if (isModuleBackwards) targetVelocityTicksPerSec *= -1;

        double currentVelocity = drive.getVelocity();

        double acceleration = (currentVelocity - lastVelocity) / 0.02; // ~20ms loop
        lastVelocity = currentVelocity;

        double ff = kS * Math.signum(targetVelocityTicksPerSec)
                + kV * targetVelocityTicksPerSec
                + kA * acceleration;

        ff = Math.max(-1.0, Math.min(1.0, ff));

        drive.setPower(ff);
    }

    public double getVelocity() {
        return drive.getVelocity();
    }

    public double getVelocityError() {
        return Math.abs(drive.getVelocity() - getVelocityTarget());
    }

    public double getVelocityTarget() {
        return targetVelocityTicksPerSec;
    }

    //take a value from -1 to 1
    public void setTurnSpeed(double speed) {

        angle.setPower(speed * 0.5);

    }

    public double getRawAngle() {
        return moduleHeading.getVoltage();
    }
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
        double servoOutput = angleController.calculate(getDegrees(true), getModuleSetpoint());
        servoOutput = Math.max(-1, Math.min(1, servoOutput));
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
            telemetry.addData("Raw Angle \t", getDegrees(false));
            telemetry.addData("Degrees \t", getDegrees(true));
            telemetry.addData("Angular Error \t", getWrappedError(moduleSetpoint, getDegrees(true)));
            telemetry.addData("Drive speed \t", drive.getVelocity());
            telemetry.addData("Velocity Error\t", getVelocityError());
            telemetry.addLine();
        }

    }

}
