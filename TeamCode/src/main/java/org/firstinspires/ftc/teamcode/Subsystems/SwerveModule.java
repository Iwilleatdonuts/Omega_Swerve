package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

    private final int modNumber;
    private final DcMotorEx drive;
    private final CRServo angle;
    private final AnalogInput moduleHeading;
    private final PIDFController controller;
    private final double moduleOffset;
    private final Telemetry telemetry;
    private double moduleSetpoint;

    //Idk if this is how ur supposed to make a swervy drive but I'm gonna
    // put a boolean to tell the module when it is backwards and so the
    // drivy motor should be backwards because im rly smart definetely yes yes i can speel
    private boolean isModuleBackwards;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry, SwerveModuleConstants moduleConstants) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServo.class, moduleConstants.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setVelocityPIDFCoefficients(1, 0, 0, 0.00036);

        angle.setDirection(DcMotorSimple.Direction.REVERSE);

        controller = new PIDFController(moduleConstants.kP, moduleConstants.kI, moduleConstants.kD, moduleConstants.kF);


        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        moduleOffset = moduleConstants.moduleOffset;

        moduleSetpoint = getDegrees(true);

        modNumber = moduleConstants.modNumber;

    }

    public void setDrivePower(double power) {

        double newPower = power;

        if(isModuleBackwards) {
            newPower = -newPower;
        }

//        drive.setPower(newPower);

        drive.setVelocity(newPower*2800);

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

        if (realAngle < 0) {
            realAngle += 360;
        }

        return withOffset ? realAngle : rawAngle;
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

        setTurnSpeed(-controller.calculate(placeholder, moduleSetpoint));
    }

    public void update() {

        telemetry.addLine("Module " + modNumber);
        telemetry.addData("Degrees \t", getDegrees(true));
        telemetry.addData("Setpoint \t", getModuleSetpoint());
        telemetry.addData("Drive speed \t", drive.getVelocity());
        telemetry.addLine();

    }

}
