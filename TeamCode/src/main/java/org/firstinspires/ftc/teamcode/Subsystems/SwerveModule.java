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
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.SwerveModuleConstants;

public class SwerveModule extends SubsystemBase {

    private Telemetry telemetry;

    private final int modNumber;

    private final DcMotorEx drive;
    private final CRServo angle;
    private final AnalogInput moduleHeading;

    private final PIDFController controller;

    private final double moduleOffset;
    private double moduleSetpoint;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry, SwerveModuleConstants moduleConstants) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(DcMotorEx.class, moduleConstants.driveMotor);
        angle = hardwareMap.get(CRServo.class, moduleConstants.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(Constants.DriveTrainConstants.angleKP, Constants.DriveTrainConstants.angleKI, Constants.DriveTrainConstants.angleKD, Constants.DriveTrainConstants.angleKF);


        moduleHeading = hardwareMap.get(AnalogInput.class, moduleConstants.feedback);
        moduleOffset = moduleConstants.moduleOffset;

        moduleSetpoint = getDegrees(true);

        modNumber = moduleConstants.modNumber;

    }

    public void setDrivePower(double power) {

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

        double rawAngle = (getRawAngle()/ moduleHeading.getMaxVoltage())*360;

        double realAngle = rawAngle - moduleOffset;

        if(realAngle < 0){
            realAngle +=360;
        }

        return withOffset ? realAngle : rawAngle;
    }

    public double getWrappedError(double setpoint, double measurement) {
        double error = setpoint - measurement;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    public void setModuleSetpoint(double setpoint) {
        moduleSetpoint = setpoint;
    }

    public double getModuleSetpoint() {
        return moduleSetpoint;
    }

    public void setModulePosition() {
        double error = getWrappedError(moduleSetpoint, getDegrees(true));

        double placeholder = moduleSetpoint - error;

        telemetry.addData("idjsafjasdjfhsdah",error);

        setTurnSpeed(-controller.calculate(placeholder, moduleSetpoint));
    }

    public void update(){

        setModulePosition();

        telemetry.addLine("Module " + modNumber);
        telemetry.addData("Raw Angle: ", getRawAngle());
        telemetry.addData("Degrees: ", getDegrees(true));
        telemetry.addData("Setpoint: ", getModuleSetpoint());
        telemetry.addData("Max Angle: ", moduleHeading.getMaxVoltage());

    }

}
