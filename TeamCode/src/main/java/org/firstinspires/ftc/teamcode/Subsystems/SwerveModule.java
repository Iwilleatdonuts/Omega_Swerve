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

public class SwerveModule extends SubsystemBase {

    private Telemetry telem;

    private final DcMotorEx drive;
    private final CRServo angle;
    private final AnalogInput moduleHeading;

    private final PIDFController controller;


    private double moduleSetpoint;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telem) {

        this.telem = telem;

        drive = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.Mod0.driveMotor);
        angle = hardwareMap.get(CRServo.class, Constants.DriveTrainConstants.Mod0.angleServo);

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(Constants.DriveTrainConstants.angleKP, Constants.DriveTrainConstants.angleKI, Constants.DriveTrainConstants.angleKD, Constants.DriveTrainConstants.angleKF);


        moduleHeading = hardwareMap.get(AnalogInput.class, Constants.DriveTrainConstants.Mod0.feedback);

        moduleSetpoint = getDegrees();

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

    public double getDegrees() {
        return (getRawAngle()/ moduleHeading.getMaxVoltage())*360;
    }

    public double getWrappedError(double setpoint, double measurement) {
        double error = setpoint - measurement;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    public void setModuleSetpoint(double setpoint) {
        moduleSetpoint = setpoint;
    }

    public void setModulePosition() {
        double error = getWrappedError(moduleSetpoint, getDegrees());

        double placeholder = moduleSetpoint - error;

        telem.addData("idjsafjasdjfhsdah",error);

        setTurnSpeed(-controller.calculate(placeholder, moduleSetpoint));
    }

    public double getModuleSetpoint() {
        return moduleSetpoint;
    }

    public void update(){

        setModulePosition();

        telem.addLine("Module 0");
        telem.addData("Angle: ", getRawAngle());
        telem.addData("Degrees: ", getDegrees());
        telem.addData("Setpoint: ", getModuleSetpoint());
        telem.addData("Max Angle: ", moduleHeading.getMaxVoltage());

    }

}
