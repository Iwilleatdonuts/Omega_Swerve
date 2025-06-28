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

    private final Telemetry telemetry;

    private final DcMotorEx drive;
    private final CRServo angle;
    private final AnalogInput moduleHeading;

    private final PIDFController controller;

    private double continuousModulePosition;
    private double lastAngle;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.Mod0.driveMotor);
        angle = hardwareMap.get(CRServo.class, Constants.DriveTrainConstants.Mod0.angleServo);;

        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setDirection(DcMotorSimple.Direction.FORWARD);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(Constants.DriveTrainConstants.kP, Constants.DriveTrainConstants.kI, Constants.DriveTrainConstants.kD, Constants.DriveTrainConstants.kF);

        moduleHeading = hardwareMap.get(AnalogInput.class, Constants.DriveTrainConstants.Mod0.feedback);

        continuousModulePosition = getRawAngle();

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

    public double getContinuousModulePosition() {

        double maxVoltage = moduleHeading.getMaxVoltage();
        double currentAngle = getRawAngle();

        double positionCheck = currentAngle - lastAngle;

        if(positionCheck > maxVoltage/2) {
            positionCheck -= maxVoltage;
        }

        if(positionCheck < -maxVoltage/2) {
            positionCheck += maxVoltage;
        }

        lastAngle = currentAngle;
        continuousModulePosition += positionCheck;

        return continuousModulePosition;
    }

    public void update(){

        telemetry.addLine("Module 0");
        telemetry.addData("Angle: ", getRawAngle());
        telemetry.addData("Degrees: ", getDegrees());
        telemetry.addData("Continuous Angle: ", getContinuousModulePosition());
        telemetry.addData("Max Angle: ", moduleHeading.getMaxVoltage());

    }

}
