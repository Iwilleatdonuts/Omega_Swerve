package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class SwerveModule extends SubsystemBase {

    private final Telemetry telemetry;

    private final MotorEx drive;
    private final CRServo angle;
    private final AnalogInput feedback;

    private final PIDFController controller;

    private double continuousModulePosition;
    private double lastAngle;

    public SwerveModule(HardwareMap hardwareMap, Telemetry telemetry) {

        this.telemetry = telemetry;

        drive = hardwareMap.get(MotorEx.class, Constants.DriveTrainConstants.Mod0.driveMotor);
        angle = hardwareMap.get(CRServo.class, Constants.DriveTrainConstants.Mod0.angleServo);;
        feedback = hardwareMap.get(AnalogInput.class, Constants.DriveTrainConstants.Mod0.feedback);

        drive.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drive.setInverted(false);

        angle.setDirection(DcMotorSimple.Direction.FORWARD);

        controller = new PIDFController(Constants.DriveTrainConstants.kP, Constants.DriveTrainConstants.kI, Constants.DriveTrainConstants.kD, Constants.DriveTrainConstants.kF);

        continuousModulePosition = getRawAngle();

    }

    public void setDrivePower(double power) {

        drive.setRunMode(Motor.RunMode.RawPower);
        drive.set(power);

    }

    //take a value from -1 to 1
    public void setTurnSpeed(double speed) {

        angle.setPower(speed * 0.5);

    }

    public double getRawAngle() {
        return feedback.getVoltage();
    }

    public double getDegrees() {
        return (getRawAngle()/ feedback.getMaxVoltage())*360;
    }

    public double getContinuousModulePosition() {

        double maxVoltage = feedback.getMaxVoltage();
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
        telemetry.addData("Max Angle: ", feedback.getMaxVoltage());

    }

}
