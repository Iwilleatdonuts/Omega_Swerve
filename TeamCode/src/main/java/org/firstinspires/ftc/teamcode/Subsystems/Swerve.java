package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;

public class Swerve extends SubsystemBase {

    private final DcMotorEx motor0;
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final DcMotorEx motor3;

    private final CRServo servo0;
    private final CRServo servo1;
    private final CRServo servo2;
    private final CRServo servo3;

    private final AnalogInput moduleHeading0;
    private final AnalogInput moduleHeading1;
    private final AnalogInput moduleHeading2;
    private final AnalogInput moduleHeading3;

    public Swerve(HardwareMap hardwareMap) {
        motor0 = hardwareMap.get(DcMotorEx.class, DriveTrainConstants.Mod0.driveMotor);
        motor1 = hardwareMap.get(DcMotorEx.class, DriveTrainConstants.Mod1.driveMotor);
        motor2 = hardwareMap.get(DcMotorEx.class, DriveTrainConstants.Mod2.driveMotor);
        motor3 = hardwareMap.get(DcMotorEx.class, DriveTrainConstants.Mod3.driveMotor);

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo0 = hardwareMap.get(CRServo.class, DriveTrainConstants.Mod0.angleServo);
        servo1 = hardwareMap.get(CRServo.class, DriveTrainConstants.Mod1.angleServo);
        servo2 = hardwareMap.get(CRServo.class, DriveTrainConstants.Mod2.angleServo);
        servo3 = hardwareMap.get(CRServo.class, DriveTrainConstants.Mod3.angleServo);

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);
        motor3.setDirection(DcMotorSimple.Direction.FORWARD);

        moduleHeading0 = hardwareMap.get(AnalogInput.class, DriveTrainConstants.Mod0.feedback);
        moduleHeading1 = hardwareMap.get(AnalogInput.class, DriveTrainConstants.Mod1.feedback);
        moduleHeading2 = hardwareMap.get(AnalogInput.class, DriveTrainConstants.Mod2.feedback);
        moduleHeading3 = hardwareMap.get(AnalogInput.class, DriveTrainConstants.Mod3.feedback);
    }

    public void setPower(int module, double power){
        switch(module){
            case 0:
                motor0.setPower(power);
                break;
            case 1:
                motor1.setPower(power);
                break;
            case 2:
                motor2.setPower(power);
                break;
            case 3:
                motor3.setPower(power);
                break;
        }
    }

    public void setModuleSpeed(int module, double speed){
        switch(module){
            case 0:
                servo0.setPower(speed);
                break;
            case 1:
                servo1.setPower(speed);
                break;
            case 2:
                servo2.setPower(speed);
                break;
            case 3:
                servo3.setPower(speed);
                break;
        }
    }

    public double getModuleRotation(int module){
        double rotation = 0;
        switch(module){
            case 0:
                rotation = moduleHeading0.getVoltage();
                break;
            case 1:
                rotation = moduleHeading1.getVoltage();
                break;
            case 2:
                rotation = moduleHeading2.getVoltage();
                break;
            case 3:
                rotation = moduleHeading3.getVoltage();
                break;
        }

        return rotation;
    }

}