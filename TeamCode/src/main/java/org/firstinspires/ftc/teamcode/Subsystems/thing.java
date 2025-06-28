package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.DriveTrainConstants;

public class thing extends SubsystemBase {

    private final DcMotorEx motor0;

    private final CRServo servo0;

    private final AnalogInput moduleHeading0;

    public thing(HardwareMap hardwareMap) {
        motor0 = hardwareMap.get(DcMotorEx.class, DriveTrainConstants.Mod0.driveMotor);

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);

        motor0.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        servo0 = hardwareMap.get(CRServo.class, DriveTrainConstants.Mod0.angleServo);

        motor0.setDirection(DcMotorSimple.Direction.FORWARD);

        moduleHeading0 = hardwareMap.get(AnalogInput.class, DriveTrainConstants.Mod0.feedback);
    }

    public void setPower(double power){
        motor0.setPower(power);
    }

    public void setModuleSpeed(double speed){
        servo0.setPower(speed);
    }

    public double getModuleRotation(int module){
        double rotation = 0;
                rotation = moduleHeading0.getVoltage();

        return rotation;
    }

}