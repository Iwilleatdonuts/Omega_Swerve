package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;

public class SwerveModule extends SubsystemBase {

    private final DcMotorEx drive;
    private final CRServo angle;
    private final AnalogInput feedback;

    public SwerveModule(HardwareMap hardwareMap) {

        drive = hardwareMap.get(DcMotorEx.class, Constants.DriveTrainConstants.Mod0.driveMotor);
        angle = hardwareMap.get(CRServo.class, Constants.DriveTrainConstants.Mod0.angleServo);;
        feedback = hardwareMap.get(AnalogInput.class, Constants.DriveTrainConstants.Mod0.feedback);

    }

}
