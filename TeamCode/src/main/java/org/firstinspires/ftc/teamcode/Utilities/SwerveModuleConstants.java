package org.firstinspires.ftc.teamcode.Utilities;

public class SwerveModuleConstants {

    public final int modNumber;
    public final String driveMotor;
    public final String angleServo;
    public final String feedback;
    public final double moduleOffset;

    public SwerveModuleConstants(int modNumber, String driveMotor, String angleServo, String feedback, double moduleOffset){

        this.modNumber = modNumber;
        this.driveMotor = driveMotor;
        this.angleServo = angleServo;
        this.feedback = feedback;
        this.moduleOffset = moduleOffset;

    }

}
