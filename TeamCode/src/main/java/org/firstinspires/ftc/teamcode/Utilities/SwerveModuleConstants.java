package org.firstinspires.ftc.teamcode.Utilities;

public class SwerveModuleConstants {

    public final int modNumber;
    public final String driveMotor;
    public final String angleServo;
    public final String feedback;
    public final double moduleOffset;

    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;

    public SwerveModuleConstants(int modNumber, String driveMotor, String angleServo, String feedback, double moduleOffset, double kP, double kI, double kD, double kF){

        this.modNumber = modNumber;
        this.driveMotor = driveMotor;
        this.angleServo = angleServo;
        this.feedback = feedback;
        this.moduleOffset = moduleOffset;
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;

    }

}
