package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Swerve {

    private final Telemetry telemetry;

    private final SwerveModule mod0;
    private final SwerveModule mod1;
    private final SwerveModule mod2;
    private final SwerveModule mod3;

    private final OTOSSensor otos;

    public Swerve(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;

        mod0 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants);
        mod1 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants);
        mod2 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants);
        mod3 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants);

        otos = new OTOSSensor(hardwareMap, telemetry);

    }

    public void drive(double xVal, double yVal, double rVal, boolean fieldRelative){

        double x = xVal;
        double y = yVal;
        double r = rVal;

        if (Math.abs(x) < 0.1) {
            x = 0;
        }
        if (Math.abs(y) < 0.1) {
            y = 0;
        }
        if (Math.abs(r) < 0.1){
            r = 0;
        }

        if(fieldRelative){

            double robotHeading = Math.toRadians(otos.getHeading());

            x = x * Math.cos(robotHeading) + y * Math.sin(robotHeading);
            y = -x * Math.sin(robotHeading) + y * Math.cos(robotHeading);

        }

        double widthVector = r * Constants.DriveTrainConstants.widthRotation;
        double lengthVector = r * Constants.DriveTrainConstants.lengthRotation;

        double aVector = x - widthVector;
        double bVector = x + widthVector;
        double cVector = y - lengthVector;
        double dVector = y + lengthVector;

        double mod0Speed = Math.hypot(bVector, dVector);
        double mod1Speed = Math.hypot(bVector, cVector);
        double mod2Speed = Math.hypot(aVector, dVector);
        double mod3Speed = Math.hypot(aVector, cVector);

        double max = Math.max(Math.abs(mod0Speed), Math.abs(mod1Speed));
        max = Math.max(max, Math.abs(mod2Speed));
        max = Math.max(max, Math.abs(mod3Speed));

        if (max > 1.0) {
            mod0Speed /= max;
            mod1Speed /= max;
            mod2Speed /= max;
            mod3Speed /= max;
        }

        double mod0Angle = Math.toDegrees(Math.atan2(-bVector, dVector));
        double mod1Angle = Math.toDegrees(Math.atan2(-bVector, cVector));
        double mod2Angle = Math.toDegrees(Math.atan2(-aVector, dVector));
        double mod3Angle = Math.toDegrees(Math.atan2(-aVector, cVector));
        mod0Angle = (mod0Angle + 360) % 360;
        mod1Angle = (mod1Angle + 360) % 360;
        mod2Angle = (mod2Angle + 360) % 360;
        mod3Angle = (mod3Angle + 360) % 360;

        mod0.setDrivePower(mod0Speed);
        mod1.setDrivePower(mod1Speed);
        mod2.setDrivePower(mod2Speed);
        mod3.setDrivePower(mod3Speed);

        if(Math.abs(xVal) > 0.1 || Math.abs(yVal) > 0.1 || Math.abs(rVal) > 0.1){
            mod0.setModuleSetpoint(mod0Angle);
            mod1.setModuleSetpoint(mod1Angle);
            mod2.setModuleSetpoint(mod2Angle);
            mod3.setModuleSetpoint(mod3Angle);
        }

        mod0.setModulePosition();
        mod1.setModulePosition();
        mod2.setModulePosition();
        mod3.setModulePosition();

    }

    public void update(){

        telemetry.addLine("Swerve");
        telemetry.addData("X Position ", otos.getPose().x);
        telemetry.addData("Y Position ", otos.getPose().y);
        telemetry.addData("Heading ", otos.getHeading());
        telemetry.addData("OTOS Heading ", otos.getPose().h);
        telemetry.addLine();
    }

}
