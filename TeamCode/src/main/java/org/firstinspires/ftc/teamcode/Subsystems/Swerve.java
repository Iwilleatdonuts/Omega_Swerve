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

    public Swerve(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;

        mod0 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants);
        mod1 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants);
        mod2 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants);
        mod3 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants);

    }

    public void drive(double xJoy, double yJoy, double rJoy){

    }

    public double getAngleFromJoystick(double x, double y){
        double angle = Math.toDegrees(Math.atan2(-y, x));
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void update(){

        telemetry.addLine("Swerve");
        telemetry.addLine();
    }

}
