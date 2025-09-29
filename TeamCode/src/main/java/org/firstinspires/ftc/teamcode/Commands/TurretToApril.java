package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class TurretToApril extends CommandBase {

    private final Turret s_Turret;
    private final AprilVision s_Vision;

    private double aprilX;
    private double aprilZ;
    private double cos, sin;

    public TurretToApril(Turret s_Turret, AprilVision s_Vision){

        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;

        addRequirements(s_Turret);
    }

    @Override
    public void initialize(){

        aprilX = s_Vision.getAprilX();
        aprilZ = s_Vision.getAprilZ();

        cos = Math.cos(Constants.VisionConstants.aimingCameraYaw);
        sin = Math.sin(Constants.VisionConstants.aimingCameraYaw);

        s_Vision.toggleTelemetry();

    }

    @Override
    public void execute(){

        s_Vision.periodic();


        if(s_Vision.hasTag()){

            aprilX = s_Vision.getAprilX();
            aprilZ = s_Vision.getAprilZ();

            double rotatedX = cos * aprilX - sin * aprilZ;
            double rotatedY = sin * aprilX + cos * aprilZ;

            double normalX =  rotatedX + Constants.VisionConstants.xOffsetFromTurret;
            double normalY = rotatedY + Constants.VisionConstants.yOffsetFromTurret;

            double bearing = Math.toDegrees(Math.atan2(normalX, normalY));

            double targetRotation = s_Turret.getDegrees() + bearing;
            s_Turret.setSetpoint(targetRotation);
            s_Turret.runToSetpoint();

        }



    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
