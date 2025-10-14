package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class TurretToApril extends CommandBase {

    private final Turret s_Turret;
    private final AprilVision s_Vision;
    private final FtcDashboard dashboard;

    private double aprilX;
    private double aprilY;
    private double aprilBearing;

    private double cos, sin;


    TelemetryPacket packet = new TelemetryPacket();

    public TurretToApril(Turret s_Turret, AprilVision s_Vision, FtcDashboard dashboard){

        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;
        this.dashboard = dashboard;

        addRequirements(s_Turret);
    }

    @Override
    public void initialize(){

        aprilX = s_Vision.getAprilX();
        aprilY = s_Vision.getAprilZ();

        cos = Math.cos(Constants.VisionConstants.aimingCameraYaw);
        sin = Math.sin(Constants.VisionConstants.aimingCameraYaw);


    }

    @Override
    public void execute(){

        s_Vision.periodic(dashboard);


        if(s_Vision.hasTag()){

//            aprilX = s_Vision.getAprilX();
//            aprilY = s_Vision.getAprilY();

            aprilBearing = s_Vision.getAprilBearing();
//
//            double rotatedX = cos * aprilX - sin * aprilY;
//            double rotatedY = sin * aprilX + cos * aprilY;
//
//            double normalX =  rotatedX + Constants.VisionConstants.xOffsetFromTurret;
//            double normalY = rotatedY + Constants.VisionConstants.yOffsetFromTurret;
//
//            double bearing = Math.toDegrees(Math.atan2(-normalX, normalY));
//            bearing = (bearing+360)%360;
//
            double bearing = s_Turret.getDegrees() + aprilBearing;

            s_Turret.setSetpoint(bearing);
//            s_Turret.runToSetpoint();

            packet.put("April tag bearing", aprilBearing);
            packet.put("turret Setpoint", bearing);
            packet.put("turret position", s_Turret.getDegrees());
            packet.put("turret is out of boudns", s_Turret.isOutOfBounds());
        }

        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void end(boolean interrupted) {
        s_Turret.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
