package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class TurretToApril extends CommandBase {

    private final Swerve s_Swerve;
    private final Turret s_Turret;
    private final AprilVision s_Vision;
    private final FtcDashboard dashboard;
    private double aprilBearing;
    private double aprilX;
    private double aprilY;

    TelemetryPacket packet = new TelemetryPacket();

    public TurretToApril(Swerve s_Swerve, Turret s_Turret, AprilVision s_Vision, FtcDashboard dashboard){

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;
        this.dashboard = dashboard;

        addRequirements(s_Turret);
    }

    @Override
    public void initialize(){

        aprilBearing = s_Vision.getGoalBearing();
        aprilX = s_Vision.getGoalX();
        aprilY = s_Vision.getGoalY();

    }

    @Override
    public void execute(){

        if(s_Vision.hasTag()){

            aprilBearing = s_Vision.getGoalBearing();
            double bearing = s_Turret.getDegrees() + aprilBearing;

            s_Turret.setSetpoint(bearing);

        }
        s_Turret.runToSetpoint();

        packet.put("April tag bearing", aprilBearing);
        packet.put("actual turret setpoint", s_Turret.getSetpoint());
        packet.put("turret position", s_Turret.getDegrees());
        packet.put("turret is out of bounds", s_Turret.isOutOfBounds());
        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void end(boolean interrupted) {
        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Turret.setSpeed(0);
    }
}
