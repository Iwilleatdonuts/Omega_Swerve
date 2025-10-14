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
    private double aprilBearing;
    private double previousBearing;

    TelemetryPacket packet = new TelemetryPacket();

    public TurretToApril(Turret s_Turret, AprilVision s_Vision, FtcDashboard dashboard){

        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;
        this.dashboard = dashboard;

        addRequirements(s_Turret);
    }

    @Override
    public void initialize(){

        aprilBearing = s_Vision.getAprilBearing();
        previousBearing = aprilBearing;

    }

    @Override
    public void execute(){

        s_Vision.periodic(dashboard);


        if(s_Vision.hasTag()){

            aprilBearing = s_Vision.getAprilBearing();
            double bearing = s_Turret.getDegrees() + aprilBearing;

            double filteredBearing = 0.8 * previousBearing + 0.2 * aprilBearing;
            previousBearing = bearing;

            s_Turret.setSetpoint(filteredBearing);
            s_Turret.runToSetpoint();

            packet.put("April tag bearing", aprilBearing);
            packet.put("turret Setpoint", filteredBearing);
            packet.put("turret position", s_Turret.getDegrees());
            packet.put("turret is out of boudns", s_Turret.isOutOfBounds());
        }

        dashboard.sendTelemetryPacket(packet);

    }

    @Override
    public void end(boolean interrupted) {
        s_Turret.setSpeed(0);
    }
}
