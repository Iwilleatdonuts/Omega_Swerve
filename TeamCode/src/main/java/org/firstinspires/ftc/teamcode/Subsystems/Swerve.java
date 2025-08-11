package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.HashMap;
import java.util.Map;

public class Swerve extends SubsystemBase {

    private final Telemetry telemetry;

    private final SwerveModule mod0, mod1, mod2, mod3;

    private final OTOSSensor otos;

    private boolean enableTelemetry;

    private final FtcDashboard dashboard;

    public Swerve(HardwareMap hardwareMap, Telemetry telemetry){

        this.telemetry = telemetry;

        mod0 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants);
        mod1 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants);
        mod2 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants);
        mod3 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants);

        otos = new OTOSSensor(hardwareMap, telemetry);

        enableTelemetry = false;

        dashboard = FtcDashboard.getInstance();

    }

    public void drive(double xVal, double yVal, double rVal, boolean fieldRelative, boolean slowMode){

        if (Math.abs(xVal) < 0.03) {xVal = 0;}
        if (Math.abs(yVal) < 0.03) {yVal = 0;}
        if (Math.abs(rVal) < 0.03) {rVal = 0;}

        double x = xVal;
        double y = yVal;

        if(fieldRelative){
            double robotHeading = Math.toRadians(otos.getHeading());
            double cosHeading = Math.cos(robotHeading);
            double sinHeading = Math.sin(robotHeading);

            x = xVal * cosHeading + yVal * sinHeading;
            y = -xVal * sinHeading + yVal * cosHeading;
        }

        double r = rVal;

        double rotVecX = r * (Constants.DriveTrainConstants.trackWidth / Constants.DriveTrainConstants.moduleHypotenuse);
        double rotVecY = r * (Constants.DriveTrainConstants.wheelbase / Constants.DriveTrainConstants.moduleHypotenuse);

        double aVec = x - rotVecX;
        double bVec = x + rotVecX;
        double cVec = y - rotVecY;
        double dVec = y + rotVecY;

        double mod0Speed = Math.hypot(bVec, dVec);
        double mod1Speed = Math.hypot(bVec, cVec);
        double mod2Speed = Math.hypot(aVec, dVec);
        double mod3Speed = Math.hypot(aVec, cVec);

        double max = Math.max(Math.max(mod0Speed, mod1Speed), Math.max(mod2Speed, mod3Speed));

        if (max > 1.0) {
            double optimized = 1.0 / max;  // division is supposedly slower than multiplication, gonna optimize here
            mod0Speed *= optimized;
            mod1Speed *= optimized;
            mod2Speed *= optimized;
            mod3Speed *= optimized;
        }

        if(slowMode) {
            mod0Speed *= 0.3;
            mod1Speed *= 0.3;
            mod2Speed *= 0.3;
            mod3Speed *= 0.3;
        }

        double mod0Angle = normalizeAngle(Math.toDegrees(Math.atan2(-bVec, dVec)));
        double mod1Angle = normalizeAngle(Math.toDegrees(Math.atan2(-bVec, cVec)));
        double mod2Angle = normalizeAngle(Math.toDegrees(Math.atan2(-aVec, dVec)));
        double mod3Angle = normalizeAngle(Math.toDegrees(Math.atan2(-aVec, cVec)));

        mod0.setDrivePower(mod0Speed);
        mod1.setDrivePower(mod1Speed);
        mod2.setDrivePower(mod2Speed);
        mod3.setDrivePower(mod3Speed);

        if (xVal != 0 || yVal != 0 || rVal != 0) {
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

    public Map<String, Object> getMotorCurrents(){
        Map<String, Object> motorCurrents = new HashMap<>();
        motorCurrents.put("Mod 0 Current: \t", mod0.getMotorCurrent());
        motorCurrents.put("Mod 1 Current: \t", mod1.getMotorCurrent());
        motorCurrents.put("Mod 2 Current: \t", mod2.getMotorCurrent());
        motorCurrents.put("Mod 3 Current: \t", mod3.getMotorCurrent());
        return motorCurrents;
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    @Override
    public void periodic(){

        TelemetryPacket packet = new TelemetryPacket();
        packet.putAll(getMotorCurrents());

        dashboard.sendTelemetryPacket(packet);

        if(enableTelemetry) {
            telemetry.addLine("Swerve");
            telemetry.addData("X Position ", otos.getPose().x);
            telemetry.addData("Y Position ", otos.getPose().y);
            telemetry.addData("Heading ", otos.getHeading());
            telemetry.addData("OTOS Heading ", otos.getPose().h);
            telemetry.addLine();
        }
    }
}

