package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.FusionOdometry;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class LimeTurret {

    private final Swerve s_Swerve;
    private final Turret s_Turret;
    private final Limelight s_Lime;
    private final FusionOdometry s_Lemon;
    private double aprilBearing;

    private final OmegaController m_Operator;
    private final OmegaController m_Driver;
    private final OmegaPose2D targetPose;

    private final EZTelemetry telem;
    private final OmegaPose2D gatePose;
    private final OmegaPose2D farZonePose;

    private double timestamp;

    private boolean areWeWinners;

    private double closeSkew;
    private double farSkew;

    public LimeTurret(Swerve s_Swerve, Turret s_Turret, Limelight s_Lime, FusionOdometry s_Lemon, OmegaController m_Operator, OmegaController m_Driver, EZTelemetry telem, boolean areWeWinners){

        this.areWeWinners = areWeWinners;

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;
        this.s_Lemon = s_Lemon;

        this.m_Operator = m_Operator;
        this.m_Driver = m_Driver;
        this.telem = telem;

        targetPose = areWeWinners ? Constants.TurretConstants.redTarget : Constants.TurretConstants.blueTarget;
        gatePose = new OmegaPose2D(0, 1.39, 0);
        farZonePose = areWeWinners ? new OmegaPose2D(1.36, 0, 0) : new OmegaPose2D(-1.36, 0, 0);

        closeSkew = areWeWinners ? 1 : -1;
        farSkew = areWeWinners ? -2 : 2;
    }

    public void initialize(){

        aprilBearing = s_Lime.getFilteredBearing();
        timestamp = System.nanoTime();

    }

    public void execute(){

        if(m_Driver.wasJustPressed(GamepadKeys.Button.Y)) {
            s_Lemon.setLinearPose(new OmegaPose2D(0, 0, 0));
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.Y)) {
            s_Lemon.setLinearPose(gatePose);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.X)) {
            s_Lemon.setLinearPose(farZonePose);
        }

        if (s_Lime.isValidReaing()) {

            aprilBearing = s_Lime.getFilteredBearing();
            double bearing = s_Turret.getDegrees() - aprilBearing;

            if((s_Lemon.getCurrentPose().x() < -0.4 && !areWeWinners) || (s_Lemon.getCurrentPose().x() > 0.4 && areWeWinners)) {
                bearing += farSkew;
            }

            if((s_Lemon.getCurrentPose().x() > 1.3 && !areWeWinners) || (s_Lemon.getCurrentPose().x() < -1.3 && areWeWinners)) {
                bearing += closeSkew;
            }

            s_Turret.setSetpoint(bearing);

            timestamp = System.nanoTime();

        } else if (Math.hypot(m_Operator.getLeftX(), m_Operator.getLeftY()) > 0.9){

            double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_Operator.getLeftX(), m_Operator.getLeftY()));
            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            operatorJoystickAngle -= s_Swerve.getHeading();

            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            s_Turret.setSetpoint(operatorJoystickAngle);

        } else if(System.nanoTime() - timestamp > 0.8e9) {

            OmegaPose2D currentPose = s_Lemon.getCurrentPose();
            //gtes x and Y value on field

            double theta = Math.toDegrees(Math.atan2(-(targetPose.x() - currentPose.x()), targetPose.y() - currentPose.y()));
            double turretHeading = theta - s_Lemon.getHeading();

            if(turretHeading < -180) {
                turretHeading += 360;
            }

            if(turretHeading > 180) {
                turretHeading -= 360;
            }

            s_Turret.setSetpoint(turretHeading);
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.A) && !s_Lime.isValidReaing()) {
            timestamp = System.nanoTime() - 1e9;
        }

        s_Turret.runToSetpoint();
    }

    public void end() {
        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Turret.setSpeed(0);
    }
}
