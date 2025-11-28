package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;

public class LimeTurret {

    private final Swerve s_Swerve;
    private final Turret s_Turret;
    private final Limelight s_Lime;
    private final OTOSSensor s_Sparky;
    private double aprilBearing;

    private final OmegaController m_Operator;
    private final OmegaController m_Driver;
    private final OmegaPose2D targetPose;

    private final EZTelemetry telem;
    private final boolean areWeWinners;
    private final double farSkew;
    private final double closeSkew;

    public LimeTurret(Swerve s_Swerve, Turret s_Turret, Limelight s_Lime, OTOSSensor s_Sparky, OmegaController m_Operator, OmegaController m_Driver, EZTelemetry telem, boolean areWeWinners){

        this.s_Swerve = s_Swerve;
        this.s_Turret = s_Turret;
        this.s_Lime = s_Lime;
        this.s_Sparky = s_Sparky;

        this.m_Operator = m_Operator;
        this.m_Driver = m_Driver;
        this.telem = telem;

        this.areWeWinners = areWeWinners;

        targetPose = areWeWinners ? Constants.ShooterConstants.redTarget : Constants.ShooterConstants.blueTarget;
        farSkew = areWeWinners ? 2 : -2;
        closeSkew = areWeWinners ? -3 : 3;

    }

    public void initialize(){

        aprilBearing = s_Lime.getFilteredBearing();

    }

    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.Y) || m_Driver.wasJustPressed(GamepadKeys.Button.Y)) {
            s_Sparky.setNewLinearPose(new OmegaPose2D(0, 0, 0));
        }

        if(s_Lime.isValidReaing()){

            aprilBearing = s_Lime.getFilteredBearing();
            double bearing = s_Turret.getDegrees() - aprilBearing;

            if((s_Sparky.getPose().x() < -1.3 && !areWeWinners) || (s_Sparky.getPose().x() > 1.3 && areWeWinners)) {
                bearing -= farSkew;
            }

            if((s_Sparky.getPose().x() > 1.3 && !areWeWinners) || (s_Sparky.getPose().x() < -1.3 && areWeWinners)) {
                bearing -= closeSkew;
            }

            s_Turret.setSetpoint(bearing);

        } else if (Math.hypot(m_Operator.getLeftX(), m_Operator.getLeftY()) > 0.9){

            double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_Operator.getLeftX(), m_Operator.getLeftY()));
            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            operatorJoystickAngle -= s_Swerve.getHeading();

            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

                s_Turret.setSetpoint(operatorJoystickAngle);

        } else {

            if(!m_Driver.isDown(GamepadKeys.Button.A)){

                OmegaPose2D currentPose = s_Sparky.getPose();

                double theta = Math.toDegrees(Math.atan2(-(targetPose.x() - currentPose.x()), targetPose.y() - currentPose.y()));
                double turretHeading = theta - s_Sparky.getHeading();

                if(turretHeading < -180) {
                    turretHeading += 360;
                }

                if(turretHeading > 180) {
                    turretHeading -= 360;
                }

                s_Turret.setSetpoint(turretHeading);

//                telem.putTelemetry("Theta", theta);
//                telem.putTelemetry("X", currentPose.x());
//                telem.putTelemetry("Y", currentPose.y());
            }

        }

        s_Turret.runToSetpoint();
    }

    public void end() {
        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Turret.setSpeed(0);
    }
}
