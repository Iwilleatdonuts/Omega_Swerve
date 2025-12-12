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

    private final boolean areWeWinners;

    private final double closeCloseSkew;
    private final double closeSkew;
    private final double mediumCloseSkew;
    private final double mediumFarSkew;
    private final double farSkew;

    private final double farManualSetpoint;
    private final double mediumManualSetpoint;
    private final double closeManualSetpoint;

    private double heading;

    private double totalSkew;

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
        gatePose = new OmegaPose2D(0, 1.42377413355856, 0);
        farZonePose = areWeWinners ? new OmegaPose2D(1.4478, 0, 0) : new OmegaPose2D(-1.4478, 0, 0);

        closeCloseSkew = areWeWinners ? 1 : -1;

        farSkew = areWeWinners ? -4 : 4;
        mediumFarSkew = areWeWinners ? -3.5 : 3.5;
        mediumCloseSkew = areWeWinners ? -3.2 : 3.2;
        closeSkew = areWeWinners ? -2 : 2;

        farManualSetpoint = areWeWinners? 57.26 : -57.26;
        mediumManualSetpoint = areWeWinners? 41.63 : -41.63;
        closeManualSetpoint = areWeWinners ? 37.75 : -37.75;
        totalSkew = 0;
    }

    public void initialize(){

        aprilBearing = s_Lime.getFilteredBearing();
        timestamp = System.nanoTime();

    }

    public void execute(){

        if(m_Driver.wasJustPressed(GamepadKeys.Button.Y)) {
            s_Lemon.setLinearPose(new OmegaPose2D(0, 0, 0));
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.START)) {
            s_Lemon.setLinearPose(gatePose);
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.BACK)) {
            s_Lemon.setLinearPose(farZonePose);
        }

        //MANUAL CLOSE SHOT
        if (m_Operator.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {

            double heading = closeManualSetpoint - s_Lemon.getHeading();

            heading = wrapTo180(heading);
            this.heading = heading;

            //MANUAL MEDOIUM SHOT
        } else if(m_Operator.isDown(GamepadKeys.Button.LEFT_BUMPER)) {

            double heading = mediumManualSetpoint - s_Lemon.getHeading();

            heading = wrapTo180(heading);
            this.heading = heading;

            //MANUAL FAR SHOT
        } else if(m_Operator.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.9) {

            double heading = farManualSetpoint - s_Lemon.getHeading();

            heading = wrapTo180(heading);
            this.heading = heading;

        } else if (s_Lime.isValidReaing()) {

            totalSkew = 0;

            double turretOnField = wrapTo360(s_Lemon.getHeading() + s_Turret.getDegrees());

            double theta = wrapTo180(turretOnField - s_Lime.getFilteredBearing());

            if((areWeWinners && theta > 45) || (!areWeWinners && theta < -45)) {
                double foo = 0.933308 * theta + (areWeWinners ? 0.0292659 : -0.0292659);
                totalSkew = foo - theta;
            }

            aprilBearing = s_Lime.getFilteredBearing();

            double bearing = s_Turret.getDegrees() - aprilBearing;

            if((s_Lemon.getCurrentPose().x() > 1.3 && !areWeWinners) || (s_Lemon.getCurrentPose().x() < -1.3 && areWeWinners)) {
                bearing += closeCloseSkew;
            }

            heading = bearing;

            timestamp = System.nanoTime();

        } else if (Math.hypot(m_Operator.getLeftX(), m_Operator.getLeftY()) > 0.9){

            double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_Operator.getLeftX(), m_Operator.getLeftY()));
            operatorJoystickAngle = wrapTo360(operatorJoystickAngle);

            operatorJoystickAngle -= s_Swerve.getHeading();

            operatorJoystickAngle = wrapTo360(operatorJoystickAngle);

            heading = operatorJoystickAngle;

        } else if(System.nanoTime() - timestamp > 0.5e9) {

            OmegaPose2D currentPose = s_Lemon.getCurrentPose();
            //gtes x and Y value on field

            double theta = Math.toDegrees(Math.atan2(-(targetPose.x() - currentPose.x()), targetPose.y() - currentPose.y()));
            double turretHeading = theta - s_Lemon.getHeading();

            turretHeading = wrapTo180(turretHeading);

            heading = turretHeading;
        }

        if(m_Driver.wasJustPressed(GamepadKeys.Button.A) && !s_Lime.isValidReaing()) {
            timestamp = System.nanoTime() - 1e9;
        }

//        double skewCounter = 0;
//        double totalSkew = 0;
//
//        if(m_Operator.isDown(GamepadKeys.Button.Y)) {
//            totalSkew += farSkew;
//            skewCounter++;
//        }
//
//        if(m_Operator.isDown(GamepadKeys.Button.X)) {
//            totalSkew += areWeWinners ? mediumCloseSkew : mediumFarSkew;
//            heading++;
//        }
//
//        if(m_Operator.isDown(GamepadKeys.Button.B)) {
//            totalSkew += areWeWinners ? mediumFarSkew : mediumCloseSkew;
//            heading++;
//        }
//
//        if(m_Operator.isDown(GamepadKeys.Button.A)) {
//            totalSkew += closeSkew;
//            heading++;
//        }
//
//        if(skewCounter != 0) {
//            totalSkew/=skewCounter;
//        }

        s_Turret.setSetpoint(heading + totalSkew);
        s_Turret.runToSetpoint();

        telem.putLine("Turret");
        telem.putTelemetry("Skew", totalSkew);
        telem.putLine();

    }

    public void end() {
        s_Turret.setSetpoint(s_Turret.getDegrees());
        s_Turret.setSpeed(0);
    }

    public static double wrapTo360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    public static double wrapTo180(double deg) {
        deg = ((deg + 180.0) % 360.0);
        if (deg <= 0) deg += 360.0;
        return deg - 180.0;
    }
}
