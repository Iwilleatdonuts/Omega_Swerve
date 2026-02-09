package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class TurretCalculator {

    private final FusionOdometry odom;
    private final Shooter shooter;
    private final EZTelemetry telem;

    private Rotation2d lastTurretAngle;
    private double lastHoodAngle;

    private Pose2d robotPose;
    private Pose2d turretPose;
    private Vector2d robotVelocity;
    private double omega;

    private double distanceToTarget;
    private double timeOfFlight;

    private final Pose2d robotToTurret = new Pose2d(new Translation2d(0, -0.025), new Rotation2d(0));

    private final Translation2d goalPose;

    public TurretCalculator(FusionOdometry odom, Shooter shooter, EZTelemetry telem, Translation2d goalPose) {
        this.odom = odom;
        this.shooter = shooter;
        this.telem = telem;
        this.goalPose = goalPose;

        lastTurretAngle = null;
        lastHoodAngle = Double.NaN;
    }

    public static class TurretParameters {
        public final Pose2d turretPose;
        public final Pose2d lookaheadPose;
        public final Pose2d robotPose;
        public final Rotation2d turretAngle;
        public final double hoodAngle;
        public final double flywheelSpeed;
        public final double distanceToTarget;

        public TurretParameters(Pose2d turretPose,
                                Pose2d lookaheadPose,
                                Pose2d robotPose,
                                Rotation2d turretAngle,
                                double hoodAngle,
                                double flywheelSpeed,
                                double distanceToTarget) {
            this.turretPose = turretPose;
            this.lookaheadPose = lookaheadPose;
            this.robotPose = robotPose;
            this.turretAngle = turretAngle;
            this.hoodAngle = hoodAngle;
            this.flywheelSpeed = flywheelSpeed;
            this.distanceToTarget = distanceToTarget;
        }
    }

    public TurretParameters getShootingParameters() {

        robotPose = new Pose2d(odom.getCurrentPose().x(), odom.getCurrentPose().y(), Rotation2d.fromDegrees(odom.getHeading()));
        robotVelocity = odom.getFieldRelativeVelocity(); // X = right, Y = forward
        omega = odom.getAngularVelocity(); // rad/s

        double dt = 0.03;
        double dx = robotVelocity.getX() * dt;
        double dy = robotVelocity.getY() * dt;
        double dTheta = omega * dt;

        Pose2d trueRobotPose = new Pose2d(
                robotPose.getX() + dx,
                robotPose.getY() + dy,
                robotPose.getRotation().plus(new Rotation2d(dTheta))
        );

        turretPose = trueRobotPose.transformBy(new Transform2d(new Translation2d(0, -0.025), new Rotation2d(0)));

        distanceToTarget = goalPose.getDistance(turretPose.getTranslation());

        double robotAngle = trueRobotPose.getRotation().getRadians();
        double turretVelX = robotVelocity.getX() + omega * (-robotToTurret.getY() * Math.sin(robotAngle) + robotToTurret.getX() * Math.cos(robotAngle));
        double turretVelY = robotVelocity.getY() + omega * ( robotToTurret.getX() * Math.cos(robotAngle) + robotToTurret.getY() * Math.sin(robotAngle));

        Pose2d lookaheadPose = turretPose;
        double lookaheadDistance = distanceToTarget;

        for (int i = 0; i < 20; i++) {
            timeOfFlight = shooter.getTimeOfFlightFromDistance(lookaheadDistance);
            double dxLook = turretVelX * timeOfFlight;
            double dyLook = turretVelY * timeOfFlight;

            lookaheadPose = new Pose2d(
                    turretPose.getTranslation().plus(new Translation2d(dxLook, dyLook)),
                    turretPose.getRotation()
            );

            lookaheadDistance = goalPose.getDistance(lookaheadPose.getTranslation());
        }

        Rotation2d turretAngle = new Rotation2d(
                Math.atan2(goalPose.getY() - lookaheadPose.getY(), goalPose.getX() - lookaheadPose.getX())
        );

        double hoodAngle = shooter.getShooterAngleFromDistance(lookaheadDistance);
        double flywheelSpeed = shooter.getShooterSpeedFromDistance(lookaheadDistance);

        if (lastTurretAngle == null) lastTurretAngle = turretAngle;
        if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;

        lastTurretAngle = turretAngle;
        lastHoodAngle = hoodAngle;

        telem.putDashboard("Turret Pose", turretPose);
        telem.putDashboard("Lookahead Pose", lookaheadPose);
        telem.putDashboard("Robot Pose", trueRobotPose);
        telem.putDashboard("Turret Angle", Math.toDegrees(turretAngle.getRadians()));
        telem.putDashboard("Hood Angle", hoodAngle);
        telem.putDashboard("Flywheel Speed", flywheelSpeed);

        return new TurretParameters(
                turretPose,
                lookaheadPose,
                trueRobotPose,
                turretAngle,
                hoodAngle,
                flywheelSpeed,
                lookaheadDistance
        );
    }
}
