package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry; // kept for constructor compatibility
import org.firstinspires.ftc.teamcode.Utilities.OmegaPose2D;
import org.firstinspires.ftc.teamcode.Utilities.math.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Utilities.math.kinematics.ChassisSpeeds;

public class Swerve {

    @SuppressWarnings("unused")
    private final EZTelemetry telem;

    private OTOSSensor sparky;

    private final SwerveModule[] mods = new SwerveModule[4];
    private final IMU imu;
    private OmegaPose2D targetPose;

    private final double[] speeds = new double[4];
    private final double[] angles = new double[4];

    //max theoretical speed is 1.93m/s
    //max theoretical angular velocity is 5.20884002936 radians per seconds;
    private final double MAX_SPEED_MPS = 1.93;
    private final double MAX_ANGULAR_VELOCITY_RAD_PER_SECONDS = 5.20884002936;

    private boolean enableTelemetry;

    public Swerve(HardwareMap hardwareMap, EZTelemetry telem){
        this.telem = telem;

        mods[0] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod0.modConstants);
        mods[1] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod1.modConstants);
        mods[2] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod2.modConstants);
        mods[3] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod3.modConstants);

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        targetPose = new OmegaPose2D(0,0,0);

        enableTelemetry = false;
    }

    public Swerve(HardwareMap hardwareMap, EZTelemetry telem, OTOSSensor sparky){
        this.telem = telem;
        this.sparky = sparky;

        mods[0] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod0.modConstants);
        mods[1] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod1.modConstants);
        mods[2] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod2.modConstants);
        mods[3] = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod3.modConstants);

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logo, usb);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        targetPose = new OmegaPose2D(0,0,0);

        enableTelemetry = false;
    }

    public void stop(){
        for (int i = 0; i < 4; i++) {
            mods[i].setDrivePower(0);
            mods[i].setModuleSetpoint(mods[i].getDegrees(true));
            mods[i].setTurnSpeed(0);
        }
    }

    public void drive(double xVal, double yVal, double rVal, boolean fieldRelative, boolean slowMode){

        if (Math.abs(xVal) < 0.03) xVal = 0;
        if (Math.abs(yVal) < 0.03) yVal = 0;
        if (Math.abs(rVal) < 0.03) rVal = 0;

        double x = yVal;
        double y = -xVal;

        if (fieldRelative) {
            double headingRad = Math.toRadians(getHeading());
            double cos = Math.cos(headingRad);
            double sien = Math.sin(headingRad);
            double xTemp = x;
            double yTemp = y;

            x = xTemp * cos + yTemp * sien;
            y = -xTemp * sien + yTemp * cos;
        }

        double r = -rVal;

        final double rotX = r * (Constants.DriveTrainConstants.trackWidth / Constants.DriveTrainConstants.moduleHypotenuse);
        final double rotY = r * (Constants.DriveTrainConstants.wheelbase / Constants.DriveTrainConstants.moduleHypotenuse);

        final double xLeft = x - rotX;
        final double xRight = x + rotX;
        final double yBack = y - rotY;
        final double yFront = y + rotY;

        double mod0Speed = Math.hypot(xLeft, yFront);
        double mod1Speed = Math.hypot(xRight, yFront);
        double mod2Speed = Math.hypot(xLeft, yBack);
        double mod3Speed = Math.hypot(xRight, yBack);

        double max = mod0Speed;
        if (mod1Speed > max) max = mod1Speed;
        if (mod2Speed > max) max = mod2Speed;
        if (mod3Speed > max) max = mod3Speed;

        if (max > 1.0) {
            double inv = 1.0 / max;
            mod0Speed *= inv; mod1Speed *= inv; mod2Speed *= inv; mod3Speed *= inv;
        }

        if (slowMode) {
            mod0Speed *= 0.3; mod1Speed *= 0.3; mod2Speed *= 0.3; mod3Speed *= 0.3;
        }

        double a0 = Math.toDegrees(Math.atan2(yFront, xLeft));
        if (a0 < 0) a0 += 360.0;
        double a1 = Math.toDegrees(Math.atan2(yFront, xRight));
        if (a1 < 0) a1 += 360.0;
        double a2 = Math.toDegrees(Math.atan2(yBack, xLeft));
        if (a2 < 0) a2 += 360.0;
        double a3 = Math.toDegrees(Math.atan2(yBack, xRight));
        if (a3 < 0) a3 += 360.0;

        mods[0].setDrivePower(mod0Speed);
        mods[1].setDrivePower(mod1Speed);
        mods[2].setDrivePower(mod2Speed);
        mods[3].setDrivePower(mod3Speed);

        if (xVal != 0 || yVal != 0 || rVal != 0) {
            mods[0].setModuleSetpoint(a0);
            mods[1].setModuleSetpoint(a1);
            mods[2].setModuleSetpoint(a2);
            mods[3].setModuleSetpoint(a3);
        }

        mods[0].setModulePosition();
        mods[1].setModulePosition();
        mods[2].setModulePosition();
        mods[3].setModulePosition();
    }

    public void drive(ChassisSpeeds speeds) {

//        ChassisSpeeds newSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, Rotation2d.fromDegrees(getHeading()));

        double x = speeds.vxMetersPerSecond/MAX_SPEED_MPS;
        double y = speeds.vyMetersPerSecond/MAX_SPEED_MPS;
        double r = speeds.omegaRadiansPerSecond/MAX_ANGULAR_VELOCITY_RAD_PER_SECONDS;

        drive(x, y, r, true, false);

    }

    public double getHeading() {
        if (sparky == null) {
            double rotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            rotation += 360.0;
            rotation %= 360.0;
            return rotation;
        } else {
            return sparky.getHeading();
        }
    }

    public void zeroGyro() {
        imu.resetYaw();
    }

    public void setTargetPose(double newX, double newY, double newH) {
        targetPose = new OmegaPose2D(newX, newY, newH);
    }

    public void setTargetPose(OmegaPose2D newPose) {
        targetPose = newPose;
    }

    public OmegaPose2D getTargetPose() {
        return targetPose;
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

        if(enableTelemetry) {

            telem.putTelemetry("Robot Heading", getHeading());
            telem.putTelemetry("Target Pose X", getTargetPose().x());
            telem.putTelemetry("Target Pose Y", getTargetPose().y());
            telem.putTelemetry("Target Pose H", getTargetPose().r());

            telem.putDashboard("Robot Heading", getHeading());
            telem.putDashboard("Target Pose X", getTargetPose().x());
            telem.putDashboard("Target Pose Y", getTargetPose().y());
            telem.putDashboard("Target Pose H", getTargetPose().r());

        }

    }
}