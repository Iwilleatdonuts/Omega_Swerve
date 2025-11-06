package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

import java.util.HashMap;
import java.util.Map;

public class Swerve extends SubsystemBase {

    private final EZTelemetry telem;

    private final SwerveModule mod0, mod1, mod2, mod3;
    private final IMU imu;

    private boolean enableTelemetry;

    private final Map<String, Object> motorCurrents = new HashMap<>();
    private final Map<String, Object> velocityErrors = new HashMap<>();
    private final Map<String, Object> angleErrors = new HashMap<>();

    private SparkFunOTOS.Pose2D targetPose;

    public Swerve(HardwareMap hardwareMap, EZTelemetry telem){

        this.telem = telem;

        mod0 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod0.modConstants);
        mod1 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod1.modConstants);
        mod2 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod2.modConstants);
        mod3 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod3.modConstants);


        enableTelemetry = false;

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        targetPose = new SparkFunOTOS.Pose2D(1500, 200, 0);

    }

    public void drive(double xVal, double yVal, double rVal, boolean fieldRelative, boolean slowMode){

        if (Math.abs(xVal) < 0.03) {xVal = 0;}
        if (Math.abs(yVal) < 0.03) {yVal = 0;}
        if (Math.abs(rVal) < 0.03) {rVal = 0;}

        double x = yVal;
        double y = -xVal;

        double xValPlaceholder = x;
        double yValPlaceholder = y;

        if(fieldRelative){
            double robotHeading = Math.toRadians(getHeading());
            double cosHeading = Math.cos(robotHeading);
            double sinHeading = Math.sin(robotHeading);

            x = xValPlaceholder * cosHeading + yValPlaceholder * sinHeading;
            y = -xValPlaceholder * sinHeading + yValPlaceholder * cosHeading;
        }

        double r = -rVal;

        double rotationXComponent = r * (Constants.DriveTrainConstants.trackWidth / Constants.DriveTrainConstants.moduleHypotenuse);
        double rotationYComponent = r * (Constants.DriveTrainConstants.wheelbase / Constants.DriveTrainConstants.moduleHypotenuse);

        double xLeftComponent = x - rotationXComponent;
        double xRightComponent = x + rotationXComponent;
        double yBackComponent = y - rotationYComponent;
        double yFrontComponent = y + rotationYComponent;

        double mod0Speed = Math.hypot(xLeftComponent, yFrontComponent);
        double mod1Speed = Math.hypot(xRightComponent, yFrontComponent);
        double mod2Speed = Math.hypot(xLeftComponent, yBackComponent);
        double mod3Speed = Math.hypot(xRightComponent, yBackComponent);

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

        double mod0Angle = normalizeModuleAngle(Math.toDegrees(Math.atan2(yFrontComponent, xLeftComponent)));
        double mod1Angle = normalizeModuleAngle(Math.toDegrees(Math.atan2(yFrontComponent, xRightComponent)));
        double mod2Angle = normalizeModuleAngle(Math.toDegrees(Math.atan2(yBackComponent, xLeftComponent)));
        double mod3Angle = normalizeModuleAngle(Math.toDegrees(Math.atan2(yBackComponent, xRightComponent)));

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
        motorCurrents.clear();
        motorCurrents.put("Mod 0 Current: \t", mod0.getMotorCurrent());
        motorCurrents.put("Mod 1 Current: \t", mod1.getMotorCurrent());
        motorCurrents.put("Mod 2 Current: \t", mod2.getMotorCurrent());
        motorCurrents.put("Mod 3 Current: \t", mod3.getMotorCurrent());
        return motorCurrents;
    }

    public Map<String, Object> getVelocityErrors() {
        velocityErrors.clear();
        velocityErrors.put("Mod 0 Velocity Error: \t", mod0.getVelocityError());
        velocityErrors.put("Mod 1 Velocity Error: \t", mod1.getVelocityError());
        velocityErrors.put("Mod 2 Velocity Error: \t", mod2.getVelocityError());
        velocityErrors.put("Mod 3 Velocity Error: \t", mod3.getVelocityError());
        return velocityErrors;
    }

    public Map<String, Object> getAngularError() {
        angleErrors.clear();
        angleErrors.put("Mod 0 Angle Error: \t", mod0.getWrappedError(mod0.getModuleSetpoint(), mod0.getDegrees(true)));
        angleErrors.put("Mod 1 Angle Error: \t", mod1.getWrappedError(mod1.getModuleSetpoint(), mod1.getDegrees(true)));
        angleErrors.put("Mod 2 Angle Error: \t", mod2.getWrappedError(mod2.getModuleSetpoint(), mod2.getDegrees(true)));
        angleErrors.put("Mod 3 Angle Error: \t", mod3.getWrappedError(mod3.getModuleSetpoint(), mod3.getDegrees(true)));
        return angleErrors;
    }

    private double normalizeModuleAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    public double getHeading() {
        double rotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        rotation = (rotation+360)%360;

        return rotation;
    }

    public void zeroGyro() {
        imu.resetYaw();
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void setTargetPose(double newX, double newY, double newH) {
        targetPose = new SparkFunOTOS.Pose2D(newX, newY, newH);
    }

    public void setTargetPose(SparkFunOTOS.Pose2D newPose) {
        targetPose = newPose;
    }

    public SparkFunOTOS.Pose2D getTargetPose(){
        return targetPose;
    }

    @Override
    public void periodic(){

        if(enableTelemetry) {

            telem.putTelemetry("Robot Heading", getHeading());
            telem.putTelemetry("Target Pose X", getTargetPose().x);
            telem.putTelemetry("Target Pose Y", getTargetPose().y);
            telem.putTelemetry("Target Pose H", getTargetPose().h);

            telem.putDashboard("Robot Heading", getHeading());
            telem.putDashboard("Target Pose X", getTargetPose().x);
            telem.putDashboard("Target Pose Y", getTargetPose().y);
            telem.putDashboard("Target Pose H", getTargetPose().h);

        }

    }
}

