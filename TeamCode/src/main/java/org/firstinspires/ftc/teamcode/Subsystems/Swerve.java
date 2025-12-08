package org.firstinspires.ftc.teamcode.Subsystems;

import static com.pedropathing.math.MathFunctions.findNormalizingScaling;

import com.pedropathing.Drivetrain;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

public class Swerve extends Drivetrain {

    private final EZTelemetry telem;

    private final SwerveModule[] mods = new SwerveModule[4];
    private final IMU imu;

    //max theoretical speed is 1.93m/s
    //max theoretical angular velocity is 5.20884002936 radians per seconds;
    private double MAX_SPEED_MPS = 1.93;
    private final double MAX_ANGULAR_VELOCITY_RAD_PER_SECONDS = 5.20884002936;

    private boolean enableTelemetry;

    protected Vector lastTranslationalVector = new Vector();
    protected Vector lastHeadingPower = new Vector();
    protected Vector lastCorrectivePower = new Vector();
    protected Vector lastPathingPower = new Vector();
    protected double lastHeading = 0;


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

    public void drivePrep(double xVal, double yVal, double rVal, boolean fieldRelative){

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

        double a0 = Math.toDegrees(Math.atan2(yFront, xLeft));
        if (a0 < 0) a0 += 360.0;
        double a1 = Math.toDegrees(Math.atan2(yFront, xRight));
        if (a1 < 0) a1 += 360.0;
        double a2 = Math.toDegrees(Math.atan2(yBack, xLeft));
        if (a2 < 0) a2 += 360.0;
        double a3 = Math.toDegrees(Math.atan2(yBack, xRight));
        if (a3 < 0) a3 += 360.0;

        mods[0].setDrivePower(0);
        mods[1].setDrivePower(0);
        mods[2].setDrivePower(0);
        mods[3].setDrivePower(0);

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

    public double getHeading() {
            double rotation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            rotation += 360.0;
            rotation %= 360.0;
            return rotation;
    }

    public void zeroGyro() {
        imu.resetYaw();
    }

    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() >= maxPowerScaling) {
            correctivePower.setMagnitude(maxPowerScaling);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    0
            };
        }
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        if (scaleDown(correctivePower, headingPower, true)) {
            headingPower = scaledVector(correctivePower, headingPower, true);
            return new double[] {
                    correctivePower.getXComponent(),
                    correctivePower.getYComponent(),
                    headingPower.dot(new Vector(-1, robotHeading))
            };
        } else {
            Vector combinedStatic = correctivePower.plus(headingPower);
            if (scaleDown(combinedStatic, pathingPower, false)) {
                pathingPower = scaledVector(combinedStatic, pathingPower, false);
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(-1, robotHeading))
                };
            } else {
                Vector combinedMovement = correctivePower.plus(pathingPower);
                return new double[] {
                        combinedMovement.getXComponent(),
                        combinedMovement.getYComponent(),
                        headingPower.dot(new Vector(-1, robotHeading))
                };
            }
        }
    }

    private boolean scaleDown(Vector staticVector, Vector variableVector, boolean useMinus) {
        return (staticVector.plus(variableVector).getMagnitude() >= maxPowerScaling) ||
                (useMinus && staticVector.minus(variableVector).getMagnitude() >= maxPowerScaling);
    }

    private Vector scaledVector(Vector staticVector, Vector variableVector, boolean useMinus) {
        double scalingFactor = useMinus
                ? Math.min(
                findNormalizingScaling(staticVector, variableVector, maxPowerScaling),
                findNormalizingScaling(staticVector, variableVector.times(-1), maxPowerScaling)
        )
                : findNormalizingScaling(staticVector, variableVector, maxPowerScaling);
        return variableVector.times(scalingFactor);
    }

    @Override
    public void updateConstants() {
    }

    @Override
    public void breakFollowing() {
        for (SwerveModule pod : mods) {
            pod.setDrivePower(0);
        }
    }

    @Deprecated
    @Override
    public void runDrive(double[] drivePowers) {

    }

    @Override
    public void runDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        double[] calculatedDrive = calculateDrive(correctivePower, headingPower, pathingPower, robotHeading);

        Vector translationalVector = new Vector();
        translationalVector.setOrthogonalComponents(calculatedDrive[0], calculatedDrive[1]);

        lastPathingPower = pathingPower;
        lastCorrectivePower = correctivePower;
        lastTranslationalVector = translationalVector;
        lastHeadingPower = headingPower;
        lastHeading = robotHeading;

        translationalVector.rotateVector(-robotHeading);
        drive(translationalVector.getXComponent(),
                translationalVector.getYComponent(),
                calculatedDrive[2],
                false,
                false);
    }

    @Override
    public void startTeleopDrive() {
        for(SwerveModule mod : mods) {
            mod.setDrivePower(0);
            mod.setModuleSetpoint(mod.getDegrees(true));
            mod.setTurnSpeed(0);
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        for(SwerveModule mod : mods) {
            mod.setDrivePower(0);
            mod.setModuleSetpoint(mod.getDegrees(true));
            mod.setTurnSpeed(0);
        }
    }

    @Override
    public double xVelocity() {
        return MAX_SPEED_MPS;
    }

    @Override
    public double yVelocity() {
        return MAX_SPEED_MPS;
    }

    @Override
    public void setXVelocity(double xMovement) {
        MAX_SPEED_MPS = xMovement;
    }

    @Override
    public void setYVelocity(double yMovement) {
        MAX_SPEED_MPS = yMovement;
    }

    @Override
    public double getVoltage() {
        return getNominalVoltage();
    }

    @Override
    public String debugString() {
//        return "Swerve{" +
//                "\nforward input=" + lastForward +
//                "\n, strafe input=" + lastStrafe +
//                "\n, rotation input=" + lastRotation +
//                "\n, unrotated translationVector x" + lastTranslationalVector.getXComponent() +
//                "\n, unrotated translationVector y" + lastTranslationalVector.getYComponent() +
//                "\n, correctivePower x" + lastCorrectivePower.getXComponent() +
//                "\n, correctivePower y" + lastCorrectivePower.getYComponent() +
//                "\n, pathingPower x" + lastPathingPower.getXComponent() +
//                "\n, pathingPower y" + lastPathingPower.getYComponent() +
//                "\n, headingPower magnitude" + lastHeadingPower.getMagnitude() +
//                "\n, headingPower direction" + lastHeadingPower.getTheta() +
//                "\nrobot heading" + lastHeading +
//                ", leftFront=" + leftFrontPod.debugString() +
//                ", rightFront=" + rightFrontPod.debugString() +
//                ", rightRear=" + rightRearPod.debugString() +
//                "\n}";
        return "Hi";
    }


//    @Override
    public void setTeleOpDrive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, true, false);
    }

    public void toggleTelemetry() {
        enableTelemetry = !enableTelemetry;
    }

    public void skadoodle(){

    }
}