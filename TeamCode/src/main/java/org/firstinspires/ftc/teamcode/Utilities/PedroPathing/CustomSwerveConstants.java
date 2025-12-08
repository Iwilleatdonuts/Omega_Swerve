package org.firstinspires.ftc.teamcode.Utilities.PedroPathing;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class CustomSwerveConstants {
    public double xVelocity = 60.2239;
    public double yVelocity = 60.1371;
    private double[] convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
    public Vector frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();

    public  double maxPower = 1;

    public  double motorCachingThreshold = 0.01;
    public  boolean useVoltageCompensation = true;
    public  double nominalVoltage = 13.0;
    public  double staticFrictionCoefficient = 0.1;

    public String leftFrontMotorName = "drive0";
    public String leftRearMotorName = "drive2";
    public String rightFrontMotorName = "drive1";
    public String rightRearMotorName = "drive3";
    public String leftFrontServoName = "angle0";
    public String leftRearServoName = "angle2";
    public String rightFrontServoName = "angle1";
    public String rightRearServoName = "angle3";
    public String leftFrontEncoderName = "angleFeedback0";
    public String leftRearEncoderName = "angleFeedback2";
    public String rightFrontEncoderName = "angleFeedback1";
    public String rightRearEncoderName = "angleFeedback2";
    public double lfEncoderMaxVoltage = 3.3;
    public double lrEncoderMaxVoltage = 3.3;
    public double rfEncoderMaxVoltage = 3.3;
    public double rrEncoderMaxVoltage = 3.3;
    public double lfEncoderMinVoltage = 0;
    public double lrEncoderMinVoltage = 0;
    public double rfEncoderMinVoltage = 0;
    public double rrEncoderMinVoltage = 0;
    public double lfEncoderOffset = -1.5441;
    public double lrEncoderOffset = 1.6676;
    public double rfEncoderOffset = 2.6131;
    public double rrEncoderOffset = -2.9261;
    public double lfServoBasePower = 0.22;
    public double lrServoBasePower = 0.22;
    public double rfServoBasePower = 0.22;
    public double rrServoBasePower = 0.22;
    public double lfRotationKP = 0.009;
    public double lrRotationKP = 0.009;
    public double rfRotationKP = 0.009;
    public double rrRotationKP = 0.009;
    public double lfRotationKI = 0.0;
    public double lrRotationKI = 0.0;
    public double rfRotationKI = 0.0;
    public double rrRotationKI = 0.0;
    public double lfRotationKD = 0.0;
    public double lrRotationKD = 0.0;
    public double rfRotationKD = 0.0;
    public double rrRotationKD = 0.0;
    public  DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public  DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    public CustomSwerveConstants() {
        defaults();
    }

    public CustomSwerveConstants xVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
        return this;
    }

    public CustomSwerveConstants yVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
        return this;
    }

    public CustomSwerveConstants maxPower(double maxPower) {
        this.maxPower = maxPower;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
        return this;
    }

    public CustomSwerveConstants leftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
        return this;
    }

    public CustomSwerveConstants rightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
        return this;
    }

    public CustomSwerveConstants leftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants leftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
        return this;
    }

    public CustomSwerveConstants rightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
        return this;
    }

    public CustomSwerveConstants motorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
        return this;
    }

    public CustomSwerveConstants useVoltageCompensation(boolean useVoltageCompensation) {
        this.useVoltageCompensation = useVoltageCompensation;
        return this;
    }

    public CustomSwerveConstants nominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public CustomSwerveConstants staticFrictionCoefficient(double staticFrictionCoefficient) {
        this.staticFrictionCoefficient = staticFrictionCoefficient;
        return this;
    }

    public double getXVelocity() {
        return xVelocity;
    }

    public void setXVelocity(double xVelocity) {
        this.xVelocity = xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public void setYVelocity(double yVelocity) {
        this.yVelocity = yVelocity;
    }

    public Vector getFrontLeftVector() {
        return frontLeftVector;
    }

    public void setFrontLeftVector(Vector frontLeftVector) {
        this.frontLeftVector = frontLeftVector;
    }

    public double getMaxPower() {
        return maxPower;
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public String getLeftFrontMotorName() {
        return leftFrontMotorName;
    }

    public void setLeftFrontMotorName(String leftFrontMotorName) {
        this.leftFrontMotorName = leftFrontMotorName;
    }

    public String getLeftRearMotorName() {
        return leftRearMotorName;
    }

    public void setLeftRearMotorName(String leftRearMotorName) {
        this.leftRearMotorName = leftRearMotorName;
    }

    public String getRightFrontMotorName() {
        return rightFrontMotorName;
    }

    public void setRightFrontMotorName(String rightFrontMotorName) {
        this.rightFrontMotorName = rightFrontMotorName;
    }

    public String getRightRearMotorName() {
        return rightRearMotorName;
    }

    public void setRightRearMotorName(String rightRearMotorName) {
        this.rightRearMotorName = rightRearMotorName;
    }

    public DcMotorSimple.Direction getLeftFrontMotorDirection() {
        return leftFrontMotorDirection;
    }

    public void setLeftFrontMotorDirection(DcMotorSimple.Direction leftFrontMotorDirection) {
        this.leftFrontMotorDirection = leftFrontMotorDirection;
    }

    public DcMotorSimple.Direction getLeftRearMotorDirection() {
        return leftRearMotorDirection;
    }

    public void setLeftRearMotorDirection(DcMotorSimple.Direction leftRearMotorDirection) {
        this.leftRearMotorDirection = leftRearMotorDirection;
    }

    public DcMotorSimple.Direction getRightFrontMotorDirection() {
        return rightFrontMotorDirection;
    }

    public void setRightFrontMotorDirection(DcMotorSimple.Direction rightFrontMotorDirection) {
        this.rightFrontMotorDirection = rightFrontMotorDirection;
    }

    public DcMotorSimple.Direction getRightRearMotorDirection() {
        return rightRearMotorDirection;
    }

    public void setRightRearMotorDirection(DcMotorSimple.Direction rightRearMotorDirection) {
        this.rightRearMotorDirection = rightRearMotorDirection;
    }

    public double getMotorCachingThreshold() {
        return motorCachingThreshold;
    }

    public void setMotorCachingThreshold(double motorCachingThreshold) {
        this.motorCachingThreshold = motorCachingThreshold;
    }

    public void defaults() {
        xVelocity = 81.34056;
        yVelocity = 65.43028;
        convertToPolar = Pose.cartesianToPolar(xVelocity, -yVelocity);
        frontLeftVector = new Vector(convertToPolar[0], convertToPolar[1]).normalize();
        maxPower = 1;
        motorCachingThreshold = 0.01;
        useVoltageCompensation = true;
        nominalVoltage = 13.0;
        staticFrictionCoefficient = 0.1;
        leftFrontMotorName = "drive0";
        leftRearMotorName = "drive2";
        rightFrontMotorName = "drive1";
        rightRearMotorName = "drive3";
        leftFrontServoName = "angle0";
        leftRearServoName = "angle2";
        rightFrontServoName = "angle1";
        rightRearServoName = "angle3";
        leftFrontEncoderName = "angleFeedback0";
        leftRearEncoderName = "angleFeedback2";
        rightFrontEncoderName = "angleFeedback1";
        rightRearEncoderName = "angleFeedback3";
        lfEncoderMaxVoltage = 3.3;
        lrEncoderMaxVoltage = 3.3;
        rfEncoderMaxVoltage = 3.3;
        rrEncoderMaxVoltage = 3.3;
        lfEncoderMinVoltage = 0;
        lrEncoderMinVoltage = 0;
        rfEncoderMinVoltage = 0;
        rrEncoderMinVoltage = 0;
        lfEncoderOffset = -1.5441;
        lrEncoderOffset = 1.6676;
        rfEncoderOffset = 2.6131;
        rrEncoderOffset = -2.9261;
        lfServoBasePower = 0.22;
        lrServoBasePower = 0.22;
        rfServoBasePower = 0.22;
        rrServoBasePower = 0.22;
        lfRotationKP = 0.009;
        lrRotationKP = 0.009;
        rfRotationKP = 0.009;
        rrRotationKP = 0.009;
        lfRotationKI = 0.0;
        lrRotationKI = 0.0;
        rfRotationKI = 0.0;
        rrRotationKI = 0.0;
        lfRotationKD = 0.0;
        lrRotationKD = 0.0;
        rfRotationKD = 0.0;
        rrRotationKD = 0.0;
        leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        leftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    }
}
