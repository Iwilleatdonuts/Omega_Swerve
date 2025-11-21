package org.firstinspires.ftc.teamcode.Utilities.math.controller;

import org.firstinspires.ftc.teamcode.Utilities.math.MathUtil;
import org.firstinspires.ftc.teamcode.Utilities.math.trajectory.TrapezoidProfile;

/**
 * Implements a PID control loop whose setpoint is constrained by a trapezoid profile. Users should
 * call reset() when they first start running the controller to avoid unwanted behavior.
 */
public class ProfiledPIDController {

    private static int instances;
    private final PIDController m_controller;
    private double m_minimumInput;
    private double m_maximumInput;
    private TrapezoidProfile.Constraints m_constraints;
    private TrapezoidProfile m_profile;
    private TrapezoidProfile.State m_goal;
    private TrapezoidProfile.State m_setpoint;

    public ProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints) {
        this(Kp, Ki, Kd, constraints, 0.02);
    }

    public ProfiledPIDController(double Kp, double Ki, double Kd, TrapezoidProfile.Constraints constraints, double period) {
        this.m_goal = new TrapezoidProfile.State();
        this.m_setpoint = new TrapezoidProfile.State();
        this.m_controller = new PIDController(Kp, Ki, Kd, period);
        this.m_constraints = constraints;
        this.m_profile = new TrapezoidProfile(this.m_constraints);
        ++instances;
//        SendableRegistry.add(this, "ProfiledPIDController", instances);
//        MathSharedStore.reportUsage(MathUsageId.kController_ProfiledPIDController, instances);
    }

    public void setPID(double Kp, double Ki, double Kd) {
        this.m_controller.setPID(Kp, Ki, Kd);
    }

    public void setP(double Kp) {
        this.m_controller.setP(Kp);
    }

    public void setI(double Ki) {
        this.m_controller.setI(Ki);
    }

    public void setD(double Kd) {
        this.m_controller.setD(Kd);
    }

    public void setIZone(double iZone) {
        this.m_controller.setIZone(iZone);
    }

    public double getP() {
        return this.m_controller.getP();
    }

    public double getI() {
        return this.m_controller.getI();
    }

    public double getD() {
        return this.m_controller.getD();
    }

    public double getIZone() {
        return this.m_controller.getIZone();
    }

    public double getPeriod() {
        return this.m_controller.getPeriod();
    }

//    public double getPositionTolerance() {
//        return this.m_controller.getErrorTolerance();
//    }
//
//    public double getVelocityTolerance() {
//        return this.m_controller.getErrorDerivativeTolerance();
//    }

    public double getAccumulatedError() {
        return this.m_controller.getAccumulatedError();
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.m_goal = goal;
    }

    public void setGoal(double goal) {
        this.m_goal = new TrapezoidProfile.State(goal, 0.0);
    }

    public TrapezoidProfile.State getGoal() {
        return this.m_goal;
    }

    public boolean atGoal() {
        return this.atSetpoint() && this.m_goal.equals(this.m_setpoint);
    }

    public void setConstraints(TrapezoidProfile.Constraints constraints) {
        this.m_constraints = constraints;
        this.m_profile = new TrapezoidProfile(this.m_constraints);
    }

    public TrapezoidProfile.Constraints getConstraints() {
        return this.m_constraints;
    }

    public TrapezoidProfile.State getSetpoint() {
        return this.m_setpoint;
    }

    public boolean atSetpoint() {
        return this.m_controller.atSetpoint();
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        this.m_controller.enableContinuousInput(minimumInput, maximumInput);
        this.m_minimumInput = minimumInput;
        this.m_maximumInput = maximumInput;
    }

    public void disableContinuousInput() {
        this.m_controller.disableContinuousInput();
    }

    public void setIntegratorRange(double minimumIntegral, double maximumIntegral) {
        this.m_controller.setIntegratorRange(minimumIntegral, maximumIntegral);
    }

    public void setTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance, Double.POSITIVE_INFINITY);
    }

    public void setTolerance(double positionTolerance, double velocityTolerance) {
        this.m_controller.setTolerance(positionTolerance, velocityTolerance);
    }

//    public double getPositionError() {
//        return this.m_controller.getError();
//    }
//
//    public double getVelocityError() {
//        return this.m_controller.getErrorDerivative();
//    }

    public double calculate(double measurement) {
        if (this.m_controller.isContinuousInputEnabled()) {
            double errorBound = (this.m_maximumInput - this.m_minimumInput) / 2.0;
            double goalMinDistance = MathUtil.inputModulus(this.m_goal.position - measurement, -errorBound, errorBound);
            double setpointMinDistance = MathUtil.inputModulus(this.m_setpoint.position - measurement, -errorBound, errorBound);
            this.m_goal.position = goalMinDistance + measurement;
            this.m_setpoint.position = setpointMinDistance + measurement;
        }

        this.m_setpoint = this.m_profile.calculate(this.getPeriod(), this.m_setpoint, this.m_goal);
        return this.m_controller.calculate(measurement, this.m_setpoint.position);
    }

    public double calculate(double measurement, TrapezoidProfile.State goal) {
        this.setGoal(goal);
        return this.calculate(measurement);
    }

    public double calculate(double measurement, double goal) {
        this.setGoal(goal);
        return this.calculate(measurement);
    }

    public double calculate(double measurement, TrapezoidProfile.State goal, TrapezoidProfile.Constraints constraints) {
        this.setConstraints(constraints);
        return this.calculate(measurement, goal);
    }

    public void reset(TrapezoidProfile.State measurement) {
        this.m_controller.reset();
        this.m_setpoint = measurement;
    }

    public void reset(double measuredPosition, double measuredVelocity) {
        this.reset(new TrapezoidProfile.State(measuredPosition, measuredVelocity));
    }

    public void reset(double measuredPosition) {
        this.reset(measuredPosition, 0.0);
    }
}
