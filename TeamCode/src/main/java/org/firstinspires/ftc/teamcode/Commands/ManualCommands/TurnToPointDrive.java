package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.ProfiledPIDController;
import org.firstinspires.ftc.teamcode.Utilities.math.trajectory.TrapezoidProfile;

public class TurnToPointDrive {

    private final EZTelemetry telem;
    private final Swerve s_Swerve;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;
    private final PIDController anglePID;
    private final PIDController dynamicAnglePID;
    private double turnAngle;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final ElapsedTime timer;

    public TurnToPointDrive(EZTelemetry telem, Swerve s_Swerve, OmegaController m_Driver, OmegaController m_Operator){

        this.telem = telem;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        turnAngle = 0;

        dynamicAnglePID = new PIDController(0.0025, 0.015, 0.0001);
        dynamicAnglePID.setIZone(40);
        dynamicAnglePID.enableContinuousInput(0, 360);

        anglePID = new PIDController(0.006, 0.02, 0.00015);
        anglePID.setIZone(40);
        anglePID.enableContinuousInput(0, 360);

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);

        timer = new ElapsedTime();
    }

    public void initialize(){

        xLimiter.reset(0);
        yLimiter.reset(0);

    }

    public void execute(){

        double timestamp = timer.milliseconds();

        m_Driver.readButtons();
        m_Operator.readButtons();

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();

        double xLimited = xLimiter.calculate(xVal);
        double yLimited = yLimiter.calculate(yVal);

        double rotationOutput = 0;

        double rightX = m_Driver.getRightX();
        double rightY = m_Driver.getRightY();

        if(Math.hypot(rightX, rightY) > 0.9) {
            turnAngle = Math.toDegrees(Math.atan2(rightX,rightY));
            turnAngle = (turnAngle + 180) % 360;
        }

        if(Math.hypot(rightX, rightY) > 0.9) {

            double currentHeading = s_Swerve.getHeading();
            if(Math.abs(currentHeading - turnAngle) > 0.5) {
                if(xLimited != 0 || yLimited != 0) {
                    rotationOutput = -dynamicAnglePID.calculate(s_Swerve.getHeading(), turnAngle);
                } else {
                    rotationOutput = -anglePID.calculate(s_Swerve.getHeading(), turnAngle);
                }
            }
        }

        s_Swerve.setTeleOpDrive(xLimited, yLimited, rotationOutput);

        telem.putTelemetry("CLT", timer.milliseconds() - timestamp);
    }

}
