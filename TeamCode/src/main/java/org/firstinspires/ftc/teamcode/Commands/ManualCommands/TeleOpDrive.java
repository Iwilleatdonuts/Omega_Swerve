package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.math.controller.PIDController;

public class TeleOpDrive {

    private final EZTelemetry telem;
    private final Swerve s_Swerve;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final ElapsedTime timer;

    public TeleOpDrive(EZTelemetry telem, Swerve s_Swerve, OmegaController m_Driver, OmegaController m_Operator){

        this.telem = telem;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        xLimiter = new SlewRateLimiter(4);
        yLimiter = new SlewRateLimiter(4);

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

        double rightX = m_Driver.getRightX();

        double sqrX = Math.pow(Math.abs(rightX), 2);

        rightX = Math.signum(rightX) * sqrX;

        s_Swerve.drive(xLimited, yLimited, rightX, true);

        telem.putTelemetry("CLT", timer.milliseconds() - timestamp);
        telem.putTelemetry("Right X Value", rightX);
        telem.putLine();
    }

}
