package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;

public class TeleOpDrive {

    private final EZTelemetry telem;
    private final Swerve s_Swerve;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;

    private boolean slowMode;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final SlewRateLimiter rLimiter;

    private final ElapsedTime timer;

    private double timestamp;

    public TeleOpDrive(EZTelemetry telem, Swerve s_Swerve, OmegaController m_Driver, OmegaController m_Operator){

        this.telem = telem;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);
        rLimiter = new SlewRateLimiter(5);

        timer = new ElapsedTime();
    }

    public void initialize() {

        xLimiter.reset(0);
        yLimiter.reset(0);
        rLimiter.reset(0);

    }

    public void execute(){

        timestamp = timer.milliseconds();

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();
        double rVal = m_Driver.getRightX();

        double xLimited = xLimiter.calculate(xVal);
        double yLimited = yLimiter.calculate(yVal);
        double rLimited = rLimiter.calculate(rVal);

        s_Swerve.drive(xLimited, yLimited, rLimited, true);

        telem.putTelemetry("CLT", timer.milliseconds() - timestamp);

    }

}
