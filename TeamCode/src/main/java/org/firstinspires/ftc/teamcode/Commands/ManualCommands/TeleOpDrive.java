package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;

public class TeleOpDrive extends CommandBase {

    private final Swerve s_Swerve;
    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean slowMode;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final SlewRateLimiter rLimiter;

    private final Telemetry telemetry;

    private final ElapsedTime timer;

    private double timestamp;

    private final FtcDashboard dashboard;
    private final TelemetryPacket packet;

    public TeleOpDrive(Telemetry telemetry, FtcDashboard dashboard, Swerve s_Swerve, GamepadEx m_Driver, GamepadEx m_Operator){

        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);
        rLimiter = new SlewRateLimiter(5);

        packet = new TelemetryPacket(true);

        timer = new ElapsedTime();

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize() {

        xLimiter.reset(0);
        yLimiter.reset(0);
        rLimiter.reset(0);

    }

    @Override
    public void execute(){

        timestamp = timer.milliseconds();

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();
        double rVal = m_Driver.getRightX();

        double xLimited = xLimiter.calculate(xVal);
        double yLimited = yLimiter.calculate(yVal);
        double rLimited = rLimiter.calculate(rVal);


        slowMode = m_Driver.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) || m_Driver.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        s_Swerve.drive(xLimited, yLimited, rLimited, true, slowMode);

        telemetry.addData("CLT", timer.milliseconds() - timestamp);

//        packet.fieldOverlay().strokeCircle(0, 0, 5);
        dashboard.sendTelemetryPacket(packet);

    }

}
