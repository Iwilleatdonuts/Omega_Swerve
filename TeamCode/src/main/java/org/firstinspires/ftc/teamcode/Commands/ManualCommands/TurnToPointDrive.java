package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;

public class TurnToPointDrive extends CommandBase {

    private final Telemetry telemetry;
    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;
    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean slowMode;
    private final PIDController anglePID;

    private boolean enableAutoRotate;
    private double turnAngle;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final ElapsedTime timer;

    private double timestamp;

    private final FtcDashboard dashboard;
    private final TelemetryPacket packet;

    public TurnToPointDrive(Telemetry telemetry, FtcDashboard dashboard, Swerve s_Swerve, OTOSSensor s_Sparky, GamepadEx m_Driver, GamepadEx m_Operator){

        this.telemetry = telemetry;
        this.dashboard = dashboard;
        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;
        enableAutoRotate = false;

        turnAngle = 0;

        anglePID = new PIDController(0.0026, 0.01, 0);
//        anglePID = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kF);

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);

        packet = new TelemetryPacket(true);

        timer = new ElapsedTime();

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){

        xLimiter.reset(0);
        yLimiter.reset(0);

    }

    @Override
    public void execute(){

        timestamp = timer.milliseconds();

        m_Driver.readButtons();
        m_Operator.readButtons();

        if(m_Driver.wasJustPressed(GamepadKeys.Button.BACK)){
            enableAutoRotate = !enableAutoRotate;
        }

        if(m_Operator.wasJustPressed(GamepadKeys.Button.Y) || m_Driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            s_Swerve.setTargetPose(s_Sparky.getPose());
        }

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();

        slowMode = m_Driver.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) || m_Driver.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        double xLimited = xLimiter.calculate(xVal);
        double yLimited = yLimiter.calculate(yVal);

        double rotationOutput = 0;

        double rightX = m_Driver.getRightX();
        double rightY = m_Driver.getRightY();

        if(Math.hypot(rightX, rightY) > 0.9) {
            turnAngle = Math.toDegrees(Math.atan2(rightX,rightY));
            turnAngle = (turnAngle + 180) % 360;
        }

        if(Math.hypot(rightX, rightY) > 0.9 || enableAutoRotate) {

            double robotHeading = s_Swerve.getHeading();

            double error = turnAngle - robotHeading;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            rotationOutput = -anglePID.calculate(0, error);

        }

        s_Swerve.drive(xLimited, yLimited, rotationOutput, true, slowMode);

        telemetry.addData("CLT", timer.milliseconds() - timestamp);

        packet.clearLines();

        packet.fieldOverlay().strokeCircle(0, 0, 20);
        dashboard.sendTelemetryPacket(packet);
    }

}
