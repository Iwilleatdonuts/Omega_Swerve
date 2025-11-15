package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.Controller.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;
import org.firstinspires.ftc.teamcode.Utilities.PIDController;

public class TurnToPointDrive {

    private final EZTelemetry telem;
    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;
    private final OmegaController m_Driver;
    private final OmegaController m_Operator;

    private boolean slowMode;
    private final PIDController staticAnglePID;
    private final PIDController dynamicAnglePID;

    private boolean enableAutoRotate;
    private double turnAngle;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final ElapsedTime timer;

    private double timestamp;

    public TurnToPointDrive(EZTelemetry telem, Swerve s_Swerve, OTOSSensor s_Sparky, OmegaController m_Driver, OmegaController m_Operator){

        this.telem = telem;
        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;
        enableAutoRotate = false;

        turnAngle = 0;

        staticAnglePID = new PIDController(0.006, 0.02, 0.00015);
        staticAnglePID.setIZone(40);
        staticAnglePID.enableContinuousInput(0, 360);

        dynamicAnglePID = new PIDController(0.0025, 0.015, 0.0001);
//        dynamicAnglePID = new PIDController(PIDTuning.k1P, PIDTuning.k1I, PIDTuning.k1D);
        dynamicAnglePID.setIZone(50);
        dynamicAnglePID.enableContinuousInput(0, 360);
//        staticAnglePID = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kF);

        xLimiter = new SlewRateLimiter(2);
        yLimiter = new SlewRateLimiter(2);

        timer = new ElapsedTime();
    }

    public void initialize(){

        xLimiter.reset(0);
        yLimiter.reset(0);

    }

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

            double currentHeading = s_Sparky.getHeading();
            if(Math.abs(currentHeading - turnAngle) > 0.5) {
                if(xLimited != 0 || yLimited != 0) {
                    rotationOutput = -dynamicAnglePID.calculate(s_Sparky.getHeading(), turnAngle);
                } else {
                    rotationOutput = -staticAnglePID.calculate(s_Sparky.getHeading(), turnAngle);
                }
            }
        }

        s_Swerve.drive(xLimited, yLimited, rotationOutput, true, slowMode);

        telem.putTelemetry("CLT", timer.milliseconds() - timestamp);
    }

}
