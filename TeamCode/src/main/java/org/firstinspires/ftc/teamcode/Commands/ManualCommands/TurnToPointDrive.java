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

    private boolean slowMode;
    private final ProfiledPIDController profiledAnglePID;
    private final PIDController anglePID;
    private final PIDController dynamicAnglePID;

    private boolean enableAutoRotate;
    private double turnAngle;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    private final ElapsedTime timer;

    private double timestamp;

    public TurnToPointDrive(EZTelemetry telem, Swerve s_Swerve, OmegaController m_Driver, OmegaController m_Operator){

        this.telem = telem;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;
        enableAutoRotate = false;

        turnAngle = 0;

        profiledAnglePID = new ProfiledPIDController(0.006, 0.02, 0.00015, new TrapezoidProfile.Constraints(4, 2));
        profiledAnglePID.setIZone(40);
        profiledAnglePID.enableContinuousInput(0, 360);

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

        timestamp = timer.milliseconds();

        m_Driver.readButtons();
        m_Operator.readButtons();

//        if(m_Driver.wasJustPressed(GamepadKeys.Button.BACK)){
//            enableAutoRotate = !enableAutoRotate;
//        }

//        if(m_Operator.wasJustPressed(GamepadKeys.Button.Y) || m_Driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//            s_Swerve.setTargetPose(s_Sparky.getPose());
//        }

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

            double currentHeading = s_Swerve.getHeading();
            if(Math.abs(currentHeading - turnAngle) > 0.5) {
                if(xLimited != 0 || yLimited != 0) {
                    rotationOutput = -dynamicAnglePID.calculate(s_Swerve.getHeading(), turnAngle);
                } else {
                    rotationOutput = -anglePID.calculate(s_Swerve.getHeading(), turnAngle);
                }
            }
        }

        s_Swerve.drive(xLimited, yLimited, rotationOutput, true, slowMode);

//        telem.putTelemetry("TUrn Angle", turnAngle);
//        telem.putTelemetry("Target spec", profiledAnglePID.getGoal().position);
        telem.putTelemetry("CLT", timer.milliseconds() - timestamp);
    }

}
