package org.firstinspires.ftc.teamcode.Commands.ManualCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.SlewRateLimiter;

public class TurnToPointDrive extends CommandBase {

    private final Telemetry telemetry;
    private final Swerve s_Swerve;
    private final GamepadEx m_Driver;

    private boolean slowMode;
    private final PIDController anglePID;

    private boolean enableAutoRotate;
    private double turnAngle;

    private final SlewRateLimiter xLimiter;
    private final SlewRateLimiter yLimiter;

    public TurnToPointDrive(Telemetry telemetry, Swerve s_Swerve, GamepadEx m_Driver){

        this.telemetry = telemetry;
        this.s_Swerve = s_Swerve;
        this.m_Driver = m_Driver;

        slowMode = false;
        enableAutoRotate = false;

        turnAngle = 0;

        anglePID = new PIDController(0.0026, 0.01, 0);
//        anglePID = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kF);

        xLimiter = new SlewRateLimiter(5);
        yLimiter = new SlewRateLimiter(5);

        addRequirements(s_Swerve);
    }

    @Override
    public void initialize(){

        xLimiter.reset(0);
        yLimiter.reset(0);

    }

    @Override
    public void execute(){

        m_Driver.readButtons();

        if(m_Driver.wasJustPressed(GamepadKeys.Button.BACK)){
            enableAutoRotate = !enableAutoRotate;
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
    }

}
