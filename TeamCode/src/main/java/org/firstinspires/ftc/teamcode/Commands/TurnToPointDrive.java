package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.PIDTuning;

public class TurnToPointDrive extends CommandBase {

    private final Telemetry telemetry;
    private final Swerve s_Swerve;
    private final OTOSSensor s_Sparky;
    private final GamepadEx m_Driver;
    private final GamepadEx m_Operator;

    private boolean slowMode;
    private boolean dashboardDriving;

    private final PIDController anglePID;

    public TurnToPointDrive(Telemetry telemetry, Swerve s_Swerve, OTOSSensor s_Sparky, GamepadEx m_Driver, GamepadEx m_Operator){

        this.telemetry = telemetry;
        this.s_Swerve = s_Swerve;
        this.s_Sparky = s_Sparky;
        this.m_Driver = m_Driver;
        this.m_Operator = m_Operator;

        slowMode = false;
        dashboardDriving = false;

        anglePID = new PIDController(0.0026, 0.035, 0);
//        anglePID = new PIDController(PIDTuning.kP, PIDTuning.kI, PIDTuning.kF);

        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){

        if(m_Operator.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)){
            dashboardDriving = !dashboardDriving;
        }

        double xVal = m_Driver.getLeftX();
        double yVal = m_Driver.getLeftY();

        if(dashboardDriving){
            xVal = m_Driver.getLeftY();
            yVal = m_Driver.getLeftX();
        }

        slowMode = m_Driver.isDown(GamepadKeys.Button.LEFT_STICK_BUTTON) || m_Driver.isDown(GamepadKeys.Button.RIGHT_STICK_BUTTON);

        double rotationOutput = 0;

        double rightX = m_Driver.getRightX();
        double rightY = m_Driver.getRightY();

        if(Math.hypot(rightX, rightY) > 0.9) {

            double robotHeading = s_Sparky.getHeading();

            double turnAngle = Math.toDegrees(Math.atan2(rightX,rightY));
            turnAngle = (turnAngle + 180) % 360;

            double error = turnAngle - robotHeading;
            error = ((error + 180) % 360 + 360) % 360 - 180;

            double placeholder = turnAngle - error;

            rotationOutput = -anglePID.calculate(placeholder, turnAngle);

            telemetry.addData("Right x", rightX);
            telemetry.addData("Right y", rightY);
            telemetry.addData("Turn ANgle", turnAngle);
            telemetry.addData("error", error);
            telemetry.update();
        }

        s_Swerve.drive(xVal, yVal, rotationOutput, true, slowMode);
    }

}
