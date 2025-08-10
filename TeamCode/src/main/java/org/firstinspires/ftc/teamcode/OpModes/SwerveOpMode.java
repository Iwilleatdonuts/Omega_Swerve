package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Janky Swerve")
public class SwerveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        SwerveModule s_Mod0 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants);
        SwerveModule s_Mod1 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants);
        SwerveModule s_Mod2 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants);
        SwerveModule s_Mod3 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants);

        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telemetry);

        ElapsedTime runtime = new ElapsedTime();

        s_Sparky.configureOTOS();

        double hyp = Math.hypot(Constants.DriveTrainConstants.trackWidth / 2, Constants.DriveTrainConstants.wheelbase / 2);

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            double xJoy = m_DriverOp.getLeftX();
            double yJoy = m_DriverOp.getLeftY();

            if (Math.abs(xJoy) < 0.03) {
                xJoy = 0;
            }
            if (Math.abs(yJoy) < 0.03) {
                yJoy = 0;
            }

            double robotHeading = Math.toRadians(s_Sparky.getHeading());

            double x = xJoy * Math.cos(robotHeading) + yJoy * Math.sin(robotHeading);
            double y = -xJoy * Math.sin(robotHeading) + yJoy * Math.cos(robotHeading);

            double r = m_DriverOp.getRightX();

            if (Math.abs(r) < 0.03){
                r = 0;
            }

            double rotVec = r * (Constants.DriveTrainConstants.wheelbase / hyp);

            double aVec = x - rotVec;
            double bVec = x + rotVec;
            double cVec = y - rotVec;
            double dVec = y + rotVec;

            double mod0Speed = Math.hypot(bVec, dVec);
            double mod1Speed = Math.hypot(bVec, cVec);
            double mod2Speed = Math.hypot(aVec, dVec);
            double mod3Speed = Math.hypot(aVec, cVec);

            double max = Math.max(Math.abs(mod0Speed), Math.abs(mod1Speed));
            max = Math.max(max, Math.abs(mod2Speed));
            max = Math.max(max, Math.abs(mod3Speed));

            if (max > 1.0) {
                mod0Speed /= max;
                mod1Speed /= max;
                mod2Speed /= max;
                mod3Speed /= max;
            }

            double mod0Angle = Math.toDegrees(Math.atan2(-bVec, dVec));
            double mod1Angle = Math.toDegrees(Math.atan2(-bVec, cVec));
            double mod2Angle = Math.toDegrees(Math.atan2(-aVec, dVec));
            double mod3Angle = Math.toDegrees(Math.atan2(-aVec, cVec));
            mod0Angle = (mod0Angle + 360) % 360;
            mod1Angle = (mod1Angle + 360) % 360;
            mod2Angle = (mod2Angle + 360) % 360;
            mod3Angle = (mod3Angle + 360) % 360;

            s_Mod0.setDrivePower(mod0Speed);
            s_Mod1.setDrivePower(mod1Speed);
            s_Mod2.setDrivePower(mod2Speed);
            s_Mod3.setDrivePower(mod3Speed);

            if(Math.abs(m_DriverOp.getLeftY())>0.03 || Math.abs(m_DriverOp.getRightX())>0.03 || Math.abs(m_DriverOp.getLeftX())>0.03){
                s_Mod0.setModuleSetpoint(mod0Angle);
                s_Mod1.setModuleSetpoint(mod1Angle);
                s_Mod2.setModuleSetpoint(mod2Angle);
                s_Mod3.setModuleSetpoint(mod3Angle);
            }

            s_Mod0.setModulePosition();
            s_Mod1.setModulePosition();
            s_Mod2.setModulePosition();
            s_Mod3.setModulePosition();

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Sparky.zeroGyro();
            }

            s_Mod0.update(false);
            s_Mod1.update(false);
            s_Mod2.update(false);
            s_Mod3.update(false);
            s_Sparky.update();

            telemetry.addData("Left Joystick Angle \t", Math.toDegrees(Math.atan2(-m_DriverOp.getLeftX(), m_DriverOp.getLeftY())));
            telemetry.addData("Right X \t", m_DriverOp.getRightX());
            telemetry.update();

        }
    }
}
