package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Module Test")
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

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Mod0.setDrivePower(m_DriverOp.getLeftY());
            s_Mod1.setDrivePower(m_DriverOp.getLeftY());
            s_Mod2.setDrivePower(m_DriverOp.getLeftY());
            s_Mod3.setDrivePower(m_DriverOp.getLeftY());

            s_Mod0.update();
            s_Mod1.update();
            s_Mod2.update();
            s_Mod3.update();
            s_Sparky.update();
            telemetry.addData("Joystick Angle: \t", getAngleFromJoystick(m_DriverOp.getLeftX(), m_DriverOp.getLeftY()));
            telemetry.update();

        }
    }

    public double getAngleFromJoystick(double x, double y){
        double angle = Math.toDegrees(Math.atan2(y, x));
        if (angle < 0) {
            angle += 360;
        }
        return angle;
    }
}
