package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "New Swerve")
public class NewSwerveOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        Swerve s_Swerve = new Swerve(hardwareMap, telemetry);

        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telemetry);

        ElapsedTime runtime = new ElapsedTime();

        s_Sparky.configureOTOS();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Swerve.drive(
                    m_DriverOp.getLeftY(),
                    m_DriverOp.getLeftX(),
                    m_DriverOp.getRightX(),
                    true
            );

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Sparky.zeroGyro();
            }

            s_Swerve.update();
            telemetry.update();

        }
    }
}
