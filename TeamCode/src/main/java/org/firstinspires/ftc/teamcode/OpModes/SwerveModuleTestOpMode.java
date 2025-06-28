package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Module Test")
public class SwerveModuleTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        SwerveModule s_Swerve = new SwerveModule(hardwareMap, telemetry);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Swerve.setDrivePower(m_DriverOp.getLeftY());

//            s_Swerve.setTurnSpeed(m_DriverOp.getRightX());

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.A)){
                s_Swerve.setModuleSetpoint(270);
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.B)){
                s_Swerve.setModuleSetpoint(0);
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.Y)){
                s_Swerve.setModuleSetpoint(90);
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.X)){
                s_Swerve.setModuleSetpoint(180);
            }


            s_Swerve.update();
            telemetry.update();

        }
    }
}
