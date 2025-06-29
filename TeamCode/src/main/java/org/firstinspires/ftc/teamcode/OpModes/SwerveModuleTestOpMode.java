package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Swerve Module Test")
public class SwerveModuleTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {

        GamepadEx m_DriverOp = new GamepadEx(gamepad1);
        GamepadEx m_OperatorOp = new GamepadEx(gamepad2);

        SwerveModule s_Mod0 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod0.modConstants);
        SwerveModule s_Mod1 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod1.modConstants);
        SwerveModule s_Mod2 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod2.modConstants);
        SwerveModule s_Mod3 = new SwerveModule(hardwareMap, telemetry, Constants.DriveTrainConstants.Mod3.modConstants);

        ElapsedTime runtime = new ElapsedTime();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Mod0.setDrivePower(m_DriverOp.getLeftY());

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                if(m_DriverOp.isDown(GamepadKeys.Button.A)){
                    s_Mod0.setModuleSetpoint(0);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.B)){
                    s_Mod1.setModuleSetpoint(0);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.X)){
                    s_Mod2.setModuleSetpoint(0);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.Y)){
                    s_Mod3.setModuleSetpoint(0);
                }
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                if(m_DriverOp.isDown(GamepadKeys.Button.A)){
                    s_Mod0.setModuleSetpoint(90);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.B)){
                    s_Mod1.setModuleSetpoint(90);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.X)){
                    s_Mod2.setModuleSetpoint(90);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.Y)){
                    s_Mod3.setModuleSetpoint(90);
                }
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                if(m_DriverOp.isDown(GamepadKeys.Button.A)){
                    s_Mod0.setModuleSetpoint(180);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.B)){
                    s_Mod1.setModuleSetpoint(180);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.X)){
                    s_Mod2.setModuleSetpoint(180);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.Y)){
                    s_Mod3.setModuleSetpoint(180);
                }
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
                if(m_DriverOp.isDown(GamepadKeys.Button.A)){
                    s_Mod0.setModuleSetpoint(270);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.B)){
                    s_Mod1.setModuleSetpoint(270);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.X)){
                    s_Mod2.setModuleSetpoint(270);
                }
                if(m_DriverOp.isDown(GamepadKeys.Button.Y)){
                    s_Mod3.setModuleSetpoint(270);
                }
            }

            s_Mod0.update();
            s_Mod1.update();
            s_Mod2.update();
            s_Mod3.update();
            telemetry.update();

        }
    }
}
