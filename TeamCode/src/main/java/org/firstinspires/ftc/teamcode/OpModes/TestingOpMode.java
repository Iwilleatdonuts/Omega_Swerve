package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;

@TeleOp(name = "Test Mode")
public class TestingOpMode extends LinearOpMode {

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

        boolean runTurnos = false;

        s_Sparky.configureOTOS();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Sparky.zeroGyro();
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.BACK)){
                runTurnos = !runTurnos;
            }

            double drivePower = m_DriverOp.getLeftY();

            s_Mod0.setDrivePower(drivePower);
            s_Mod1.setDrivePower(drivePower);
            s_Mod2.setDrivePower(drivePower);
            s_Mod3.setDrivePower(drivePower);

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

            if(runTurnos) {
                s_Mod0.setModulePosition();
                s_Mod1.setModulePosition();
                s_Mod2.setModulePosition();
                s_Mod3.setModulePosition();
            } else {
                s_Mod0.setTurnSpeed(0);
                s_Mod1.setTurnSpeed(0);
                s_Mod2.setTurnSpeed(0);
                s_Mod3.setTurnSpeed(0);
            }

            if(m_DriverOp.isDown(GamepadKeys.Button.LEFT_BUMPER)){
                s_Mod0.setDrivePower(0.5);
                s_Mod1.setDrivePower(0.5);
                s_Mod2.setDrivePower(0.5);
                s_Mod3.setDrivePower(0.5);
            } else {
                s_Mod0.setDrivePower(0);
                s_Mod1.setDrivePower(0);
                s_Mod2.setDrivePower(0);
                s_Mod3.setDrivePower(0);
            }

            telemetry.addData("Turn Servos", runTurnos);
            s_Mod0.update();
            s_Mod1.update();
            s_Mod2.update();
            s_Mod3.update();
            s_Sparky.update();
            telemetry.addData("Left Joystick Angle \t", Math.atan2(-m_DriverOp.getLeftX(), m_DriverOp.getLeftY()));
            telemetry.addData("Right X \t", m_DriverOp.getRightX());
            telemetry.update();

        }
    }

    public double getAngleFromJoystick(double x, double y){

        double angle = Math.toDegrees(Math.atan2(y, x));

        if (angle < 0) {
            angle += 360;
        }

        angle -= 90;

        if (angle < 0) {
            angle += 360;
        }

        return angle;
    }
}
