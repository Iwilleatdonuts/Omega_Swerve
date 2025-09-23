package org.firstinspires.ftc.teamcode.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

import com.acmerobotics.dashboard.FtcDashboard;

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
        Intake s_Intake = new Intake(hardwareMap, telemetry);
        Turret s_Turret = new Turret(hardwareMap, telemetry);
        Shooter s_Shooter = new Shooter(hardwareMap, telemetry);
        OTOSSensor s_Sparky = new OTOSSensor(hardwareMap, telemetry);

//        AprilVision s_Vision = new AprilVision(hardwareMap, telemetry);

        FtcDashboard dashboard = FtcDashboard.getInstance();
//        dashboard.startCameraStream(s_Vision.getAprilCamera(), 30);

        ElapsedTime runtime = new ElapsedTime();

        boolean runTurns = false;
        boolean enableOtherTelemetries = false;

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {

            m_DriverOp.readButtons();
            m_OperatorOp.readButtons();

            s_Intake.setSpeed(m_DriverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - m_DriverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.START)){
                s_Mod0.toggleTelemetry();
                s_Mod1.toggleTelemetry();
                s_Mod2.toggleTelemetry();
                s_Mod3.toggleTelemetry();
                s_Sparky.toggleTelemetry();
                s_Intake.toggleTelemetry();
                s_Turret.toggleTelemetry();
//                s_Vision.toggleTelemetry();
                enableOtherTelemetries = !enableOtherTelemetries;
            }

            if(m_DriverOp.wasJustPressed(GamepadKeys.Button.BACK)){
                runTurns = !runTurns;
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.START)) {
                s_Sparky.zeroGyro();
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.BACK)) {
                s_Turret.resetTurretPosition();
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                s_Turret.setSetpoint(0);
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                s_Turret.setSetpoint(90);
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                s_Turret.setSetpoint(180);
            }

            if(m_OperatorOp.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                s_Turret.setSetpoint(270);
            }

            double operatorJoystickAngle = Math.toDegrees(Math.atan2(-m_OperatorOp.getLeftX(), m_OperatorOp.getLeftY()));
            operatorJoystickAngle += 360;
            operatorJoystickAngle %= 360;

            if(Math.hypot(m_OperatorOp.getLeftX(), m_OperatorOp.getLeftY()) > 0.9) {
                s_Turret.setSetpoint(operatorJoystickAngle);
            }

            if(m_OperatorOp.isDown(GamepadKeys.Button.A)) {
                s_Turret.runToSetpoint();
            } else {
                s_Turret.setSpeed(0);
            }

            s_Shooter.setShooterSpeed(m_DriverOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

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

            if(runTurns) {
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

            s_Mod0.periodic();
            s_Mod1.periodic();
            s_Mod2.periodic();
            s_Mod3.periodic();
            s_Intake.periodic();
            s_Turret.periodic();
//            s_Vision.periodic(dashboard);

            if(enableOtherTelemetries){
                telemetry.addData("Turn Servos", runTurns);
                telemetry.addData("Left Joystick Angle \t", Math.atan2(-m_DriverOp.getLeftX(), m_DriverOp.getLeftY()));
                telemetry.addData("Right X \t", m_DriverOp.getRightX());
                telemetry.addData("Operator Left Joystick \t", operatorJoystickAngle);
            }
            if (!enableOtherTelemetries) {
                telemetry.addLine("Telemetry Off");
            }
            telemetry.update();


        }
    }
}
