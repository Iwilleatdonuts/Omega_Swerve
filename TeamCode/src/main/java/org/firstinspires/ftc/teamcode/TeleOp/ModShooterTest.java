package org.firstinspires.ftc.teamcode.TeleOp;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveModule;
import org.firstinspires.ftc.teamcode.Utilities.OmegaController.OmegaController;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

@TeleOp(name = "Mod Shooter Test", group = "Main")
public class ModShooterTest extends LinearOpMode {

    private EZTelemetry telem;
    private OmegaController driver;
    private OmegaController operator;

    private Shooter s_Shooter;
    private SwerveModule mod1;
    private SwerveModule mod3;

    private double shooterSpeed;

    @Override
    public void runOpMode(){
        boolean areWeWinners = false;

        driver = new OmegaController(gamepad1);
        operator = new OmegaController(gamepad2);

        telem = new EZTelemetry(telemetry);

        s_Shooter = new Shooter(hardwareMap, telem);

        mod1 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod1.modConstants);
        mod3 = new SwerveModule(hardwareMap, telem, Constants.DriveTrainConstants.Mod3.modConstants);

        shooterSpeed = 0;
        mod1.setModuleSetpoint(0);
        mod3.setModuleSetpoint(0);

        waitForStart();

        while (opModeIsActive()) {

            driver.readButtons();
            operator.readButtons();

            if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                shooterSpeed += 0.1;
            }

            if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                shooterSpeed -= 0.1;
            }

            if(driver.wasJustPressed(GamepadKeys.Button.A)) {
                mod1.setModuleSetpoint(90);
                mod3.setModuleSetpoint(90);
            }

            if(driver.wasJustPressed(GamepadKeys.Button.B)) {
                mod1.setModuleSetpoint(0);
                mod3.setModuleSetpoint(0);
            }

            s_Shooter.setShooterSpeed(shooterSpeed);
            mod1.setModulePosition();
            mod3.setModulePosition();

            telem.putTelemetry("Shooter RPM", s_Shooter.getShooterVelocity());
            telem.putTelemetry("Lower Shooter Current", s_Shooter.getLowerCurrentDraw());
            telem.putTelemetry("Upper Shooter Current", s_Shooter.getUpperCurrentDraw());
            telem.putLine();
            telem.putTelemetry("Module 1 degrees", mod1.getDegrees(true));
            telem.putTelemetry("Module 1 Setpoint", mod1.getModuleSetpoint());
            telem.putLine();
            telem.putTelemetry("Module 3 degrees", mod3.getDegrees(true));
            telem.putTelemetry("Module 3 Setpoint", mod3.getModuleSetpoint());
            telem.putLine();

            telem.updateTelemetry();
        }
    }

}
