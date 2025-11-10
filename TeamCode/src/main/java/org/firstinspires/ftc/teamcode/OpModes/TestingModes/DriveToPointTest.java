package org.firstinspires.ftc.teamcode.OpModes.TestingModes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.DriveToDashboardPoint;
import org.firstinspires.ftc.teamcode.Subsystems.OTOSSensor;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Utilities.EZTelemetry;

//http://192.168.43.1:8080/dash
//adb connect 192.168.43.1:5555
@TeleOp(name = "Auto Drive Test", group = "Testing")
public class DriveToPointTest extends CommandOpMode {

    private final EZTelemetry telem = new EZTelemetry(telemetry);

    private Swerve s_Swerve;
    private OTOSSensor s_Sparky;

    @Override
    public void initialize() {


        s_Swerve = new Swerve(hardwareMap, telem);
        s_Sparky = new OTOSSensor(hardwareMap, telem);

        s_Sparky.toggleTelemetry();

        s_Swerve.setDefaultCommand(new DriveToDashboardPoint(s_Swerve, s_Sparky, telem));

        schedule(new RunCommand(() -> {
            s_Sparky.periodic();
            telem.updateAll();
        }));

    }

}
