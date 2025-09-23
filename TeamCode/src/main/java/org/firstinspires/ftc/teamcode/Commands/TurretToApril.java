package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.AprilVision;
import org.firstinspires.ftc.teamcode.Subsystems.Swerve;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class TurretToApril extends CommandBase {

    private final Turret s_Turret;
    private final AprilVision s_Vision;

    public TurretToApril(Turret s_Turret, AprilVision s_Vision){

        this.s_Turret = s_Turret;
        this.s_Vision = s_Vision;

        addRequirements(s_Turret);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

}
