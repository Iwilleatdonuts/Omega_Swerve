package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.Swerve;

public class DriveToPoint extends CommandBase {

    private final Swerve s_Swerve;

    private final double xPose;

    public DriveToPoint(Swerve s_Swerve, double xPose){

        this.s_Swerve = s_Swerve;
        this.xPose = xPose;

        addRequirements(s_Swerve);
    }

    @Override
    public void execute(){



    }

}
