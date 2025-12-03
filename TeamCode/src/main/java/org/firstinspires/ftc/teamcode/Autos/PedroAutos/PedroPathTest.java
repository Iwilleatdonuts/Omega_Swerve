//package org.firstinspires.ftc.teamcode.Autos.PedroAutos;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name = "PEDRO TEST")
//public class PedroPathTest extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
////        Follower follower = ;
//        Timer pathTimer, actionTimer, opmodeTimer;
//        int pathState;
//
//
//        Pose startPose = new Pose(72, 0, Math.toRadians(0));
//        Pose scorePose = new Pose(72, 72, Math.toRadians(180));
//
//
//        PathChain path1;
//
//        path1 = follower.pathBuilder()
//                .addPath(new BezierLine(startPose, scorePose))
//                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
//                        .build();
//
//        waitForStart();
//        while(opModeIsActive()) {
//
//            follower.followPath(path1);
//
//        }
//
//    }
//}
