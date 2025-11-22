package org.firstinspires.ftc.teamcode.Utilities;

public class WaypointFollower {

    private final AutoDriveController driveController;

    private int waypointIndex;

    public WaypointFollower(AutoDriveController driveController) {

        this.driveController = driveController;

    }

    public void resetWaypointFollower() {
        waypointIndex = 0;
    }

    public double[] getWaypointOutputs(OmegaPose2D currentPose, OmegaPose2D[] poseList) {

        double[] outputs = {0, 0, 0};

        if(waypointIndex < poseList.length){

            driveController.setTargetPose(poseList[waypointIndex]);
            driveController.updateCurrentPose(currentPose);
            outputs = driveController.getOutputs();

        }

        double translationError = Math.abs(Math.hypot(currentPose.x() - poseList[waypointIndex].x(), currentPose.y() - poseList[waypointIndex].y()));

        if(translationError < 0.2 && waypointIndex < poseList.length-1) {
            waypointIndex++;
        }

        return outputs;

    }

}
