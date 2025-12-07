package org.firstinspires.ftc.teamcode.AutoCommands;

public class AutoWaitCommand {

    private int phase;

    private double timestamp;

    private boolean isFinished;

    private double waitTime;

    public AutoWaitCommand(){

        waitTime = 0;

    }

    public void reset(double waitTimeNanoSeconds){
        waitTime = waitTimeNanoSeconds;
    }

    public void execute(){

        switch(phase) {
            case 0:
                    timestamp = System.nanoTime();
                    phase++;
                break;
            case 1:
                if(System.nanoTime() - timestamp > waitTime) {
                    isFinished = true;
                    phase++;
                }

                break;
        }
    }


    public boolean isFinished() {
        return isFinished;
    }

    public boolean runCommand() {
        execute();
        return isFinished();
    }

}
