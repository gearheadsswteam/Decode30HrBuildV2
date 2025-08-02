
package org.firstinspires.ftc.teamcode.fsm.commands;

//commands;


public class TrajectoryCommand extends Command {
    @Override
    public void init() {

    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end() {

    }
//    private final SampleMecanumDrive drive;
//    private final Trajectory trajectory;
//    private boolean started = false;
//
//    public TrajectoryCommand(SampleMecanumDrive drive, Trajectory trajectory) {
//        this.drive = drive;
//        this.trajectory = trajectory;
//    }
//
//    @Override
//    public void init() {
//        drive.followTrajectoryAsync(trajectory);
//        started = true;
//        System.out.println("[FSM] STARTED TrajectoryCommand");
//    }
//
//    @Override
//    public void execute() {
//        if (started) {
//            drive.update();
//            System.out.println("[FSM] EXECUTING TrajectoryCommand");
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return started && !drive.isBusy();
//    }
//
//    @Override
//    public void end() {
//        System.out.println("[FSM] ENDED TrajectoryCommand");
//    }
}
