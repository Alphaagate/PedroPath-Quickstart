package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous
public class RedAuto extends OpMode {
    private Follower follower;

    PathChain drive1, drive2, drive3, drive4;

    Timer pathtimer;

    public enum PathState {
        START,
        DRIVE1,
        DRIVE2,
        DRIVE3,
        IDLE
    }
    PathState pathState;


    @Override
    public void init() {
        pathState = PathState.START;
        pathtimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(new Pose(47, 50, Math.toRadians(90)));
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
    }
    public void start() {
        follower.followPath(drive1, true);
        setPathState(PathState.START);
    }
    public void buildPaths(){
        drive1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(47.528, 50.193),
                                new Pose(47.286, 94.304)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();


        drive2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(47.286, 94.304),
                                new Pose(93.684, 94.042)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
        drive3 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(93.684, 94.042),
                                new Pose(93.217, 47.577)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        drive4 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(93.217, 47.577),
                                new Pose(48.555, 48.262)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // use                         new Pose(58.384, 84.526, Math.toRadians(145)) as start point for next path
    }

    public void statePathUpdate() {
        switch (pathState) {
            case START:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                     follower.followPath(drive1, true);
                     setPathState(PathState.DRIVE1);
                }
                break;
            case DRIVE1:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive2, true);
                    setPathState(PathState.DRIVE2);
                }
                break;

            case DRIVE2:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive3, true);
                    setPathState(PathState.DRIVE3);
                }
                break;

            case DRIVE3:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive4, true);
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;
//            if (pathtimer.getElapsedTimeSeconds() > SHOOT_TIME) {
//
//            }
//


            default:
                break;
        }
    }
    public void  setPathState(PathState newState) {
        pathState = newState;
        pathtimer.resetTimer();
    }



}
