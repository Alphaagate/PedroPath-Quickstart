package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
                        new BezierCurve(
                                new Pose(56.000, 8.000),
                                new Pose(41.359, 40.545),
                                new Pose(15.292, 35.186)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        new BezierLine(
                                new Pose(15.292, 35.186),
                                new Pose(55.913, 7.409)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(55.913, 7.409),
                                new Pose(1.137, 7.794)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(1.137, 7.794),
                                new Pose(55.562, 6.969)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(55.562, 6.969),
                                new Pose(1.122, 7.860)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(1.122, 7.860),
                                new Pose(55.466, 6.942)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(55.466, 6.942),
                                new Pose(1.419, 7.752)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                new Pose(1.419, 7.752),
                                new Pose(55.200, 7.653)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
