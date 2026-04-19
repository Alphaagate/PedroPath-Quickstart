package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Autonomous
public class RedAuto extends OpMode {
    private Follower follower;

    private flywheel flywheelMech = new flywheel();

    private intake intake = new intake();

    int count = 0;


    PathChain drive1, drive2, drive3, drive4, drive5, drive6, drive7, drive8, drive9, drive10, drive11, drive12;

    Timer pathtimer;

    public enum PathState {
        START,
        DRIVE1,
        DRIVE2,
        DRIVE3,
        DRIVE4,
        DRIVE5,
        DRIVE6,
        DRIVE7,
        DRIVE8,
        DRIVE9,
        DRIVE10,
        DRIVE11,
        IDLE
    }
    PathState pathState;


    @Override
    public void init() {
        pathState = PathState.START;
        pathtimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(new Pose(56, 8, Math.toRadians(180)));
//        intake.init(hardwareMap);
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
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();



        drive2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(15.292, 35.186),
                                new Pose(55.913, 7.409)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive3 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(55.913, 10.409),
                                new Pose(12.137, 10.794)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        drive4 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(12.137, 10.794),
                                new Pose(55.562, 10.969)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();



        drive5 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(55.562, 10.969),
                                new Pose(12.122, 10.860)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive6 = follower.pathBuilder()

                .addPath(
                        new BezierLine(
                                new Pose(12.122, 10.860),
                                new Pose(55.466, 10.942)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive7 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.466, 10.942),
                                new Pose(12.419, 10.752)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive8 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.419, 10.752),
                                new Pose(55.200, 10.653)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive9 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.466, 10.942),
                                new Pose(12.419, 10.752)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive10 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.419, 10.752),
                                new Pose(55.200, 10.653)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive11 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(55.466, 10.942),
                                new Pose(12.419, 10.752)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        drive12 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(12.419, 10.752),
                                new Pose(55.200, 10.653)
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
                    setPathState(PathState.DRIVE4);
                }
                break;

            case DRIVE4:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive5, true);
                    setPathState(PathState.DRIVE5);
                }
                break;

            case DRIVE5:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive6, true);
                    setPathState(PathState.DRIVE6);
                }
                break;
            case DRIVE6:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive7, true);
                    setPathState(PathState.DRIVE7);
                }
                break;
            case DRIVE7:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive8, true);
                    setPathState(PathState.DRIVE8);
                }
                break;
            case DRIVE8:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive9, true);
                    setPathState(PathState.DRIVE9);
                }
                break;
            case DRIVE9:
                if (!follower.isBusy()) {
                    //stuff
                    //setPathState(PathState.SHOOT);
                    follower.followPath(drive10, true);
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
