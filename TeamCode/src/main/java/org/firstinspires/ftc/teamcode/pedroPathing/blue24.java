package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Configurable
@Autonomous
public class blue24 extends OpMode {

    private intake intake = new intake();

    private int count = -1;

    private Follower follower;
    Timer pathTimer, opModeTimer, shootTimer;

    // Servo positions
    private final double GATE_OPEN = 0;
    private final double GATE_CLOSED = 1;

    // Motor powers
    private final double INTAKE_POWER = 1.0;

    private boolean isShooting = false;

    public enum PathState {
        DRIVE_START_TO_SHOOT,
        SHOOT_PRELOAD,
        DRIVE_TO_SET2,
        INTAKE_SET2,
        DRIVE_SET2_TO_SHOOT,
        SHOOT_SET2,
        DRIVE_TO_GATE,
        ADJUST,
        ADJUST2,
        ADJUST3,
        DRIVE_GATE_TO_SHOOT,
        SHOOT_GATE,
        DRIVE_TO_SET1,
        INTAKE_SET1,
        DRIVE_SET1_TO_SHOOT,
        SHOOT_SET1,
        DRIVE_TO_SET3,
        INTAKE_SET3,
        DRIVE_SET3_TO_SHOOT,
        SHOOT_SET3,
        DRIVE_TO_END,
        IDLE
    }

    PathState pathState;

    private PathChain driveStartToShoot, driveToSet1, driveSet1ToShoot, driveToGate, adjust, adjust2, adjust3,
            driveGateToShoot, driveToSet2, driveSet2ToShoot, driveToSet3, driveSet3ToShoot, driveToEnd;

    private final double SHOOT_TIME = 0.8;
    private final double INTAKE_TIME = 0;
    private final double INTAKE_GATE_TIME = 1.5;

    public void buildPaths() {
        // RED SIDE PATHS
        // Mirror formula: X_red = 144 - X_blue, Y unchanged, heading_red = 180° - heading_blue

        // Path 1: Start to shoot position
        // Blue: (29.949, 132.454, 145°) -> (55.884, 93, 180°)
        // Red:  (114.051, 132.454, 35°) -> (88.116, 93, 0°)
        driveStartToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(114.051, 132.454, Math.toRadians(35)),
                        new Pose(88.116, 93, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(35), Math.toRadians(0))
                .build();

        // Blue: (55, 90, 180°) -> (62, 54.832, 180°) -> (24.5, 61, 180°)
        // Red:  (89, 90, 0°)   -> (82, 54.832, 0°)   -> (119.5, 61, 0°)
        driveToSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(89, 90, Math.toRadians(0)),
                        new Pose(82, 54.832, Math.toRadians(0)),
                        new Pose(119.5, 61, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (24.5, 61, 180°) -> (60.884, 84.526, 180°)
        // Red:  (119.5, 61, 0°)  -> (83.116, 84.526, 0°)
        driveSet2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(119.5, 61, Math.toRadians(0)),
                        new Pose(83.116, 84.526, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (60.884, 84.526, 180°) -> (59.385, 72.917, 180°) -> (22, 65.25, 180°)
        // Red:  (83.116, 84.526, 0°)   -> (84.615, 72.917, 0°)   -> (122, 65.25, 0°)
        driveToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.116, 84.526, Math.toRadians(0)),
                        new Pose(84.615, 72.917, Math.toRadians(0)),
                        new Pose(122, 65.25, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (22, 65.25, 180°) -> (16.7, 63.55, 148°)
        // Red:  (122, 65.25, 0°)  -> (127.3, 63.55, 32°)
        adjust = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(122, 65.25, Math.toRadians(0)),
                        new Pose(127.3, 63.55, Math.toRadians(32))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32))
                .build();

        // Blue: (16.7, 63.55, 180°) -> (57.359, 64.970, 180°) -> (60.884, 84.526, 180°)
        // Red:  (127.3, 63.55, 0°)  -> (86.641, 64.970, 0°)   -> (83.116, 84.526, 0°)
        driveGateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(127.3, 63.55, Math.toRadians(0)),
                        new Pose(86.641, 64.970, Math.toRadians(0)),
                        new Pose(83.116, 84.526, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (60.884, 84.526, 180°) -> (26.5, 84.591, 180°)
        // Red:  (83.116, 84.526, 0°)   -> (117.5, 84.591, 0°)
        driveToSet1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(83.116, 84.526, Math.toRadians(0)),
                        new Pose(117.5, 84.591, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (26.5, 84.591, 180°) -> (60.884, 84.526, 180°)
        // Red:  (117.5, 84.591, 0°)  -> (83.116, 84.526, 0°)
        driveSet1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(117.5, 84.591, Math.toRadians(0)),
                        new Pose(83.116, 84.526, Math.toRadians(0))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Blue: (60.884, 84.526, 180°) -> (75.928, 30.415, 180°) -> (23.5, 36, 180°)
        // Red:  (83.116, 84.526, 0°)   -> (68.072, 30.415, 0°)   -> (120.5, 36, 0°)
        driveToSet3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(83.116, 84.526, Math.toRadians(0)),
                        new Pose(68.072, 30.415, Math.toRadians(0)),
                        new Pose(120.5, 36, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (23.5, 36, 180°) -> (54.884, 120, 180°)
        // Red:  (120.5, 36, 0°)  -> (89.116, 120, 0°)
        driveSet3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(120.5, 36, Math.toRadians(0)),
                        new Pose(89.116, 120, Math.toRadians(0))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // Blue: (54.884, 120, 180°) -> (54, 120, 180°)
        // Red:  (89.116, 120, 0°)   -> (90, 120, 0°)
        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(89.116, 120, Math.toRadians(0)),
                        new Pose(90, 120, Math.toRadians(0))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_START_TO_SHOOT:
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    follower.followPath(driveToSet2, true);
                    setPathState(PathState.DRIVE_TO_SET2);
                }
                break;

            case DRIVE_TO_SET2:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET2);
                }
                break;

            case INTAKE_SET2:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    follower.followPath(driveSet2ToShoot, true);
                    setPathState(PathState.DRIVE_SET2_TO_SHOOT);
                }
                break;

            case DRIVE_SET2_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET2);
                }
                break;

            case SHOOT_SET2:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    follower.followPath(driveToGate, true);
                    setPathState(PathState.DRIVE_TO_GATE);
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy()) {
                    follower.followPath(adjust, 1, true);
                    setPathState(PathState.ADJUST);
                }
                break;

            case ADJUST:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    follower.followPath(driveGateToShoot, true);
                    setPathState(PathState.DRIVE_GATE_TO_SHOOT);
                }
                break;

            case DRIVE_GATE_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_GATE);
                }
                break;

            case SHOOT_GATE:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    count++;
                    if (count == 2) {
                        follower.followPath(driveToSet1, true);
                        setPathState(PathState.DRIVE_TO_SET1);
                    } else {
                        follower.followPath(driveToGate, true);
                        setPathState(PathState.DRIVE_TO_GATE);
                    }
                }
                break;

            case DRIVE_TO_SET1:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET1);
                }
                break;

            case INTAKE_SET1:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    follower.followPath(driveSet1ToShoot, true);
                    setPathState(PathState.DRIVE_SET1_TO_SHOOT);
                }
                break;

            case DRIVE_SET1_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET1);
                }
                break;

            case SHOOT_SET1:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    follower.followPath(driveToSet3, true);
                    setPathState(PathState.DRIVE_TO_SET3);
                }
                break;

            case DRIVE_TO_SET3:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET3);
                }
                break;

            case INTAKE_SET3:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    follower.followPath(driveSet3ToShoot, true);
                    setPathState(PathState.DRIVE_SET3_TO_SHOOT);
                }
                break;

            case DRIVE_SET3_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.SHOOT_SET3);
                }
                break;

            case SHOOT_SET3:
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
                    follower.followPath(driveToEnd, true);
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;

            case DRIVE_TO_END:
                if (!follower.isBusy()) {
                    setPathState(PathState.IDLE);
                }
                break;

            case IDLE:
                break;

            default:
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        intake.init(hardwareMap);
        pathState = PathState.DRIVE_START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        // Red starting position
        follower.setPose(new Pose(114.051, 132.454, Math.toRadians(35)));

        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
    }

    public void start() {
        opModeTimer.resetTimer();
        follower.followPath(driveStartToShoot, true);
        setPathState(PathState.DRIVE_START_TO_SHOOT);
    }

    @Override
    public void loop() {
        intake.intakeonly();
        follower.update();
        statePathUpdate();

        telemetry.addData("State", pathState);
        telemetry.addData("Elapsed Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("State Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shoot Timer", shootTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}