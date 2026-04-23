package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;

@Autonomous
public class BlueFar extends OpMode {
    private ServoTurret turret = new ServoTurret();
    private hood hoodMech = new hood();
    private flywheel flywheelMech = new flywheel();
    private Follower follower;
    Timer pathTimer, opModeTimer, shootTimer;


    // Motors
    private DcMotor intake;

    // Servos
    private Servo gate;
    private final double GATE_OPEN = 0;
    private final double GATE_CLOSED = 1;

    // Motor powers
    private final double INTAKE_POWER = 1.0;
    private boolean isShooting = false;

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
        turret.update(follower); //? not sure if needs ,0
        // Continuously update flywheel power during shooting
        if (isShooting) {
            updateShooters();
        }
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



    // Helper methods for mechanisms
    private void startIntake() {
        intake.setPower(INTAKE_POWER);
    }

    private void stopIntake() {
        intake.setPower(0);
    }

    private void startShooters() {
        // Use the flywheel mechanism's autoshoot to calculate velocity based on distance from turret
        double goalDistance = turret.getDistanceToGoal();
        double targetVelocity = flywheelMech.autoshoot(goalDistance);
        flywheelMech.shoot(targetVelocity);
    }

    private void updateShooters() {
        // Continuously update the flywheel during shooting using turret's calculated distance
        double goalDistance = turret.getDistanceToGoal();
        double targetVelocity = flywheelMech.autoshoot(goalDistance);
        flywheelMech.shoot(targetVelocity);
    }

    private void stopShooters() {
        flywheelMech.shoot(0);
    }

    private void prepareToShoot() {
        // Use the hood mechanism's autoshoot to calculate hood position based on turret's distance
        double goalDistance = turret.getDistanceToGoal();
        double hoodPosition = hoodMech.autoshoot(goalDistance);
        hoodMech.setPosition(hoodPosition);

        // Gate is already open from the drive, just set shooting flag
        isShooting = true;
        shootTimer.resetTimer();
    }

    private void shoot() {
        // Gate is already open, just run intake to push balls through
        intake.setPower(INTAKE_POWER);
    }

    private void stopShooting() {
        gate.setPosition(GATE_CLOSED);
        stopIntake();
        isShooting = false;
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
//            public void statePathUpdate() {
//                switch (pathState) {
//                    case START:
//                        if (!follower.isBusy()) {
////                    updateShooters();
//
////                    prepareToShoot();
//                            setPathState(BlueSoloClose.PathState.);
//                        }
//                        break;
//
//                    case SHOOT_PRELOAD:
////                shoot();
//                        if (pathTimer.getElapsedTimeSeconds() > 1.5) {
////                    stopShooting();
//                            follower.followPath(driveToSet2, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_TO_SET2);
////                    startIntake();
//                        }
//                        break;
//
//                    case DRIVE_TO_SET2:
//                        if (!follower.isBusy()) {
//                            setPathState(BlueSoloClose.PathState.INTAKE_SET2);
//                        }
//                        break;
//
//                    case INTAKE_SET2:
//                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
//                            // Keep gate closed and intake running during drive
////                    gate.setPosition(GATE_CLOSED);
//                            follower.followPath(driveSet2ToShoot, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_SET2_TO_SHOOT);
//                        }
//                        break;
//
//                    case DRIVE_SET2_TO_SHOOT:
//                        // Run intake for 0.5s during drive, then stop and open gate
//                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
////                    stopIntake();
////                    gate.setPosition(GATE_OPEN);
//                        }
//
//                        if (!follower.isBusy()) {
////                    prepareToShoot();
//                            setPathState(BlueSoloClose.PathState.SHOOT_SET2);
//                        }
//                        break;
//
//                    case SHOOT_SET2:
////                shoot();
//                        if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
////                    stopShooting();
//                            follower.followPath(driveToGate, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_TO_GATE);
////                    startIntake();
//                        }
//                        break;
//
//                    case DRIVE_TO_GATE:
//                        if (!follower.isBusy()){
//                            follower.followPath(adjust,1, true);
//                            setPathState(BlueSoloClose.PathState.ADJUST);
//                        }
//                        break;
//
//                    case ADJUST:
//                        if (pathTimer.getElapsedTimeSeconds() >= 1.5){
//                            // Keep gate closed and intake running during drive
////                    gate.setPosition(GATE_CLOSED);
//                            follower.followPath(driveGateToShoot, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_GATE_TO_SHOOT);
//                        }
//                        break;
//
//
//                    case DRIVE_GATE_TO_SHOOT:
//                        // Run intake for 0.5s during drive, then stop and open gate
//                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
////                    stopIntake();
////                    gate.setPosition(GATE_OPEN);
//                        }
//
//                        if(!follower.isBusy()){
////                    prepareToShoot();
//                            setPathState(BlueSoloClose.PathState.SHOOT_GATE);
//                        }
//                        break;
//
//                    case SHOOT_GATE:
////                shoot();
//                        if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
////                    stopShooting();
//                            count++;
//                            if (count == 3) {
//                                follower.followPath(driveToSet1, true);
//                                setPathState(BlueSoloClose.PathState.DRIVE_TO_SET1);
////                        startIntake();
//                            }
//                            else {
//                                follower.followPath(driveToGate, true);
//                                setPathState(BlueSoloClose.PathState.DRIVE_TO_GATE);
////                        startIntake();
//                            }
//                        }
//                        break;
//
//                    case DRIVE_TO_SET1:
//                        if (!follower.isBusy()) {
//                            setPathState(BlueSoloClose.PathState.INTAKE_SET1);
//                        }
//                        break;
//
//                    case INTAKE_SET1:
//                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
//                            // Keep gate closed and intake running during drive
////                    gate.setPosition(GATE_CLOSED);
//                            follower.followPath(driveSet1ToShoot, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_SET1_TO_SHOOT);
//                        }
//                        break;
//
//                    case DRIVE_SET1_TO_SHOOT:
//                        // Run intake for 0.5s during drive, then stop and open gate
//                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
////                    stopIntake();
////                    gate.setPosition(GATE_OPEN);
//                        }
//
//                        if (!follower.isBusy()) {
////                    prepareToShoot();
//                            setPathState(BlueSoloClose.PathState.SHOOT_SET1);
//                        }
//                        break;
//
//                    case SHOOT_SET1:
////                shoot();
//                        if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
////                    stopShooting();
//                            follower.followPath(driveToEnd, true);
////                    startIntake();
//                            setPathState(BlueSoloClose.PathState.DRIVE_TO_END);
//                        }
//                        break;
//
//                    case DRIVE_TO_SET3:
//                        if (!follower.isBusy()) {
//                            setPathState(BlueSoloClose.PathState.INTAKE_SET3);
//                        }
//                        break;
//
//                    case INTAKE_SET3:
//                        if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
//                            // Keep gate closed and intake running during drive
////                    gate.setPosition(GATE_CLOSED);
//                            follower.followPath(driveSet3ToShoot, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_SET3_TO_SHOOT);
//                        }
//                        break;
//
//                    case DRIVE_SET3_TO_SHOOT:
//                        // Run intake for 0.5s during drive, then stop and open gate
//                        if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
////                    stopIntake();
////                    gate.setPosition(GATE_OPEN);
//                        }
//
//                        if (!follower.isBusy()) {
////                    prepareToShoot();
//                            setPathState(BlueSoloClose.PathState.SHOOT_SET3);
//                        }
//                        break;
//
//                    case SHOOT_SET3:
////                shoot();
//                        if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
////                    stopShooting();
//                            follower.followPath(driveToEnd, true);
//                            setPathState(BlueSoloClose.PathState.DRIVE_TO_END);
//                        }
//                        break;
//
//                    case DRIVE_TO_END:
//                        if (!follower.isBusy()) {
//                            setPathState(BlueSoloClose.PathState.IDLE);
//                        }
//                        break;
//
//                    case IDLE:
//                        break;
//
//                    default:
//                        break;
//                }
//            }

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
