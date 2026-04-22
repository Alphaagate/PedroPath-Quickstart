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
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.TurretMechanism4;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class BlueSoloClose extends OpMode {

    private ServoTurret turret = new ServoTurret();
    private hood hoodMech = new hood();
    private flywheel flywheelMech = new flywheel();
    private int count = 0;



    private Follower follower;
    Timer pathTimer, opModeTimer, shootTimer;

    // Motors
    private DcMotor intake;

    // Servos
    private Servo gate;

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

    // Generated paths from visualizer
    private PathChain driveStartToShoot, driveToSet1, driveSet1ToShoot, driveToGate, adjust, adjust2, adjust3, driveGateToShoot, driveToSet2, driveSet2ToShoot, driveToSet3 ,driveSet3ToShoot, driveToEnd;

    private final double SHOOT_TIME = 1.6;
    private final double INTAKE_TIME = 0;
    private final double INTAKE_GATE_TIME = 1.5;


    public void buildPaths() {
        // BLUE SIDE MIRRORED PATHS
        // Y-axis mirror: X_blue = 144 - X_red, Y stays same, heading_blue = 180° - heading_red

        // Path 1: Start to shoot position
        // Red: (116.551, 132.454, 35°) -> (85.616, 88.526, 35°)
        // Blue: (29.949, 132.454, 145°) -> (60.884, 88.526, 145°)
        driveStartToShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(22.550, 120.448),
                                new Pose(57.423, 75.143)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                .build();

        // Red: (85.616, 88.526, 35°) -> (84.359, 54.832, 0°) -> (120, 57.5, 0°)
        // Blue: (60.884, 88.526, 145°) -> (62.141, 54.832, 180°) -> (26.5, 57.5, 180°)
        driveToSet2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(55, 90, Math.toRadians(180)),
                        new Pose(62, 54.832, Math.toRadians(180)),
                        new Pose(24.5, 61, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Red: (120, 57.5, 0°) -> (85.616, 84.526, 0°)
        // Blue: (26.5, 57.5, 180°) -> (60.884, 84.526, 180°)
        driveSet2ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(24.5, 61, Math.toRadians(180)),
                        new Pose(60.884, 84.526, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Red: (85.616, 84.526, 0°) -> (121.5, 64, 0°)
        // Blue: (60.884, 84.526, 180°) -> (25, 64, 180°)
        driveToGate = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(60.884, 84.526, Math.toRadians(180)),
                        new Pose(59.38451134644478, 72.91655521936462, Math.toRadians(180)),
                        new Pose(22, 65.25, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Red: (121.5, 64, 0°) -> (125, 63, 32°)
        // Blue: (25, 64, 180°) -> (21.5, 63, 148°)
        // Note: 180° - 32° = 148°
        adjust = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22, 65.25, Math.toRadians(180)),
                        new Pose(16.7, 63.55, Math.toRadians(148))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(148))
                .build();

        // Red: (125.522, 63, 0°) -> (89.141, 64.970, 0°) -> (85.616, 84.526, 0°)
        // Blue: (20.978, 63, 180°) -> (57.359, 64.970, 180°) -> (60.884, 84.526, 180°)
        driveGateToShoot = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(16.7, 63.55, Math.toRadians(180)),
                        new Pose(57.35903782148259, 64.97026172465962, Math.toRadians(180)),
                        new Pose(60.884, 84.526, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 4: Shoot to set 1 - NOW SECOND
        // Red: (85.616, 84.526, 0°) -> (120, 84.091, 0°)
        // Blue: (60.884, 84.526, 180°) -> (26.5, 84.091, 180°)
        driveToSet1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(60.884, 84.526, Math.toRadians(180)),
                        new Pose(26.5, 84.591, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 5: Set 1 back to shoot (reversed)
        // Red: (120, 84.091, 0°) -> (85.616, 110, 0°)
        // Blue: (26.5, 84.091, 180°) -> (60.884, 110, 180°)
        driveSet1ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(26.5, 84.591, Math.toRadians(180)),
                        new Pose(60.884, 84.526, Math.toRadians(180))
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        // Red: (85.616, 84.526, 0°) -> (70.572, 30.415, 0°) -> (121, 36, 0°)
        // Blue: (60.884, 84.526, 180°) -> (75.928, 30.415, 180°) -> (25.5, 36, 180°)
        driveToSet3 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(60.884, 84.526, Math.toRadians(180)),
                        new Pose(75.928, 30.415, Math.toRadians(180)),
                        new Pose(23.5, 36, Math.toRadians(180))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Red: (121, 36, 0°) -> (85.616, 100.526, 0°)
        // Blue: (25.5, 36, 180°) -> (60.884, 100.526, 180°)
        driveSet3ToShoot = follower.pathBuilder()
                .addPath((new BezierLine(
                        new Pose(23.5, 36, Math.toRadians(180)),
                        new Pose(54.884, 120, Math.toRadians(180)))
                ))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // Path 6: Shoot to end position (park)
        // Red: (85.616, 84.526, 0°) -> (86.554, 120, 0°)
        // Blue: (60.884, 84.526, 180°) -> (59.946, 120, 180°)
        driveToEnd = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(54.884, 120, Math.toRadians(180)),
                        new Pose(54, 120, Math.toRadians(180))
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
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
            case DRIVE_START_TO_SHOOT:
                if (!follower.isBusy()) {
//                    updateShooters();

//                    prepareToShoot();
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;

            case SHOOT_PRELOAD:
//                shoot();
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
//                    stopShooting();
                    follower.followPath(driveToSet2, true);
                    setPathState(PathState.DRIVE_TO_SET2);
//                    startIntake();
                }
                break;

            case DRIVE_TO_SET2:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET2);
                }
                break;

            case INTAKE_SET2:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    // Keep gate closed and intake running during drive
//                    gate.setPosition(GATE_CLOSED);
                    follower.followPath(driveSet2ToShoot, true);
                    setPathState(PathState.DRIVE_SET2_TO_SHOOT);
                }
                break;

            case DRIVE_SET2_TO_SHOOT:
                // Run intake for 0.5s during drive, then stop and open gate
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
//                    stopIntake();
//                    gate.setPosition(GATE_OPEN);
                }

                if (!follower.isBusy()) {
//                    prepareToShoot();
                    setPathState(PathState.SHOOT_SET2);
                }
                break;

            case SHOOT_SET2:
//                shoot();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
//                    stopShooting();
                    follower.followPath(driveToGate, true);
                    setPathState(PathState.DRIVE_TO_GATE);
//                    startIntake();
                }
                break;

            case DRIVE_TO_GATE:
                if (!follower.isBusy()){
                    follower.followPath(adjust,1, true);
                    setPathState(PathState.ADJUST);
                }
                break;

            case ADJUST:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5){
                    // Keep gate closed and intake running during drive
//                    gate.setPosition(GATE_CLOSED);
                    follower.followPath(driveGateToShoot, true);
                    setPathState(PathState.DRIVE_GATE_TO_SHOOT);
                }
                break;


            case DRIVE_GATE_TO_SHOOT:
                // Run intake for 0.5s during drive, then stop and open gate
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
//                    stopIntake();
//                    gate.setPosition(GATE_OPEN);
                }

                if(!follower.isBusy()){
//                    prepareToShoot();
                    setPathState(PathState.SHOOT_GATE);
                }
                break;

            case SHOOT_GATE:
//                shoot();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
//                    stopShooting();
                    count++;
                    if (count == 3) {
                        follower.followPath(driveToSet1, true);
                        setPathState(PathState.DRIVE_TO_SET1);
//                        startIntake();
                    }
                    else {
                        follower.followPath(driveToGate, true);
                        setPathState(PathState.DRIVE_TO_GATE);
//                        startIntake();
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
                    // Keep gate closed and intake running during drive
//                    gate.setPosition(GATE_CLOSED);
                    follower.followPath(driveSet1ToShoot, true);
                    setPathState(PathState.DRIVE_SET1_TO_SHOOT);
                }
                break;

            case DRIVE_SET1_TO_SHOOT:
                // Run intake for 0.5s during drive, then stop and open gate
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
//                    stopIntake();
//                    gate.setPosition(GATE_OPEN);
                }

                if (!follower.isBusy()) {
//                    prepareToShoot();
                    setPathState(PathState.SHOOT_SET1);
                }
                break;

            case SHOOT_SET1:
//                shoot();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
//                    stopShooting();
                    follower.followPath(driveToEnd, true);
//                    startIntake();
                    setPathState(PathState.DRIVE_TO_END);
                }
                break;

            case DRIVE_TO_SET3:
                if (!follower.isBusy()) {
                    setPathState(PathState.INTAKE_SET3);
                }
                break;

            case INTAKE_SET3:
                if (pathTimer.getElapsedTimeSeconds() > INTAKE_TIME) {
                    // Keep gate closed and intake running during drive
//                    gate.setPosition(GATE_CLOSED);
                    follower.followPath(driveSet3ToShoot, true);
                    setPathState(PathState.DRIVE_SET3_TO_SHOOT);
                }
                break;

            case DRIVE_SET3_TO_SHOOT:
                // Run intake for 0.5s during drive, then stop and open gate
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
//                    stopIntake();
//                    gate.setPosition(GATE_OPEN);
                }

                if (!follower.isBusy()) {
//                    prepareToShoot();
                    setPathState(PathState.SHOOT_SET3);
                }
                break;

            case SHOOT_SET3:
//                shoot();
                if (pathTimer.getElapsedTimeSeconds() > SHOOT_TIME) {
//                    stopShooting();
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
        pathState = PathState.DRIVE_START_TO_SHOOT;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        shootTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        // Initialize mechanisms
        turret.init(hardwareMap);
        hoodMech.init(hardwareMap);
        flywheelMech.init(hardwareMap);

        // Initialize motors
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//         Initialize servos
        gate = hardwareMap.get(Servo.class, "gate");
        gate.setDirection(Servo.Direction.REVERSE);

        // Set initial servo positions
        gate.setPosition(GATE_OPEN);

        buildPaths();
        // Blue starting position: mirrored X and heading (with 2.5 offset)
        follower.setPose(new Pose(29.949, 132.454, Math.toRadians(145)));

        telemetry.addLine("Initialized - Ready!");
        telemetry.update();
    }

    public void start() {
        opModeTimer.resetTimer();
        startShooters(); // Start shooters using flywheel mechanism
        follower.followPath(driveStartToShoot, true);
        setPathState(PathState.DRIVE_START_TO_SHOOT);
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

        // Telemetry
        telemetry.addData("State", pathState);
        telemetry.addData("Elapsed Time", opModeTimer.getElapsedTimeSeconds());
        telemetry.addData("State Timer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Shoot Timer", shootTimer.getElapsedTimeSeconds());
        telemetry.addData("Follower Busy", follower.isBusy());
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
//        telemetry.addData("Turret Position", turret.getCurrentPosition());
//        telemetry.addData("Turret Target", turret.getTargetPosition());
//        telemetry.addData("Distance to Goal", turret.getDistanceToGoal());
//        telemetry.addData("Hood Position", hoodMech.getPosition());

        telemetry.update();
    }
}