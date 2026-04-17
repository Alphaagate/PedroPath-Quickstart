package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.ServoTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.flywheel;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.gate;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.hood;
import org.firstinspires.ftc.teamcode.pedroPathing.mechanisms.intake;
@Configurable
@TeleOp(name = "red", group = "TeleOp")
public class red extends OpMode {

    // ============================================================================
    // MECHANISMS
    // ============================================================================

    private Follower follower;
    public double shootvel;
    public static double hoodpos;
    public static double vel;
    private hood hood = new hood();
    private ElapsedTime loopTimer = new ElapsedTime();
    private ServoTurret turret = new ServoTurret();
    private flywheel shooter = new flywheel();
    private boolean shoot = false;
    private intake intake = new intake();
    private gate gate = new gate();
    private DcMotorEx outtake, outtake2;

    // ============================================================================
    // SETTINGS
    // ============================================================================

    private static final double SLOW_MODE_FACTOR = 0.4;

    // ============================================================================
    // OPMODE LIFECYCLE
    // ============================================================================

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        intake.init(hardwareMap);
        gate.init(hardwareMap);
        shooter.init(hardwareMap);
        turret.init(hardwareMap);
        hood.init(hardwareMap);
        outtake = hardwareMap.get(DcMotorEx.class, "o1");
        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
        outtake2 = hardwareMap.get(DcMotorEx.class, "o2");

        outtake2.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        hoodpos = hood.autoshoot(turret.getDistanceToGoal());
        hood.setPosition(hoodpos);
        vel = (int) MathFunctions.clamp(
                -0.00000496881 * Math.pow(turret.getDistanceToGoal(), 4)
                        + 0.00196997 * Math.pow(turret.getDistanceToGoal(), 3)
                        - 0.262396 * Math.pow(turret.getDistanceToGoal(), 2)
                        + 18.24098 * turret.getDistanceToGoal()
                        + 455.32109,
                0,
                1720
        );
        hood.setPosition(hoodpos);

        double loopTime = loopTimer.milliseconds();
        loopTimer.reset();
        turret.update(follower);

        // ====================================================================
        // DRIVING
        // ====================================================================
        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        if (gamepad1.left_trigger > 0.5) {
            forward *= SLOW_MODE_FACTOR;
            strafe  *= SLOW_MODE_FACTOR;
            rotate  *= SLOW_MODE_FACTOR;
        }

        follower.setTeleOpDrive(forward, strafe, rotate, false);
        follower.update();

        // ====================================================================
        // INTAKE
        // ====================================================================
        if (gamepad1.rightBumperWasPressed()) {
            if (shoot) {
                intake.allspin();
            }
            else{
            intake.intakeonly();}
        } else if (gamepad1.dpadDownWasPressed()) {
            intake.reverse();
        } else if (gamepad1.leftBumperWasPressed()){
            intake.allspin();
        }
        else if (gamepad1.rightBumperWasReleased() || gamepad1.dpadDownWasReleased()||gamepad1.leftBumperWasReleased()) {
            intake.stop();
        }


        if (gamepad1.dpadUpWasPressed()){
            shoot = true;
            gate.open();
            shootvel = vel;
        } else if (gamepad1.dpadUpWasReleased()){
            shoot = false;
            gate.close();
            shootvel = 0;
        }
        double velocity = (outtake.getVelocity());
        double error = shootvel - velocity;
        double feedback = error * 0.005;
        double feedforward = 0.00036 * shootvel + 0.08;
        outtake.setPower(feedback + feedforward);
        outtake2.setPower(feedback + feedforward);

        // ====================================================================
        // TURRET
        // ====================================================================
        // Right stick Y on gamepad2 to adjust turret position


        // ====================================================================
        // TELEMETRY
        // ====================================================================
        Pose pose = follower.getPose();

        telemetry.addLine("=== ODOMETRY ===");
        telemetry.addData("X", "%.1f in", pose.getX());
        telemetry.addData("Y", "%.1f in", pose.getY());
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(pose.getHeading()));
        telemetry.addData("Loop Time (ms)", "%.2f", loopTime);
        telemetry.addData("servo pos", turret.getPosition());
        telemetry.addData("dis to goal", turret.getDistanceToGoal());
        telemetry.addData("target vel", shootvel);
        telemetry.addData("realvel", outtake.getVelocity());
        telemetry.update();


    }

    @Override
    public void stop() {}
}