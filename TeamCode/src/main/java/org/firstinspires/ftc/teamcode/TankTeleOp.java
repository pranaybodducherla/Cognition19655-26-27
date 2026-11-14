package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Tank drive for the swerve robot with every module zip-tied pointing straight forward.
 * The steering servos are not used at all.
 *
 * CONTROLS (gamepad1)
 *   left stick up/down     forward / backward
 *   right stick left/right turn
 *   left bumper (hold)     slow mode
 * INTAKE / FLYWHEEL (either gamepad, skipped if not in the robot config)
 *   right trigger          intake in
 *   left trigger           intake out
 *   right bumper           flywheel on/off
 */
@TeleOp(name = "Tank Drive (wheels zip-tied)")
public class TankTeleOp extends LinearOpMode {

    // a = front-left, b = front-right, c = back-left, d = back-right
    private static final String[] LEFT_NAMES  = {"a", "c"};
    private static final String[] RIGHT_NAMES = {"b", "d"};
    // Pushing the stick forward must spin EVERY wheel forward. If one spins backwards,
    // flip its entry here (order matches the names above).
    private static final boolean[] LEFT_REVERSED  = {false, false};
    private static final boolean[] RIGHT_REVERSED = {false, true}; // d was reversed in swervev3

    // 1.0 = 100% motor power, the maximum the hardware allows (there is nothing above 1.0).
    // Motors run RUN_WITHOUT_ENCODER, so the SDK puts no hidden speed cap on them.
    private static final double DRIVE_SPEED = 1.0;
    private static final double TURN_SPEED = 1.0;
    private static final double SLOW = 0.35;

    private static final double INTAKE_POWER = 1.0;
    private static final double FLYWHEEL_POWER = 0.8;

    @Override
    public void runOpMode() {
        DcMotor[] left = new DcMotor[2];
        DcMotor[] right = new DcMotor[2];
        for (int i = 0; i < 2; i++) {
            left[i] = setUp(LEFT_NAMES[i], LEFT_REVERSED[i]);
            right[i] = setUp(RIGHT_NAMES[i], RIGHT_REVERSED[i]);
        }
        DcMotor intake = optional("intake");
        DcMotor flywheel = optional("outtake");

        telemetry.addData("intake", intake == null ? "not in config, skipped" : "ok");
        telemetry.addData("flywheel", flywheel == null ? "not in config, skipped" : "ok");
        telemetry.update();

        waitForStart();

        boolean flywheelOn = false;
        boolean prevBumper = false;

        while (opModeIsActive()) {
            // ---- Drive ----
            double forward = -gamepad1.left_stick_y;  // up = forward
            double turn = gamepad1.right_stick_x;     // right = turn right
            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            double scale = gamepad1.left_bumper ? SLOW : 1;
            double leftPower = forward * DRIVE_SPEED + turn * TURN_SPEED;
            double rightPower = forward * DRIVE_SPEED - turn * TURN_SPEED;
            double biggest = Math.max(1, Math.max(Math.abs(leftPower), Math.abs(rightPower)));
            leftPower = leftPower / biggest * scale;
            rightPower = rightPower / biggest * scale;

            for (DcMotor m : left) m.setPower(leftPower);
            for (DcMotor m : right) m.setPower(rightPower);

            // ---- Intake ----
            double in = Math.max(gamepad1.right_trigger, gamepad2.right_trigger);
            double out = Math.max(gamepad1.left_trigger, gamepad2.left_trigger);
            double intakePower = Range.clip(in - out, -1, 1) * INTAKE_POWER;
            if (intake != null) intake.setPower(intakePower);

            // ---- Flywheel ----
            boolean bumper = gamepad1.right_bumper || gamepad2.right_bumper;
            if (bumper && !prevBumper) flywheelOn = !flywheelOn;
            prevBumper = bumper;
            if (flywheel != null) flywheel.setPower(flywheelOn ? FLYWHEEL_POWER : 0);

            telemetry.addData("left / right", "%.2f / %.2f", leftPower, rightPower);
            telemetry.addData("speed", gamepad1.left_bumper ? "SLOW" : "full");
            telemetry.addData("intake", "%.2f", intakePower);
            telemetry.addData("flywheel", flywheelOn ? "ON" : "off");
            telemetry.update();
        }

        for (DcMotor m : left) m.setPower(0);
        for (DcMotor m : right) m.setPower(0);
        if (intake != null) intake.setPower(0);
        if (flywheel != null) flywheel.setPower(0);
    }

    private DcMotor setUp(String name, boolean reversed) {
        DcMotor m = hardwareMap.get(DcMotor.class, name);
        m.setDirection(reversed ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m.setPower(0);
        return m;
    }

    private DcMotor optional(String name) {
        try {
            DcMotor m = hardwareMap.get(DcMotor.class, name);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
            return m;
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
}
