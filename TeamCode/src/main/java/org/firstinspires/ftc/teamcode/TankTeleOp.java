package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Tank drive for the swerve robot with every module zip-tied pointing straight forward.
 * The steering servos are never driven; they're switched fully off at start so they go limp.
 *
 * CONTROLS (gamepad1)
 *   left stick up/down     forward / backward
 *   right stick left/right turn in place (overrides the left stick, no curving);
 *                          the outside side pushes at full power, the inside pulls back at half
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
    private static final double TURN_SPEED = 1.0;   // power on the OUTSIDE of the turn
    private static final double INNER_TURN = 0.5;   // inside of the turn pulls back at this fraction
    // Protects the zip-tied modules and servo gears: power can't jump straight to 100%,
    // it rises over about 1/4 second instead of yanking the modules.
    private static final double RAMP_PER_SECOND = 4.0;
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
        int servosOff = switchServosOff();

        telemetry.addData("steering servos", "%d switched OFF", servosOff);
        telemetry.addData("intake", intake == null ? "not in config, skipped" : "ok");
        telemetry.addData("flywheel", flywheel == null ? "not in config, skipped" : "ok");
        telemetry.update();

        waitForStart();

        boolean flywheelOn = false;
        boolean prevBumper = false;
        double leftOut = 0;
        double rightOut = 0;
        ElapsedTime timer = new ElapsedTime();
        double lastT = timer.seconds();

        while (opModeIsActive()) {
            // ---- Drive ----
            double forward = -gamepad1.left_stick_y;  // up = forward
            double turn = gamepad1.right_stick_x;     // right = turn right
            if (Math.abs(forward) < 0.05) forward = 0;
            if (Math.abs(turn) < 0.05) turn = 0;

            double scale = gamepad1.left_bumper ? SLOW : 1;
            double leftPower;
            double rightPower;
            if (turn > 0) {
                // TURN RIGHT: no curving, forward stick ignored.
                // Left side (outside of the turn) pushes forward hard, right side pulls back less.
                leftPower = turn * TURN_SPEED;
                rightPower = -turn * TURN_SPEED * INNER_TURN;
            } else if (turn < 0) {
                // TURN LEFT: right side (outside of the turn) pushes forward hard, left pulls back less.
                rightPower = -turn * TURN_SPEED;
                leftPower = turn * TURN_SPEED * INNER_TURN;
            } else {
                // STRAIGHT: both sides the same
                leftPower = forward * DRIVE_SPEED;
                rightPower = forward * DRIVE_SPEED;
            }
            leftPower = Range.clip(leftPower, -1, 1) * scale;
            rightPower = Range.clip(rightPower, -1, 1) * scale;

            // Ramp: move toward the wanted power gradually instead of jumping
            double now = timer.seconds();
            double step = RAMP_PER_SECOND * (now - lastT);
            lastT = now;
            leftOut += Range.clip(leftPower - leftOut, -step, step);
            rightOut += Range.clip(rightPower - rightOut, -step, step);

            for (DcMotor m : left) m.setPower(leftOut);
            for (DcMotor m : right) m.setPower(rightOut);

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

            telemetry.addData("left / right", "%.2f / %.2f", leftOut, rightOut);
            telemetry.addData("steering servos", "%d switched OFF", servosOff);
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

    /**
     * The steering servos are NEVER driven. This only makes sure they're completely off
     * (power 0 and signal cut), so they go limp and can't hold or fight the zip-tied modules.
     */
    private int switchServosOff() {
        int count = 0;
        for (String name : new String[] {"as", "bs", "cs", "ds"}) {
            try {
                CRServo s = hardwareMap.get(CRServo.class, name);
                s.setPower(0);
                s.getController().pwmDisable();
                count++;
            } catch (IllegalArgumentException e) {
                // not in the config: nothing to switch off
            }
        }
        return count;
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
