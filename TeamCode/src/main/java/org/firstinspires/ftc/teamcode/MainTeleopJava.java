package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Swerve drive + intake + outtake, all in one OpMode.
 *
 * CONTROLS
 *   DRIVING (gamepad1)
 *     left stick           move
 *     right stick          turn
 *     left bumper (hold)   slow mode for lining up a shot
 *     X (hold)             X-lock: wheels form an X so you can't be pushed
 *     Y                    toggle field-centric / robot-centric (needs IMU)
 *     back                 reset heading (point the robot away from you first)
 *   SCORING (either gamepad, so one or two drivers both work)
 *     right trigger        intake in (harder = faster)
 *     left trigger         intake out
 *     right bumper         flywheel on/off (gamepads rumble when it's up to speed)
 *     dpad up / down       flywheel faster / slower (100 RPM per press)
 *     B (hold)             unjam: run intake AND flywheel backwards
 *
 * The intake and flywheel motors are on the Expansion Hub: name them "intake" and "outtake"
 * in the robot configuration. Bulk reads cover both hubs automatically.
 *
 * Module angles use the same convention as swervev3, so its offsets carry over:
 *   encoder angle = V / maxV * 360 - 180 + offset, wrapped to [-180, 180)
 *   wheel pointing forward = -90, right = 0, backward = +90, left = 180
 *   positive steering power makes the angle go up
 */
@TeleOp(name = "Main TeleOp (Swerve + Intake/Outtake)")
public class MainTeleOp extends LinearOpMode {

    // ===================== HARDWARE NAMES =====================
    private static final String[] STEER_NAMES = {"as", "bs", "cs", "ds"};
    private static final String[] DRIVE_NAMES = {"a", "b", "c", "d"};
    private static final String[] ENC_NAMES   = {"aa", "ba", "ca", "da"};
    private static final String INTAKE_NAME  = "intake";   // DcMotor, optional
    private static final String OUTTAKE_NAME = "outtake";  // DcMotorEx flywheel, optional
    private static final String IMU_NAME     = "imu";      // optional

    // ===================== MODULE SETUP (from swervev3) =====================
    private static final double[]  OFFSETS        = {50, 70, 10, 70};
    private static final boolean[] DRIVE_REVERSED = {false, false, false, true};
    private static final boolean[] STEER_REVERSED = {false, false, false, false};
    // Wheel position from robot center, +X = right, +Y = forward (only ratios matter).
    // ASSUMED: a = front-left, b = front-right, c = back-left, d = back-right.
    private static final double[] MODULE_X = {-6,  6, -6,  6};
    private static final double[] MODULE_Y = { 6,  6, -6, -6};

    // ===================== STEERING: exponential dampener =====================
    // power = MIN + (MAX - MIN) * (1 - e^(-(error - deadband) / EASE_ANGLE))
    private static final double MAX_STEER = 0.5;       // power when far from target
    private static final double MIN_STEER = 0.08;      // power right before stopping
    private static final double EASE_ANGLE = 20;       // bigger = slows down earlier
    private static final double STEER_DEADBAND = 3;    // degrees: close enough
    private static final double FLIP_HYST = 10;        // degrees past 90 before flipping
    private static final double ALIGN_TOLERANCE = 15;  // no wheel drives until ALL are this close

    // ===================== DRIVING =====================
    private static final double DRIVE_SPEED = 0.8;     // max translation power
    private static final double TURN_SPEED = 0.6;      // max rotation power
    private static final double SLOW_MODE = 0.35;      // speed multiplier while left bumper held
    private static final double DRIVE_RAMP = 3.0;      // max drive power change per second
    private static final double MIN_MODULE_SPEED = 0.02;
    private static final double DEADBAND_ON = 0.12;    // stick: start moving above this
    private static final double DEADBAND_OFF = 0.08;   // stick: stop moving below this

    // ===================== INTAKE (rubber-band roller, Expansion Hub) =====================
    private static final double INTAKE_POWER = 1.0;
    private static final double TRIGGER_DEADBAND = 0.05;
    private static final boolean INTAKE_REVERSED = false;   // flip if triggers run it backwards

    // ===================== OUTTAKE (flywheel, Expansion Hub) =====================
    // Velocity control holds the flywheel at a set RPM so every shot is the same.
    // Needs the flywheel motor's encoder cable plugged in. If it isn't, set this false.
    private static final boolean FLYWHEEL_VELOCITY_CONTROL = true;
    private static final double FLYWHEEL_TICKS_PER_REV = 28;   // goBILDA bare motor / 1:1 = 28
    private static final double FLYWHEEL_RPM_DEFAULT = 3500;   // tune for your goal distance
    private static final double FLYWHEEL_RPM_STEP = 100;       // dpad up/down changes it by this
    private static final double FLYWHEEL_RPM_MAX = 6000;
    private static final double FLYWHEEL_READY_TOLERANCE = 0.05; // within 5% = ready to shoot
    private static final double OUTTAKE_POWER = 0.8;           // used only without velocity control
    private static final double OUTTAKE_REVERSE_POWER = 0.5;
    private static final boolean OUTTAKE_REVERSED = false;     // flip if it spins the wrong way

    // ===================== IMU =====================
    private static final RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection IMU_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
    private static final boolean FIELD_CENTRIC_DEFAULT = false; // turn on with Y once IMU is checked

    // ===================== STATE =====================
    private final CRServo[] steer = new CRServo[4];
    private final DcMotor[] drive = new DcMotor[4];
    private final AnalogInput[] enc = new AnalogInput[4];
    private DcMotor intake;
    private DcMotorEx outtake;
    private IMU imu;

    private final double[] angle = new double[4];       // current module angle
    private final double[] target = new double[4];      // wanted module angle
    private final double[] err = new double[4];         // steering error after flip
    private final double[] wantedSpeed = new double[4]; // 0..1
    private final double[] drivePower = new double[4];  // ramped output
    private final boolean[] flipped = new boolean[4];   // aiming backwards, driving reversed

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        for (int i = 0; i < 4; i++) {
            steer[i] = hardwareMap.get(CRServo.class, STEER_NAMES[i]);
            drive[i] = hardwareMap.get(DcMotor.class, DRIVE_NAMES[i]);
            enc[i]   = hardwareMap.get(AnalogInput.class, ENC_NAMES[i]);
            steer[i].setDirection(dir(STEER_REVERSED[i]));
            drive[i].setDirection(dir(DRIVE_REVERSED[i]));
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            steer[i].setPower(0);
            drive[i].setPower(0);
        }

        intake = optionalMotor(INTAKE_NAME);
        if (intake != null) {
            intake.setDirection(dir(INTAKE_REVERSED));
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(0);
        }
        outtake = optionalFlywheel(OUTTAKE_NAME);
        if (outtake != null) {
            outtake.setDirection(dir(OUTTAKE_REVERSED));
            outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // coast down, don't slam
            if (FLYWHEEL_VELOCITY_CONTROL) {
                outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            outtake.setPower(0);
        }
        imu = initImu();

        double rMax = 0;
        for (int i = 0; i < 4; i++) rMax = Math.max(rMax, Math.hypot(MODULE_X[i], MODULE_Y[i]));
        if (rMax < 1e-6) rMax = 1;

        if (intake == null) telemetry.addLine("No '" + INTAKE_NAME + "' motor: intake disabled");
        if (outtake == null) telemetry.addLine("No '" + OUTTAKE_NAME + "' motor: outtake disabled");
        if (imu == null) telemetry.addLine("No IMU: robot-centric only");
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        boolean fieldCentric = FIELD_CENTRIC_DEFAULT && imu != null;
        boolean active = false;
        boolean outtakeOn = false;
        boolean prevY = false;
        boolean prevBumper = false;
        boolean prevUp = false;
        boolean prevDown = false;
        boolean wasReady = false;
        double flywheelRpm = FLYWHEEL_RPM_DEFAULT;
        boolean firstLoop = true;
        double[] moduleTarget = new double[4];
        double[] moduleSpeed = new double[4];

        ElapsedTime timer = new ElapsedTime();
        double lastT = timer.seconds();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            double now = timer.seconds();
            double dt = Math.max(1e-3, now - lastT);
            lastT = now;

            for (int i = 0; i < 4; i++) {
                angle[i] = readAngle(i);
                if (firstLoop) target[i] = angle[i]; // start by holding where the wheels already are
            }
            firstLoop = false;

            // ---------------- Drive buttons ----------------
            if (imu != null && gamepad1.back) imu.resetYaw();
            if (imu != null && gamepad1.y && !prevY) fieldCentric = !fieldCentric;
            prevY = gamepad1.y;

            // ---------------- Sticks (standard frame: +x right, +y forward, +turn CCW) ----------------
            double sx = gamepad1.left_stick_x;
            double sy = -gamepad1.left_stick_y;
            double turn = -gamepad1.right_stick_x;
            double mag = Math.hypot(sx, sy);
            if (mag > 1) {
                sx /= mag;
                sy /= mag;
                mag = 1;
            }
            double input = Math.max(mag, Math.abs(turn));
            active = input > (active ? DEADBAND_OFF : DEADBAND_ON);
            if (mag < DEADBAND_OFF) {
                sx = 0;
                sy = 0;
            }
            if (Math.abs(turn) < DEADBAND_OFF) turn = 0;

            double heading = imu != null
                    ? imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) : 0;
            double vx = sx;
            double vy = sy;
            if (fieldCentric) {
                double c = Math.cos(heading);
                double s = Math.sin(heading);
                vx = sx * c + sy * s;
                vy = -sx * s + sy * c;
            }
            double slow = gamepad1.left_bumper ? SLOW_MODE : 1;
            vx *= DRIVE_SPEED * slow;
            vy *= DRIVE_SPEED * slow;
            double omega = turn * TURN_SPEED * slow / rMax;

            // ---------------- Kinematics ----------------
            double maxSpeed = 1;
            for (int i = 0; i < 4; i++) {
                double mvx = vx - omega * MODULE_Y[i];
                double mvy = vy + omega * MODULE_X[i];
                moduleSpeed[i] = Math.hypot(mvx, mvy);
                moduleTarget[i] = toModuleAngle(Math.toDegrees(Math.atan2(mvy, mvx)));
                maxSpeed = Math.max(maxSpeed, moduleSpeed[i]);
            }

            // ---------------- Steer, then drive all wheels together ----------------
            double scale = 0;
            boolean aligned = true;
            boolean xLock = gamepad1.x;
            if (xLock) {
                for (int i = 0; i < 4; i++) {
                    target[i] = toModuleAngle(Math.toDegrees(Math.atan2(MODULE_Y[i], MODULE_X[i])));
                    wantedSpeed[i] = 0;
                    steerModule(i);
                }
            } else if (active) {
                double worst = 0;
                for (int i = 0; i < 4; i++) {
                    double speed = moduleSpeed[i] / maxSpeed;
                    if (speed >= MIN_MODULE_SPEED) {
                        target[i] = moduleTarget[i];
                        wantedSpeed[i] = speed;
                    } else {
                        wantedSpeed[i] = 0; // no real direction: keep the old heading
                    }
                    steerModule(i);
                    worst = Math.max(worst, Math.abs(err[i]));
                }
                aligned = worst <= ALIGN_TOLERANCE;
                scale = aligned ? Math.cos(Math.toRadians(worst)) : 0;
            } else {
                for (int i = 0; i < 4; i++) {
                    wantedSpeed[i] = 0;
                    steer[i].setPower(0);
                }
            }

            for (int i = 0; i < 4; i++) {
                double want = (flipped[i] ? -1 : 1) * wantedSpeed[i] * scale;
                double step = DRIVE_RAMP * dt;
                drivePower[i] += Range.clip(want - drivePower[i], -step, step);
                drive[i].setPower(drivePower[i]);
            }

            // ---------------- Intake ----------------
            boolean reverse = gamepad1.b || gamepad2.b; // unjam everything
            double intakeCmd = stronger(
                    gamepad1.right_trigger - gamepad1.left_trigger,
                    gamepad2.right_trigger - gamepad2.left_trigger);
            if (Math.abs(intakeCmd) < TRIGGER_DEADBAND) intakeCmd = 0;
            if (reverse) intakeCmd = -1;
            double intakePower = Range.clip(intakeCmd, -1, 1) * INTAKE_POWER;
            if (intake != null) intake.setPower(intakePower);

            // ---------------- Outtake (flywheel) ----------------
            boolean bumper = gamepad1.right_bumper || gamepad2.right_bumper;
            if (bumper && !prevBumper) outtakeOn = !outtakeOn;
            prevBumper = bumper;

            boolean up = gamepad1.dpad_up || gamepad2.dpad_up;
            boolean down = gamepad1.dpad_down || gamepad2.dpad_down;
            if (up && !prevUp) flywheelRpm = Math.min(FLYWHEEL_RPM_MAX, flywheelRpm + FLYWHEEL_RPM_STEP);
            if (down && !prevDown) flywheelRpm = Math.max(0, flywheelRpm - FLYWHEEL_RPM_STEP);
            prevUp = up;
            prevDown = down;

            double actualRpm = 0;
            boolean ready = false;
            if (outtake != null) {
                if (reverse) {
                    outtake.setPower(-OUTTAKE_REVERSE_POWER);
                } else if (outtakeOn && FLYWHEEL_VELOCITY_CONTROL) {
                    outtake.setVelocity(flywheelRpm * FLYWHEEL_TICKS_PER_REV / 60.0); // ticks per second
                } else if (outtakeOn) {
                    outtake.setPower(OUTTAKE_POWER);
                } else {
                    outtake.setPower(0);
                }
                if (FLYWHEEL_VELOCITY_CONTROL) {
                    actualRpm = outtake.getVelocity() * 60.0 / FLYWHEEL_TICKS_PER_REV;
                    ready = outtakeOn && !reverse && flywheelRpm > 0
                            && Math.abs(actualRpm - flywheelRpm) <= FLYWHEEL_READY_TOLERANCE * flywheelRpm;
                }
            }
            if (ready && !wasReady) { // buzz both drivers once when it's safe to shoot
                gamepad1.rumble(300);
                gamepad2.rumble(300);
            }
            wasReady = ready;

            // ---------------- Telemetry ----------------
            telemetry.addData("drive", fieldCentric ? "FIELD-centric" : "ROBOT-centric");
            telemetry.addData("heading", "%.1f", Math.toDegrees(heading));
            telemetry.addData("wheels", xLock ? "X-LOCK"
                    : (aligned ? "aligned" : "WAITING for modules to line up"));
            telemetry.addData("speed", gamepad1.left_bumper ? "SLOW" : "full");
            telemetry.addData("intake", intake == null ? "not found" : String.format("%.2f", intakePower));
            String flywheelState;
            if (outtake == null) {
                flywheelState = "not found";
            } else if (reverse) {
                flywheelState = "REVERSE";
            } else if (!outtakeOn) {
                flywheelState = "off";
            } else if (!FLYWHEEL_VELOCITY_CONTROL) {
                flywheelState = "ON (power mode)";
            } else {
                flywheelState = ready ? "READY - shoot" : "spinning up";
            }
            telemetry.addData("flywheel", flywheelState);
            telemetry.addData("flywheel rpm", "target %.0f  actual %.0f", flywheelRpm, actualRpm);
            for (int i = 0; i < 4; i++) {
                telemetry.addData("mod " + DRIVE_NAMES[i], "ang %.0f  tgt %.0f  err %.0f  pwr %.2f%s",
                        angle[i], target[i], err[i], drivePower[i], flipped[i] ? "  FLIP" : "");
            }
            telemetry.addData("loop ms", "%.1f", dt * 1000);
            telemetry.update();
        }

        for (int i = 0; i < 4; i++) {
            steer[i].setPower(0);
            drive[i].setPower(0);
        }
        if (intake != null) intake.setPower(0);
        if (outtake != null) outtake.setPower(0);
    }

    // ===================== HELPERS =====================

    /** Module angle, same formula as swervev3. */
    private double readAngle(int i) {
        return wrap(enc[i].getVoltage() * (360 / enc[i].getMaxVoltage()) - 180 + OFFSETS[i]);
    }

    /** Standard math angle (CCW, 0 = right, 90 = forward) to swervev3's module angle. */
    static double toModuleAngle(double standardDeg) {
        return wrap(-standardDeg);
    }

    /** Turn module i toward target[i], flipping when that's shorter. */
    private void steerModule(int i) {
        double e = wrap(target[i] - angle[i] + (flipped[i] ? 180 : 0));
        if (Math.abs(e) > 90 + FLIP_HYST) {
            flipped[i] = !flipped[i];
            e = wrap(e + 180);
        }
        err[i] = e;
        steer[i].setPower(Math.signum(e) * steerPower(Math.abs(e)));
    }

    /** Exponential dampener: full power far away, easing down near the target. */
    static double steerPower(double dist) {
        if (dist < STEER_DEADBAND) return 0;
        double ease = 1 - Math.exp(-(dist - STEER_DEADBAND) / EASE_ANGLE);
        return MIN_STEER + (MAX_STEER - MIN_STEER) * ease;
    }

    /** Wrap any angle into [-180, 180). */
    static double wrap(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }

    private static double stronger(double a, double b) {
        return Math.abs(a) >= Math.abs(b) ? a : b;
    }

    private static DcMotorSimple.Direction dir(boolean reversed) {
        return reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
    }

    private DcMotor optionalMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    private DcMotorEx optionalFlywheel(String name) {
        try {
            return hardwareMap.get(DcMotorEx.class, name);
        } catch (IllegalArgumentException e) {
            return null;
        }
    }

    private IMU initImu() {
        try {
            IMU found = hardwareMap.get(IMU.class, IMU_NAME);
            found.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(IMU_LOGO, IMU_USB)));
            found.resetYaw();
            return found;
        } catch (IllegalArgumentException e) {
            return null;
        }
    }
}
