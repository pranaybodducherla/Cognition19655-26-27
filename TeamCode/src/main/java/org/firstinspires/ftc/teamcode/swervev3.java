package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Your swervev3, with the steering working exactly like before, plus the wheels now drive.
 *
 * Left stick: push the way you want to go. The modules turn to face that way (same as before),
 * and once ALL four are lined up the wheels drive. Push further = faster.
 */
@TeleOp(name = "swervev3")
public class swervev3 extends LinearOpMode {

    // ---- Same values as your original ----
    private static final double[] OFFSETS = {50, 70, 10, 70};
    private static final double STEER_POWER = 0.3;      // servo power while turning
    private static final double STEER_DEADBAND = 20;    // degrees: close enough, stop turning
    private static final double STICK_DEADBAND = 0.2;   // stick must be pushed this far

    // ---- New: driving ----
    private static final double DRIVE_SPEED = 0.5;      // max wheel power (your old trigger used 0.5)
    private static final double KEEP_DRIVING = 35;      // once moving, only stop if a module gets this far off

    private final CRServo[] steer = new CRServo[4];
    private final DcMotor[] drive = new DcMotor[4];
    private final AnalogInput[] enc = new AnalogInput[4];

    private final double[] servoangle = new double[4];
    private final double[] diff = new double[4];
    private double joystickAngle;
    private double joystickMag;
    private boolean driving = false;

    @Override
    public void runOpMode() {
        String[] steerNames = {"as", "bs", "cs", "ds"};
        String[] driveNames = {"a", "b", "c", "d"};
        String[] encNames = {"aa", "ba", "ca", "da"};
        for (int i = 0; i < 4; i++) {
            steer[i] = hardwareMap.get(CRServo.class, steerNames[i]);
            drive[i] = hardwareMap.get(DcMotor.class, driveNames[i]);
            enc[i] = hardwareMap.get(AnalogInput.class, encNames[i]);
            steer[i].setDirection(CRServo.Direction.FORWARD);
            drive[i].setDirection(DcMotor.Direction.FORWARD);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            drive[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        drive[3].setDirection(DcMotor.Direction.REVERSE); // d was reversed in your original

        waitForStart();

        while (opModeIsActive()) {
            joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) / Math.PI * 180;
            joystickMag = Math.min(1, Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y));
            boolean stickPushed = joystickMag >= STICK_DEADBAND;

            // ---- Steering: same as your original ----
            double worst = 0;
            for (int i = 0; i < 4; i++) {
                servoangle[i] = wrap(enc[i].getVoltage() * (360 / enc[i].getMaxVoltage()) - 180 + OFFSETS[i]);
                diff[i] = wrap(joystickAngle - servoangle[i]);
                worst = Math.max(worst, Math.abs(diff[i]));

                if (stickPushed && Math.abs(diff[i]) >= STEER_DEADBAND) {
                    steer[i].setPower(Math.signum(diff[i]) * STEER_POWER);
                } else {
                    steer[i].setPower(0);
                }
            }

            // ---- Driving: only once every module faces the right way ----
            if (!stickPushed) {
                driving = false;
            } else {
                driving = worst < (driving ? KEEP_DRIVING : STEER_DEADBAND);
            }
            double wheelPower = driving ? joystickMag * DRIVE_SPEED : 0;
            for (DcMotor m : drive) m.setPower(wheelPower);

            telemetry2(wheelPower);
        }

        for (int i = 0; i < 4; i++) {
            steer[i].setPower(0);
            drive[i].setPower(0);
        }
    }

    private void telemetry2(double wheelPower) {
        telemetry.addData("joystick angle", joystickAngle);
        telemetry.addData("joystick magnitude", joystickMag);
        telemetry.addData("wheels", driving ? "DRIVING" : "waiting for modules to line up");
        telemetry.addData("wheel power", wheelPower);
        for (int i = 0; i < 4; i++) {
            telemetry.addData("servo " + (i + 1) + " angle", "%.1f  (off by %.1f)", servoangle[i], diff[i]);
        }
        telemetry.update();
    }

    private static double wrap(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
}
