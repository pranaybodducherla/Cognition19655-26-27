package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Simple swerve. Left stick = move, right stick = turn.
 * 4 steering servos (as, bs, cs, ds), 4 drive motors (a, b, c, d),
 * 4 analog encoders (aa, ba, ca, da).
 */
@TeleOp(name = "Simple Swerve")
public class SimpleSwerve extends LinearOpMode {

    // Wheel angle when the encoder reads 0 is fixed by these (same as swervev3)
    private static final double[] OFFSETS = {50, 70, 10, 70};
    // Wheel positions: a = front-left, b = front-right, c = back-left, d = back-right
    private static final double[] X = {-1, 1, -1, 1};
    private static final double[] Y = {1, 1, -1, -1};
    private static final double MAX_DRIVE = 0.6;

    private final CRServo[] steer = new CRServo[4];
    private final DcMotor[] drive = new DcMotor[4];
    private final AnalogInput[] enc = new AnalogInput[4];

    @Override
    public void runOpMode() {
        String[] s = {"as", "bs", "cs", "ds"};
        String[] m = {"a", "b", "c", "d"};
        String[] e = {"aa", "ba", "ca", "da"};
        for (int i = 0; i < 4; i++) {
            steer[i] = hardwareMap.get(CRServo.class, s[i]);
            drive[i] = hardwareMap.get(DcMotor.class, m[i]);
            enc[i] = hardwareMap.get(AnalogInput.class, e[i]);
            drive[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        drive[3].setDirection(DcMotor.Direction.REVERSE); // same as swervev3

        waitForStart();

        while (opModeIsActive()) {
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;   // up = forward
            double turn = gamepad1.right_stick_x; // right = clockwise

            // Each wheel's wanted direction and speed
            double[] wantAngle = new double[4];
            double[] wantSpeed = new double[4];
            double biggest = 1;
            for (int i = 0; i < 4; i++) {
                double wx = x + turn * Y[i];
                double wy = y - turn * X[i];
                wantSpeed[i] = Math.hypot(wx, wy);
                wantAngle[i] = -Math.toDegrees(Math.atan2(wy, wx)); // swervev3 angles: forward = -90
                biggest = Math.max(biggest, wantSpeed[i]);
            }

            boolean moving = Math.hypot(x, y) > 0.1 || Math.abs(turn) > 0.1;

            for (int i = 0; i < 4; i++) {
                double angle = wrap(enc[i].getVoltage() / enc[i].getMaxVoltage() * 360 - 180 + OFFSETS[i]);

                if (!moving) {
                    steer[i].setPower(0);
                    drive[i].setPower(0);
                    telemetry.addData("wheel " + m[i], "angle %.0f", angle);
                    continue;
                }

                double error = wrap(wantAngle[i] - angle);
                double direction = 1;
                if (Math.abs(error) > 90) {  // quicker to point the other way and drive backwards
                    error = wrap(error + 180);
                    direction = -1;
                }

                // Steer: full power when far off, slowing down as it gets close
                steer[i].setPower(Math.abs(error) < 3 ? 0 : Range.clip(error / 45, -1, 1));

                // Drive: less power while the wheel is still turning into place
                double aligned = Math.max(0, Math.cos(Math.toRadians(error)));
                drive[i].setPower(direction * wantSpeed[i] / biggest * MAX_DRIVE * aligned);

                telemetry.addData("wheel " + m[i], "angle %.0f  want %.0f  off by %.0f",
                        angle, wantAngle[i], error);
            }
            telemetry.update();
        }
    }

    private static double wrap(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
}
