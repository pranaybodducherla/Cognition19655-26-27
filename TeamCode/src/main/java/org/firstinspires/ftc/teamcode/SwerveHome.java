package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.util.List;

/**
 * Turns every swerve module to its starting position (wheels pointing straight forward)
 * and holds it there. Press START, wait for "ALL HOME", then press STOP.
 *
 * Uses the same names, offsets and angle formula as MainTeleOp:
 *   forward = -90, right = 0, backward = +90, left = 180
 */
@TeleOp(name = "Swerve Home (starting position)")
public class SwerveHome extends LinearOpMode {

    private static final String[] STEER_NAMES = {"as", "bs", "cs", "ds"};
    private static final String[] ENC_NAMES   = {"aa", "ba", "ca", "da"};
    private static final double[]  OFFSETS        = {50, 70, 10, 70};
    private static final boolean[] STEER_REVERSED = {false, false, false, false};

    private static final double HOME_ANGLE = -90;    // wheels straight forward

    // Same exponential dampener as MainTeleOp
    private static final double MAX_STEER = 0.5;
    private static final double MIN_STEER = 0.08;
    private static final double EASE_ANGLE = 20;
    private static final double DEADBAND = 3;        // degrees: close enough

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        CRServo[] steer = new CRServo[4];
        AnalogInput[] enc = new AnalogInput[4];
        for (int i = 0; i < 4; i++) {
            steer[i] = hardwareMap.get(CRServo.class, STEER_NAMES[i]);
            enc[i] = hardwareMap.get(AnalogInput.class, ENC_NAMES[i]);
            steer[i].setDirection(STEER_REVERSED[i]
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            steer[i].setPower(0);
        }

        telemetry.addLine("Press START to move the wheels to the starting position");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            boolean allHome = true;
            for (int i = 0; i < 4; i++) {
                double angle = wrap(enc[i].getVoltage() * (360 / enc[i].getMaxVoltage()) - 180 + OFFSETS[i]);
                double err = wrap(HOME_ANGLE - angle);
                double dist = Math.abs(err);
                steer[i].setPower(Math.signum(err) * steerPower(dist));
                if (dist >= DEADBAND) allHome = false;
                telemetry.addData("module " + (i + 1), "angle %.1f  off by %.1f  %s",
                        angle, err, dist < DEADBAND ? "HOME" : "moving");
            }
            telemetry.addLine(allHome ? "ALL HOME - you can press STOP" : "Moving to starting position...");
            telemetry.update();
        }

        for (CRServo s : steer) s.setPower(0);
    }

    private static double steerPower(double dist) {
        if (dist < DEADBAND) return 0;
        double ease = 1 - Math.exp(-(dist - DEADBAND) / EASE_ANGLE);
        return MIN_STEER + (MAX_STEER - MIN_STEER) * ease;
    }

    private static double wrap(double deg) {
        deg = (deg + 180) % 360;
        if (deg < 0) deg += 360;
        return deg - 180;
    }
}
