package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/** Shared moves for all autos. Not an OpMode itself. */
public abstract class AutoBase extends LinearOpMode {

    protected final HardwarePushbot robot = new HardwarePushbot();

    private static final double HEADING_KP = 1.5;  // how hard it corrects to stay straight
    private static final double AIM_KP = 0.02;     // how hard it turns toward the tag

    /** Old-style timed move (same as before). Counter-clockwise turn = positive. */
    protected void move(double forward, double strafe, double turn, long ms) {
        robot.drive(forward, strafe, turn);
        sleep(ms);
        robot.stop();
    }

    /** Drive for ms while the gyro keeps the robot pointed at holdDeg. */
    protected void driveFor(double forward, double strafe, double holdDeg, long ms) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < ms) {
            double err = angleWrap(Math.toRadians(holdDeg) - robot.getHeading());
            robot.drive(forward, strafe, Range.clip(HEADING_KP * err, -0.4, 0.4));
        }
        robot.stop();
    }

    /** Turn in place to an exact heading in degrees (counter-clockwise = positive). */
    protected void turnTo(double deg, long timeoutMs) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < timeoutMs) {
            double err = angleWrap(Math.toRadians(deg) - robot.getHeading());
            if (Math.abs(err) < Math.toRadians(2)) break;
            robot.drive(0, 0, Range.clip(HEADING_KP * err, -0.5, 0.5));
        }
        robot.stop();
    }

    /** Turn until the Limelight tag is centered. Returns false if it never saw a tag. */
    protected boolean aimAtTag(long timeoutMs) {
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.milliseconds() < timeoutMs) {
            LLResult r = robot.getTarget();
            if (r == null) {
                robot.stop();
                continue;
            }
            if (Math.abs(r.getTx()) < 1.5) {
                robot.stop();
                return true;
            }
            robot.drive(0, 0, Range.clip(-AIM_KP * r.getTx(), -0.4, 0.4));
        }
        robot.stop();
        return false;
    }

    private static double angleWrap(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}