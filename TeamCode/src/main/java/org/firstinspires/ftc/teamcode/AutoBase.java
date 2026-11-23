package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/** Shared setup and simple timed commands for the four autonomous routines. */
public abstract class AutoBase extends LinearOpMode {
    protected HardwarePushbot robot;

    protected void initializeRobot() {
        robot = new HardwarePushbot(hardwareMap);
    }

    /** Drive with drive.java axis signs: forward, strafe right, and turn right are positive. */
    protected void driveFor(double forward, double strafe, double turn, long milliseconds) {
        if (!opModeIsActive()) return;
        robot.drive(forward, strafe, turn);
        sleep(milliseconds);
        robot.stop();
    }
}
