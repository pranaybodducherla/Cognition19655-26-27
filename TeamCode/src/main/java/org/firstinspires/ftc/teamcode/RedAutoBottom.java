package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedAutoBottom", group = "Autonomous")
public class RedAutoBottom extends LinearOpMode {

    private final HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        move(0.5, 0, 0, 1600);
        move(0.5, 0, 0, 300);
        move(0, 0, 0.15, 1200); // turn left

        robot.stop();
    }

    // Counter-clockwise turn = positive
    private void move(double forward, double strafe, double turn, long ms) {
        robot.drive(forward, strafe, turn);
        sleep(ms);
        robot.stop();
    }

}