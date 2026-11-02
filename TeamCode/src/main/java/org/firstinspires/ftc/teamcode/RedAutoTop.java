package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "RedAutoTop", group = "Autonomous")
public class RedAutoTop extends LinearOpMode {

    private final HardwarePushbot robot = new HardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        move(-0.5, 0, 0, 1700);

        robot.stop();
    }

    // Counter-clockwise turn = positive
    private void move(double forward, double strafe, double turn, long ms) {
        robot.drive(forward, strafe, turn);
        sleep(ms);
        robot.stop();
    }
}