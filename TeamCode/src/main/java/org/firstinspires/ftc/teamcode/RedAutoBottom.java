package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoBottom", group = "Autonomous")
public class RedAutoBottom extends AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        driveFor(0.5, 0, 0, 1900);
        move(0, 0, 0.15, 1200); // turn left
        // aimAtTag(2000);

        robot.stop();
    }
}