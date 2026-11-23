package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoBottom", group = "Autonomous")
public class RedAutoBottom extends AutoBase {
    @Override
    public void runOpMode() {
        initializeRobot();
        waitForStart();
        if (opModeIsActive()) {
            driveFor(0.5, 0, 0, 1900);
            driveFor(0, 0, -0.15, 1200); // turn left (drive.java convention: positive = right)
        }
    }
}
