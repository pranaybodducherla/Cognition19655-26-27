package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoBottom", group = "Autonomous")
public class BlueAutoBottom extends AutoBase {
    @Override
    public void runOpMode() {
        initializeRobot();
        waitForStart();
        if (opModeIsActive()) {
            driveFor(0.5, 0, 0, 1600);
            driveFor(0, 0, 0.15, 1200); // turn right (drive.java convention: positive = right)
            driveFor(0.5, 0, 0, 300);
        }
    }
}
