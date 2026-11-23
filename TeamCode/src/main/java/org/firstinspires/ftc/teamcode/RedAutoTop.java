package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedAutoTop", group = "Autonomous")
public class RedAutoTop extends AutoBase {
    @Override
    public void runOpMode() {
        initializeRobot();
        waitForStart();
        if (opModeIsActive()) driveFor(-0.5, 0, 0, 1700);
    }
}
