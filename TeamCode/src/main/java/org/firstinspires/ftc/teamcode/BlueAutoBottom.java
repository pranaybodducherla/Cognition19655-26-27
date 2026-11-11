package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "BlueAutoBottom", group = "Autonomous")
public class BlueAutoBottom extends AutoBase {
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        driveFor(0.5, 0, 0, 1600);
        move(0, 0, -0.15, 1200); // turn right
        driveFor(0.5, 0, 0, 300);
        // aimAtTag(2000);

        robot.stop();
    }
}