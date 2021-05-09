package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp Field Relative Daria", group="Linear Opmode")
public class TeleOpRelativeDaria extends LinearOpMode {
    private RobotMap robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {
            double forward = gamepad1.left_stick_y * -1; //The y direction on the gamepad is reversed idk why
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            robot.driveFieldRelative3(forward, strafe, rotate);
        }
    }
}
