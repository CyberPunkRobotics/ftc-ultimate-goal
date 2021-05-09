package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test servo")
public class TestServo extends LinearOpMode {

    RobotMap robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);

        waitForStart();
        while (opModeIsActive()) {
            robot.rotireIntake.setPower(1);
            robot.motorIntake.setPower(gamepad1.left_stick_y);
            telemetry.addData("Rotire intake: ", robot.motorIntake.getPower());
            telemetry.update();
        }
    }
}
