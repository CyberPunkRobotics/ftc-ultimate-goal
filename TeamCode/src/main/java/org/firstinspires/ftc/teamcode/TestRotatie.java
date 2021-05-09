package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name="test rotatie")
//@Disabled
public class TestRotatie extends LinearOpMode {

    private RobotMap robot = null;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);
        //robot.servoWobble.setPosition(0);
        //robot.lansareRing.setPosition(0.5);
        robot.zeroPowerBeh();
        waitForStart();

        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (opModeIsActive()) {
            telemetry.addData("Semnal odometrie intake: ", robot.motorIntake.getCurrentPosition());
            telemetry.addData("Semnal odometrie shooter: ", robot.motorShooter.getCurrentPosition());
            telemetry.update();
        }

    }
}