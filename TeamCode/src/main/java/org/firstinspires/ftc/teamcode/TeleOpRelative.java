package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Relative TeleOP", group="Linear Opmode")
@Disabled
public class TeleOpRelative extends LinearOpMode {

    private RobotMap robot = null;

    double  contPower;


    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {


            Polar rotation = Polar.fromCartesian(gamepad1.left_stick_x, -gamepad1.left_stick_y);

            if (Math.abs(gamepad1.right_stick_x) > 0.05) {
                robot.teleOpDrive(0, gamepad1.right_stick_x);
            }
            else {
                robot.driveFieldRelativeAngle(rotation.getR(), rotation.getTheta());
            }



            // Prindere wobble
            if (gamepad1.dpad_left){
                robot.servoWobble.setPosition(1);
            }
            else if (gamepad1.dpad_right){
                robot.servoWobble.setPosition(0);
            }



            // Servo loader
            if (gamepad1.right_bumper != false) {
                //robot.lansareRing.setPower(1);
            }
            else //robot.lansareRing.setPower(0);


                // Motor shooter
                if (gamepad1.y){
                    robot.motorShooter.setPower(1);
                }
                else if(gamepad1.a){
                    robot.motorShooter.setPower(0);
                }

        }
    }
}
