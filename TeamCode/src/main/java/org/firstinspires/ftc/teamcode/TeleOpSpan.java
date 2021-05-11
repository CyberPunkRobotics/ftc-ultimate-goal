package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp Span")
public class TeleOpSpan extends LinearOpMode {

    RobotMap robot;
    ElapsedTime runtime;
    private final double COMMAND_DELAY = 0.35;
    private double ridicareShooterPosition = 1;
    private final double RIDICARE_SHOOTER_SPEED = 0.001;

    private final double TOWER_GOAL_HEIGHT = 0.49;
    private final double POWER_SHOT_HEIGHT = 0.71;

    private final double SHOOT_TIMEOUT = 350;//miliseconds
    private int repetitions = 5;


    //Odometrie
    final double COUNTS_PER_INCH = 194.04;//1440 ticks, 6cm diametru


    //TODO: regandit functia asta
    private void shootRings() {
        if (repetitions < 3) {
            if (runtime.milliseconds() >= SHOOT_TIMEOUT) {
                if (robot.lansareRing.getPosition() > 0.75) {
                    robot.lansareRing.setPosition(0.5);
                }
                else {
                    robot.lansareRing.setPosition(1);
                }
                runtime.reset();
                repetitions++;
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);

//        robot.encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rotite.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.zeroPowerBeh();
//        robot.rotite.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.encoderStanga.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        runtime = new ElapsedTime();
        runtime.reset();

        robot.servoWobble.setPosition(0.8);

        waitForStart();

        while(opModeIsActive()) {

            //Display Global (x, y, theta) coordinates
//            telemetry.addData("X Position", robot.globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y Position", robot.globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation (Degrees)", robot.globalPositionUpdate.returnOrientation());
//
//            telemetry.addData("Vertical left encoder position", robot.encoderStanga.getCurrentPosition() * robot.globalPositionUpdate.verticalLeftEncoderPositionMultiplier);
//            telemetry.addData("Vertical right encoder position", robot.encoderDreapta.getCurrentPosition() * robot.globalPositionUpdate.verticalRightEncoderPositionMultiplier);
//            telemetry.addData("horizontal encoder position", robot.motorIntake.getCurrentPosition() * robot.globalPositionUpdate.normalEncoderPositionMultiplier);
//
//            telemetry.addData("Vertical robotEncoderWheelDistance: ", robot.globalPositionUpdate.robotEncoderWheelDistance);
//            telemetry.addData("Thread Active", robot.positionThread.isAlive());
//            telemetry.update();


            //Fata - spate + rotire
            double forward = -gamepad1.left_stick_y;
            double leftRight = gamepad1.left_stick_x;
            double rotate = -gamepad1.right_stick_x;

            if (robot.motorShooter.getPower() > 0) {
                rotate /= 4;
            }

            robot.teleOpDrive(forward, rotate);
//            robot.driveFieldRelative3(leftRight, forward, rotate);
            //Shooter si intake
            if ((gamepad1.right_bumper || gamepad2.right_bumper) && runtime.seconds() > COMMAND_DELAY){
                if (robot.motorShooter.getPower() != 0) {
                    robot.motorShooter.setPower(0);
                }
                else {
                    robot.motorShooter.setPower(1);
//                    if (robot.ridicareShooter.getPosition() == TOWER_GOAL_HEIGHT) {
//                        robot.motorShooter.setPower(1);
//                    }
//                    else {
//                        robot.motorShooter.setPower(0.5);
//                    }
                }
//                robot.motorIntake.setPower(0);
                runtime.reset();
            }
            if ((gamepad1.left_bumper || gamepad2.left_bumper) && runtime.seconds() > COMMAND_DELAY){
//                robot.motorShooter.setPower(0);
                if (robot.motorIntake.getPower() != 0 || robot.rotite.getPower() != 0) {
                    robot.motorIntake.setPower(0);
                    robot.rotite.setPower(0);
                }
                else {
                    robot.motorIntake.setPower(1);
                    robot.rotite.setPower(1);
                }
                runtime.reset();
            }
            if (gamepad2.left_trigger > 0) {
//                robot.motorShooter.setPower(0);
                robot.motorIntake.setPower(-0.4);
                runtime.reset();
            }

            //Lansare inel
            if (gamepad1.left_trigger > 0.5) {
                robot.lansareRing.setPosition(1);
                sleep(600);
                robot.lansareRing.setPosition(0.5);
            }
            if (gamepad1.right_trigger == 0) {
                robot.lansareRing.setPosition(0.5);
            }

            if (gamepad1.right_trigger == 1) {
                robot.shoot3Rings();
            }

            if (gamepad2.dpad_up) {
                robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
//                robot.ridicareShooter.setPosition(robot.ridicareShooter.getPosition() + 0.05);
//                if (robot.motorShooter.getPower() != 0) {
//                    robot.motorShooter.setPower(1);
//                }
            }
            if (gamepad2.dpad_down) {
//                robot.ridicareShooter.setPosition(robot.ridicareShooter.getPosition() - 0.05);
                robot.ridicareShooter.setPosition(POWER_SHOT_HEIGHT);
//                if (robot.motorShooter.getPower() != 0) {
//                    robot.motorShooter.setPower(0.5);
//                }
            }
            telemetry.addData("Pozitie shooter: " ,robot.ridicareShooter.getPosition());
            telemetry.update();

            if (gamepad1.dpad_down) {
                robot.motorShooter.setPower(0.65);
            }
            if (gamepad1.dpad_up) {
                robot.motorShooter.setPower(1);
            }

            //ridicare brate
            if (gamepad2.dpad_right) {
                robot.servoBratStanga.setPosition(0.5);
                robot.servoBratDreapta.setPosition(0.5);
            }
            if (gamepad2.dpad_left) {
                robot.servoBratStanga.setPosition(0.1);
                robot.servoBratDreapta.setPosition(0.9);
            }

            // Ridicare wobble
            if (gamepad2.a){
                robot.servoRidicare.setPosition(0);
            }
            else if (gamepad2.y){
                robot.servoRidicare.setPosition(1);
            }
            else if (gamepad2.b){
                robot.servoRidicare.setPosition(0.5);
            }

            //prindere wobble
            if (gamepad2.x && runtime.seconds() > COMMAND_DELAY){
                if (robot.servoWobble.getPosition() > 0.7) {
                    robot.servoWobble.setPosition(0);
                }
                else {
                    robot.servoWobble.setPosition(1);
                }
                runtime.reset();
            }

        }
    }
}
