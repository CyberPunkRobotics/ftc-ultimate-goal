package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Test Hard", group="Linear Opmode")

public class TeleOpSuprem extends LinearOpMode {

    private RobotMap robot = null;

    double  contPower;
    boolean hasShot = false;


    @Override
    public void runOpMode() {

        ElapsedTime runtime = new ElapsedTime();
        robot = new RobotMap(hardwareMap, this);
        waitForStart();
        runtime.reset();
        robot.lansareRing.setPosition(0.5);

        while (opModeIsActive()) {


            telemetry.addData("Encoder stanga spate: ", robot.stangaFata.getCurrentPosition());
            telemetry.addData("Encoder dreapta spate: ", robot.dreaptaSpate.getCurrentPosition());
            telemetry.update();

            Polar rotation = Polar.fromCartesian(gamepad1.left_stick_x * 0.5, -gamepad1.left_stick_y);

            //if (Math.abs(gamepad1.right_stick_x) > 0.1) {
            //    robot.teleOpDrive(0, gamepad1.right_stick_x);
            //}
            // {
            robot.driveFieldRelativeAngle(rotation.getR()/2, rotation.getTheta());
            //}



            // Prindere wobble
            if (gamepad1.dpad_right){//il lasa
                robot.servoWobble.setPosition(1);
            }
            else if (gamepad1.dpad_left){
                robot.servoWobble.setPosition(0);
            }

            // Ridicare wobble
            if (gamepad1.y){
                robot.servoRidicare.setPosition(0);
            }
            else if (gamepad1.a){
                robot.servoRidicare.setPosition(1);
            }
            else if (gamepad1.b){
                robot.servoRidicare.setPosition(0.5);
            }


            // Servo loader
            if (gamepad1.right_trigger >= 0.3 && gamepad1.right_trigger <= 0.7 && !hasShot) {
                robot.lansareRing.setPosition(1);
                sleep(1200);
                hasShot = true;
            }
            if (hasShot){
                robot.lansareRing.setPosition(0.5);
                hasShot = false;
            }
            hasShot = true;
            while (gamepad1.right_trigger >= 0.9 && gamepad1.right_trigger <= 1|| !hasShot){
                robot.lansareRing.setPosition(1);
                sleep(500);
                robot.lansareRing.setPosition(0.5);
                sleep(800);
                robot.lansareRing.setPosition(1);

            }


            //Motor shooter,motor intake, kill all
            if (gamepad1.dpad_up){
                robot.motorShooter.setPower(1);
                robot.motorIntake.setPower(0);
            }
            else if (gamepad1.dpad_down){
                robot.motorShooter.setPower(0);
                robot.motorIntake.setPower(-1);
            }
            else if (gamepad1.x){
                robot.motorShooter.setPower(0);
                robot.motorIntake.setPower(0);
            }


            //Reset angle
            if(gamepad1.left_trigger > 0.5){
//                robot.lansareRing.setPosition(1);
                int robotAngle = (int) robot.getCurrentAngle();
                robot.rotate(-robotAngle, 0.35, 3);
                sleep(200);
                robotAngle = (int) robot.getCurrentAngle();
                robot.rotate(-robotAngle, 0.15, 1);
            }




            //mers fata-spate
            while(gamepad1.right_bumper){

                double stangaFataPower;
                double dreaptaFataPower;
                double stangaSpatePower;
                double dreaptaSpatePower;


                double drive = -gamepad1.left_stick_x;



                stangaFataPower  = robot.SQRT(-drive - 0) ;
                dreaptaFataPower= robot.SQRT(drive - 0) ;
                stangaSpatePower  = robot.SQRT(-drive - 0) ;
                dreaptaSpatePower= robot.SQRT(drive - 0) ;


                robot.stangaFata.setPower(stangaFataPower);
                robot.dreaptaFata.setPower(dreaptaFataPower);
                robot.stangaSpate.setPower(stangaSpatePower);
                robot.dreaptaSpate.setPower(dreaptaSpatePower);
            }


            // doar rotire
            while(gamepad1.left_bumper){

                double stangaFataPower;
                double dreaptaFataPower;
                double stangaSpatePower;
                double dreaptaSpatePower;


                double turn  =  gamepad1.left_stick_x * 0.1;


                stangaFataPower  = robot.SQRT(0 - turn);
                dreaptaFataPower= robot.SQRT(0 - turn);
                stangaSpatePower  = robot.SQRT(0 - turn);
                dreaptaSpatePower= robot.SQRT(0 - turn);


                robot.stangaFata.setPower(stangaFataPower);
                robot.dreaptaFata.setPower(dreaptaFataPower);
                robot.stangaSpate.setPower(stangaSpatePower);
                robot.dreaptaSpate.setPower(dreaptaSpatePower);
            }



        }
    }
}
