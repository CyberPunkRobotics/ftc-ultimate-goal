//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name="Auto test")
////@Disabled
//public class AutonomieInterior1 extends LinearOpMode {
//
//    private RobotMap robot = null;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new RobotMap(hardwareMap, this);
//        robot.servoWobble.setPosition(0);
//        robot.lansareRing.setPosition(0.5);
//        robot.zeroPowerBeh();
//        waitForStart();
//
//        if (opModeIsActive()) {
//
//
//            robot.runUsingEncoders(3300, 1, 5);
//            sleep(1000);
//
//            //FourRings();
//            //NoRing();
//            OneRing();
//
//        }
//
//    }
//
//    private void FourRings() {
//
//        //robot.motorShooter.setPower(1);
//
//        robot.runUsingEncoders(1600, 1, 5);
//        sleep(150);
//        robot.rotateConstantSpeed(3, 0.4, 3);
//        //sleep(1000);
//        //shooter
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//
//        robot.rotateConstantSpeed(35, 0.4, 5);
//        //robot.motorShooter.setPower(0);
//        robot.runUsingEncoders(5200, 1, 5);
//        robot.servoWobble.setPosition(1);
//        sleep(1000);
//        robot.runUsingEncoders(-3800, 1, 10);
//        sleep(350);
//        robot.rotateConstantSpeed(175, 0.4, 6);
//        sleep(350);
//        robot.runUsingEncoders(5100, 1, 5);
//        sleep(350);
//        robot.servoWobble.setPosition(0);
//        sleep(500);
//
//        robot.servoRidicare.setPosition(0.5);
//        sleep(1000);
//        robot.runUsingEncoders(-8600, 1, 10);
//        robot.rotateConstantSpeed(-90, 0.4, 5);
//
//        //robot.rotateConstantSpeed(-180, 0.4, 6);
//        sleep(500);
//
//
//        //sleep(350);
//        //robot.runUsingEncoders(3700, 1 ,5);
//        //sleep(200);
//        //robot.servoWobble.setPosition(0);
//        //sleep(500);
//        //robot.servoRidicare.setPosition(0.5);
//        //robot.rotateConstantSpeed(-165,0.4,4);
//
//            /*for(int i=0;i<3;i++){
//                robot.servoIntake.setPosition(1);
//                sleep(500);
//                robot.servoIntake.setPosition(0.5);
//                sleep(800);
//                robot.servoIntake.setPosition(1);
//
//            }*/
//            /*robot.runUsingEncoders(1500, 0.5,3);
//            robot.rotateConstantSpeed(-105, 0.2, 3);
//            sleep(1000);
//            robot.servoWobble.setPosition(0);
//            robot.rotateConstantSpeed(90, 0.2, 3);
//            robot.runUsingEncoders(-1500, 0.5,3);
//            //prinde wobble
//            robot.runUsingEncoders(2000, 0.5,3);
//            robot.rotateConstantSpeed(-90, 0.2, 3);*/
//        //lasa wobble
//
//        //robot.rotateConstantSpeed(-60, 0.4, 5);
//        //robot.runUsingEncoders(1000, 0.5, 5);
//        //robot.servoWobble.setPosition(0);
//    }
//
//    private void NoRing(){
//
//        //robot.motorShooter.setPower(1);
//        robot.runUsingEncoders(1600, 1, 5);
//        sleep(150);
//        robot.rotateConstantSpeed(3, 0.4, 3);
//        //sleep(1000);
//        //shooter
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//
//        robot.rotateConstantSpeed(87, 0.4, 5);
//        //robot.motorShooter.setPower(0);
//        sleep(350);
//        robot.runUsingEncoders(2850, 1, 6);
//        sleep(350);
//        robot.servoWobble.setPosition(1);
//        sleep(500);
//        robot.runUsingEncoders(-2000, 1 ,5);
//        sleep(500);
//        robot.rotateConstantSpeed(87, 0.4, 5);
//
//
//
//
//
//    }
//
//    private void OneRing(){
////        robot.motorShooter.setPower(1);
//        robot.runUsingPID(1600, 1, 5);
//        sleep(150);
//        robot.rotateConstantSpeed(3, 0.4, 3);
//        //sleep(1000);
//        //shooter
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.lansareRing.setPosition(1);
//        sleep(350);
//        robot.lansareRing.setPosition(0.5);
//        sleep(350);
//        robot.rotate(4, 0.4, 5);
//        sleep(350);
//
//        robot.runUsingPID(3300, 1, 5);
//        robot.servoWobble.setPosition(1);
//        sleep(750);
//        robot.runUsingPID(-6100,1,5);
//        sleep(500);
//        robot.rotateConstantSpeed(137, 0.4, 5);
//        sleep(500);
//        robot.runUsingPID(2150,1,5);
//        robot.servoWobble.setPosition(0);
//        sleep(500);
//        robot.rotateConstantSpeed(-130, 0.4, 5);
//        sleep(350);
//
//
//    }
//
//}