//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//@Autonomous(name="Test PID in auto")
//public class AutonomieTestPID extends LinearOpMode {
//
//    RobotMap robot;
//    private final double TOWER_GOAL_HEIGHT = 0.61;
//    private final double POWER_SHOT_HEIGHT = 0.7;
//
//    private void startAndShoot() {
//        //Porneste motoarele
////        robot.servoRidicare.setPosition(0.8);
//        robot.motorShooter.setPower(1);
//
//        //Se pozitioneaza la tragere
//        robot.runUsingPID(robot.cmToTicks(160), 1, 3);
//
//
//        //Se pozitioneaza la tower goal
//        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
//        robot.runUsingPID(-9, 0.3, 3);
//
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
//        sleep(150);
//        //Opreste shooterul
//        robot.motorShooter.setPower(0);
//    }
//
//    private void FourRings() {
//
//
//        startAndShoot();
//        //Se pozitioneaza pentru wobble si il duce
//        robot.rotateConstantSpeed(-12, 0.4, 2);
//        robot.runUsingPID(robot.cmToTicks(160), 1, 5);
//
//        //lasa primul wobble
//        robot.servoWobble.setPosition(0.8);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        //Se intoarce dupa al doilea
//        robot.runUsingPID(robot.cmToTicks(-50), 1, 5);
//        robot.rotateConstantSpeed(-139, 0.4, 3);
//        //Se duce dupa al doilea
//        robot.runUsingPID(robot.cmToTicks(230),1,7);
//
//        //Prinde al doilea wobble
//        robot.servoWobble.setPosition(0);
//        sleep(500);
////        robot.servoRidicare.setPosition(0.8);
//
//        //Se intoarce catre patrat
//        robot.rotateConstantSpeed(168, 0.4, 5);
//
//        //Duce al doilea wobble
//        robot.runUsingPID(robot.cmToTicks(262),1,7);
//
//
//        //Lasa al doilea wobble
//        robot.servoWobble.setPosition(0.8);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        //Parcheaza
//        robot.runUsingPID(robot.cmToTicks(-85),1,5);
//    }
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        robot = new RobotMap(hardwareMap, this);
//        robot.servoWobble.setPosition(0);
//        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
//        robot.lansareRing.setPosition(0.5);
//        robot.zeroPowerBeh();
//
//        waitForStart();
//
//        if(opModeIsActive()) {
//
//        }
//    }
//}
