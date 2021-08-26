package org.firstinspires.ftc.teamcode.odometryNew;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotMap;
@Disabled
@Autonomous
public class OdometryAutoTest extends LinearOpMode
{

    RobotMap robot;
    OdometryGlobalCoordinatePosition globalCoordinatePosition;
    private final double COUNTS_PER_INCH = 194.04;
    ElapsedTime runtime = new ElapsedTime();

    private final double TOWER_GOAL_HEIGHT = 0.57;

    int counts = 0;
    boolean detected = false;

    private double AngleWrap(double angle) {

        while (angle < Math.PI) {
            angle += (2*Math.PI);
        }
        while (angle > Math.PI) {
            angle -= (2*Math.PI);
        }
        return angle;

    }

    public void goToPosition(double x, double y, double prefferedAngle, double movementSpeed, double turnSpeed, int reverse, boolean precise, double timeout) {

        double worldXPosition = globalCoordinatePosition.returnXCoordinate();
        double worldYPosition = globalCoordinatePosition.returnYCoordinate();
        double worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        while (distanceToTarget > 10 && opModeIsActive()) {
            worldXPosition = globalCoordinatePosition.returnXCoordinate() / COUNTS_PER_INCH;
            worldYPosition = globalCoordinatePosition.returnYCoordinate() / COUNTS_PER_INCH;
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            if (reverse == -1) {
                worldAngularOrientation -= Math.toRadians(180);
            }

            distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
            double absoluteAngleToTarget = AngleWrap(Math.atan2(x - worldXPosition, y - worldYPosition));

            double relativeAngle = AngleWrap(worldAngularOrientation - absoluteAngleToTarget);
            double movementTurn = -Range.clip(Range.clip(relativeAngle, -1, 1) * turnSpeed, -1, 1);

            if (distanceToTarget < 6) {
                movementTurn = 0;
            }

            double movementY = Range.clip(distanceToTarget, -1, 1) * movementSpeed * reverse;

            robot.odometryMovement(movementY, movementTurn);

            telemetry.addData("X: ", worldXPosition);
            telemetry.addData("Y: ", worldYPosition);
            telemetry.addData("Orientation: ", Math.toDegrees(worldAngularOrientation));
            telemetry.addData("Distance to target: ", distanceToTarget);
            telemetry.addData("Absolute angle: ", Math.toDegrees(absoluteAngleToTarget));
            telemetry.addData("Relative angle: ", Math.toDegrees(relativeAngle));
            telemetry.addData("Rotation power: ", movementTurn);
            telemetry.addData("Forward power: ", movementY);
            telemetry.update();


        }

        worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();
        double angle = precise ? 1 : 7;
        runtime.reset();
        while (Math.abs(AngleWrap(worldAngularOrientation - prefferedAngle)) > Math.toRadians(angle) && opModeIsActive() && runtime.seconds() < timeout) {
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            double relativeAngleToPoint = AngleWrap(prefferedAngle - worldAngularOrientation);
            double movementTurn = Range.clip(relativeAngleToPoint, -1, 1);

            if (!precise) {
                movementTurn *= turnSpeed;
            }
            else {
                movementTurn *= 0.08;
            }

            robot.odometryMovement(0, movementTurn);
        }

        robot.stopDriving();

    }

    public void pickRingsOdometry(double x, double y, double prefferedAngle, double movementSpeed, double turnSpeed, int reverse, boolean precise, double timeout) {

        double worldXPosition = globalCoordinatePosition.returnXCoordinate();
        double worldYPosition = globalCoordinatePosition.returnYCoordinate();
        double worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

        boolean hasSlept = false;

        while (distanceToTarget > 10 && opModeIsActive()) {
            worldXPosition = globalCoordinatePosition.returnXCoordinate() / COUNTS_PER_INCH;
            worldYPosition = globalCoordinatePosition.returnYCoordinate() / COUNTS_PER_INCH;
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            if ((robot.senzorDreapta.getNormalizedColors().toColor() < 0 || robot.senzorStanga.getNormalizedColors().toColor() < 0) && !detected) {
                counts ++;
                detected = true;
            }
            else if (robot.senzorDreapta.getNormalizedColors().toColor() > 0 && robot.senzorStanga.getNormalizedColors().toColor() > 0) {
                detected = false;
            }

            if (counts > 3) {
                robot.motorIntake.setPower(0);
                robot.rotite.setPower(-1);
//                counts = 0;
            }

            if (reverse == -1) {
                worldAngularOrientation -= Math.toRadians(180);
            }

            distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);
            double absoluteAngleToTarget = AngleWrap(Math.atan2(x - worldXPosition, y - worldYPosition));

            double relativeAngle = AngleWrap(worldAngularOrientation - absoluteAngleToTarget);
            double movementTurn = -Range.clip(Range.clip(relativeAngle, -1, 1) * turnSpeed, -1, 1);

            if (distanceToTarget < 6) {
                movementTurn = 0;
            }

            double movementY = Range.clip(distanceToTarget, -1, 1) * movementSpeed * reverse;

            if (distanceToTarget < 42 && !hasSlept) {
                hasSlept = true;
                robot.stopDriving();
                sleep(850);
            }

            robot.odometryMovement(movementY, movementTurn);


        }

        worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();
        double angle = precise ? 2 : 7;
        runtime.reset();
        while (Math.abs(AngleWrap(worldAngularOrientation - prefferedAngle)) > Math.toRadians(angle) && opModeIsActive() && runtime.seconds() < timeout) {
            worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

            double relativeAngleToPoint = AngleWrap(prefferedAngle - worldAngularOrientation);
            double movementTurn = Range.clip(relativeAngleToPoint, -1, 1);

            if ((robot.senzorDreapta.getNormalizedColors().toColor() < 0 || robot.senzorStanga.getNormalizedColors().toColor() < 0) && !detected) {
                counts ++;
                detected = true;
            }
            else if (robot.senzorDreapta.getNormalizedColors().toColor() > 0 && robot.senzorStanga.getNormalizedColors().toColor() > 0) {
                detected = false;
            }

            if (counts > 3) {
                robot.motorIntake.setPower(0);
                robot.rotite.setPower(-1);
//                counts = 0;
            }

            if (!precise) {
                movementTurn *= turnSpeed;
            }
            else {
                movementTurn *= 0.08;
            }

            robot.odometryMovement(0, movementTurn);
        }

        robot.stopDriving();

    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalCoordinatePosition = new OdometryGlobalCoordinatePosition(robot.encoderDreapta, robot.rotite, robot.motorIntake, robot.imu, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalCoordinatePosition);
        positionThread.start();
        globalCoordinatePosition.reverseLeftEncoder();
        globalCoordinatePosition.reverseRightEncoder();
        globalCoordinatePosition.reverseNormalEncoder();

        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();

        robot.servoWobble.setPosition(0);

        waitForStart();

//        robotMovement = new RobotMovement(robot, globalCoordinatePosition);

        waitForStart();

        if (opModeIsActive()) {
            robot.motorShooter.setPower(1);
            robot.servoRidicare.setPosition(0.2);
            goToPosition(0, 58, -Math.toRadians(9), 0.8, 0.3, 1, true, 1);
            robot.shoot3Rings();
            robot.motorShooter.setPower(0);
            goToPosition(-30, 105, Math.toRadians(-40), 0.8, 0.3, 1, false, 2);
            robot.servoWobble.setPosition(0.8);
            robot.servoRidicare.setPosition(0);
            sleep(500);
            goToPosition(0, 60, -Math.toRadians(145), 0.65, 0.3, -1, true, 2);
            robot.motorIntake.setPower(1);
            robot.rotite.setPower(1);
            robot.ridicareShooter.setPosition(1);
//            pickRingsOdometry(-25, 5.55, Math.toRadians(145), 0.035, 0.2, 1, false, 0);
//            sleep(300);
            pickRingsOdometry(-36, 8, -Math.toRadians(145), 0.025, 0.2, 1, false, 0);
            robot.servoWobble.setPosition(0);
            sleep(750);
            robot.servoRidicare.setPosition(0.5);
            robot.motorShooter.setPower(1);
            if (counts > 3) {
                robot.rotite.setPower(-1);
            }
            robot.motorIntake.setPower(1);
            robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
            goToPosition(0, 70, -Math.toRadians(10), 0.6, 0.4, -1, true, 3);
            robot.servoRidicare.setPosition(0.2);
            sleep(550);
//            robot.motorIntake.setPower(1);
            robot.shoot3Rings();
            //oprim toate motoarele
            robot.motorShooter.setPower(0);
            robot.motorIntake.setPower(0);
            robot.rotite.setPower(0);
            goToPosition(-24, 100, Math.toRadians(-40), 0.8, 0.4, 1, false, 0);
            robot.servoWobble.setPosition(0.8);
            robot.servoRidicare.setPosition(0);
            sleep(500);
            goToPosition(0, 75, Math.toRadians(0), 1, 0.4, -1, true, 2);



//            goToPosition(30, 30, Math.toRadians(0), 0.1, 0.1, 1, false);
//            telemetry.addData("X: ", globalCoordinatePosition.returnXCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Y: ", globalCoordinatePosition.returnYCoordinate() / COUNTS_PER_INCH);
//            telemetry.addData("Orientation: ", globalCoordinatePosition.returnOrientation());
//            telemetry.update();
//            sleep(5000);

        }

    }
}
