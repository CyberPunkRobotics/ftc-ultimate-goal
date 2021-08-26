/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.odometryNew.OdometryGlobalCoordinatePosition;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
@Autonomous(name="Autonomie finala")
public class EasyOpenCvTesting extends LinearOpMode
{
    OpenCvCamera webcam;
    SkystoneDeterminationPipeline pipeline;
    private RobotMap robot;
    private OdometryGlobalCoordinatePosition globalCoordinatePosition;
    private final double COUNTS_PER_INCH = 194.04;
    private ElapsedTime runtime = new ElapsedTime();

    boolean hasSlept = false;

    /*
     * The core values which define the location and size of the sample regions
     */
    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(182,140);

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    public static int REGION_WIDTH = 45;
    public static int REGION_HEIGHT = 35;

    public static int FOUR_RING_THRESHOLD = 140;
    public static int ONE_RING_THRESHOLD = 132;

    private final double TOWER_GOAL_HEIGHT = 0.44;

    private int counts = 0;
    private boolean detected = false;

    private boolean backPowerIntake;

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
                movementTurn *= 0.072;
            }

            robot.odometryMovement(0, movementTurn);
        }

        robot.stopDriving();

    }

    public void goToPositionMecanum(double x, double y, double prefferedAngle, double movementSpeed, double turnSpeed, int reverse, boolean precise, double timeout) {

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

            double relativeXToPoint = Math.cos(relativeAngle) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngle) * distanceToTarget;

            double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint));
            double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) * Math.abs(relativeYToPoint));

            robot.mecanumMovement(movementXPower, movementYPower, 0);

//            double movementY = Range.clip(distanceToTarget, -1, 1) * movementSpeed * reverse;

//            robot.odometryMovement(movementY, movementTurn);

            telemetry.addData("X: ", worldXPosition);
            telemetry.addData("Y: ", worldYPosition);
            telemetry.addData("Orientation: ", Math.toDegrees(worldAngularOrientation));
            telemetry.addData("Distance to target: ", distanceToTarget);
            telemetry.addData("Absolute angle: ", Math.toDegrees(absoluteAngleToTarget));
            telemetry.addData("Relative angle: ", Math.toDegrees(relativeAngle));
            telemetry.addData("Rotation power: ", movementTurn);
            telemetry.addData("X power: ", movementXPower);
            telemetry.addData("Y Power", movementYPower);
            telemetry.update();


        }
    }

    public void pickRingsOdometry(double x, double y, double prefferedAngle, double movementSpeed, double turnSpeed, int reverse, boolean precise, double timeout) {

        double worldXPosition = globalCoordinatePosition.returnXCoordinate();
        double worldYPosition = globalCoordinatePosition.returnYCoordinate();
        double worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();

        double distanceToTarget = Math.hypot(x - worldXPosition, y - worldYPosition);

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
                robot.rotite.setPower(0);
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

            if (backPowerIntake && runtime.seconds() > 1) {
                backPowerIntake = false;
                robot.rotite.setPower(-1);
            }

            double movementY = Range.clip(distanceToTarget, -1, 1) * movementSpeed * reverse;

            if (distanceToTarget < 42 && !hasSlept) {
                hasSlept = true;
                robot.stopDriving();
                sleep(850);
            }

            telemetry.addData("Counts: ", counts);
            telemetry.update();

            robot.odometryMovement(movementY, movementTurn);


        }

        worldAngularOrientation = globalCoordinatePosition.returnRadiansOrientation();
        double angle = precise ? 1 : 7;
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
                robot.rotite.setPower(0);
//                counts = 0;
            }

            if (!precise) {
                movementTurn *= turnSpeed;
            }
            else {
                movementTurn *= 0.07;
            }

            robot.odometryMovement(0, movementTurn);
        }

        robot.stopDriving();

    }

    @Override
    public void runOpMode()
    {
        robot = new RobotMap(hardwareMap, this);
        robot.servoWobble.setPosition(0);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.lansareRing.setPosition(0.5);
        robot.zeroPowerBeh();

        globalCoordinatePosition = new OdometryGlobalCoordinatePosition(robot.encoderDreapta, robot.rotite, robot.motorIntake, robot.imu, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalCoordinatePosition);
        positionThread.start();
        globalCoordinatePosition.reverseLeftEncoder();
        globalCoordinatePosition.reverseRightEncoder();
        globalCoordinatePosition.reverseNormalEncoder();

        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);


        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
                dashboard.startCameraStream(webcam, 0);
            }
        });

        while (!opModeIsActive() && !isStopRequested()) {
            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
            dashboardTelemetry.addData("Position", pipeline.position);
            dashboardTelemetry.update();

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();
        }

        waitForStart();

        if (opModeIsActive())
        {

//            pipeline.updateRegions();
//
//            dashboardTelemetry.addData("Analysis", pipeline.getAnalysis());
//            dashboardTelemetry.addData("Position", pipeline.position);
//            dashboardTelemetry.update();
//
//            // Don't burn CPU cycles busy-looping in this sample
//            sleep(50);



            if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.FOUR) {
                FourRingsOdometry();
            }
            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.ONE) {
                OneRing();
            }
            else if (pipeline.position == SkystoneDeterminationPipeline.RingPosition.NONE) {
                NoRing();
            }
        }
    }

    private void FourRingsOdometry() {
        robot.motorShooter.setPower(1);
        robot.servoRidicare.setPosition(0.2);
        goToPosition(0, 57, -Math.toRadians(11), 0.65, 0.3, 1, true, 1);
        robot.shoot3Rings();
        robot.motorShooter.setPower(0);
        goToPosition(-30, 105, Math.toRadians(-40), 0.8, 0.3, 1, false, 2);
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(0);
        sleep(500);
        goToPosition(-7, 50, -Math.toRadians(145), 0.5, 0.3, -1, true, 1.5);
        robot.motorIntake.setPower(1);
        robot.rotite.setPower(1);
        robot.ridicareShooter.setPosition(1);
//            pickRingsOdometry(-25, 5.55, Math.toRadians(145), 0.035, 0.2, 1, false, 0);
//            sleep(300);
        hasSlept = false;
        pickRingsOdometry(-35, 12, -Math.toRadians(145), 0.035, 0.2, 1, false, 0);
        robot.servoWobble.setPosition(0);
        sleep(750);
        robot.servoRidicare.setPosition(0.5);
//        robot.motorShooter.setPower(1);
//        if (counts > 3) {
//            robot.rotite.setPower(-1);
//        }
        robot.motorIntake.setPower(1);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        backPowerIntake = true;
        counts = 0;
        robot.motorShooter.setPower(1);
        runtime.reset();
        pickRingsOdometry(0, 70, -Math.toRadians(12), 0.4, 0.4, -1, true, 3);
        robot.servoRidicare.setPosition(0.2);
        sleep(550);
//            robot.motorIntake.setPower(1);

        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);

        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);

        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);

        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);

        //oprim toate motoarele
        robot.motorShooter.setPower(0);
        robot.motorIntake.setPower(0);
        robot.rotite.setPower(0);
        goToPosition(-30, 105, Math.toRadians(-40), 0.8, 0.4, 1, false, 0);
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(0);
        sleep(500);
        goToPosition(0, 77, Math.toRadians(0), 1, 0.4, -1, true, 2);
    }

    private void pickRings(int distance, double power, double timeout) {
                    ElapsedTime runtime = new ElapsedTime();
                    double coefficient = 2000;

                    robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                    sleep(50);

                    robot.stangaFata.setTargetPosition(distance);
                    robot.stangaSpate.setTargetPosition(distance);
                    robot.dreaptaFata.setTargetPosition(-distance);
                    robot.dreaptaSpate.setTargetPosition(-distance);

                    robot.stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    robot.dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    double p = 0.05;

                    robot.stangaFata.setVelocity(p * coefficient);
                    robot.stangaSpate.setVelocity(p * coefficient);
                    robot.dreaptaFata.setVelocity(p * coefficient);
                    robot.dreaptaSpate.setVelocity(p * coefficient);

                    runtime.reset();

                    while (opModeIsActive() && robot.stangaSpate.isBusy() && robot.stangaFata.isBusy() && robot.dreaptaSpate.isBusy() && robot.dreaptaFata.isBusy() && runtime.seconds() < timeout) {
                        if (p < power) p += 0.05;

                        robot.stangaFata.setVelocity(p * coefficient);
                        robot.stangaSpate.setVelocity(p * coefficient);
                        robot.dreaptaFata.setVelocity(p * coefficient);
                        robot.dreaptaSpate.setVelocity(p * coefficient);

                        if ((robot.senzorDreapta.getNormalizedColors().toColor() < 0 || robot.senzorStanga.getNormalizedColors().toColor() < 0) && !detected) {
                            counts ++;
                detected = true;
            }
            else if (robot.senzorDreapta.getNormalizedColors().toColor() > 0 && robot.senzorStanga.getNormalizedColors().toColor() > 0) {
                detected = false;
            }

            if (counts > 3) {
                robot.motorIntake.setPower(0);
                robot.rotite.setPower(0);
                counts = 0;
            }

            if (runtime.seconds() > 2.5 && backPowerIntake) {
                robot.rotite.setPower(-1);
            }

            telemetry.addData("Counts: ", counts);
            telemetry.update();
        }

        robot.stopDriving();
    }

    private void startAndShoot() {
        //Porneste motoarele
        robot.servoRidicare.setPosition(0.2);
        robot.motorShooter.setPower(1);
        webcam.closeCameraDevice();
//        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);

        //Se pozitioneaza la tragere
        robot.runUsingEncodersLongRun(robot.cmToTicks(156), 1, 4);

        robot.rotate(-9, 0.3, 3);
//
        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);
        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        sleep(350);
        robot.lansareRing.setPosition(1);
        sleep(350);
        robot.lansareRing.setPosition(0.5);
        //Opreste shooterul
        robot.motorShooter.setPower(0);
    }

    private void FourRings() {


        startAndShoot();
        //Se pozitioneaza pentru wobble si il duce
        robot.rotate(-10, 0.4, 2);
        robot.runUsingEncodersLongRun(robot.cmToTicks(175), 1, 5);
////
////        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(0);
        sleep(200);
////
////        //Se intoarce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(-168), 1, 5);
        robot.rotate(-113, 0.4, 4);
////        //Se duce dupa al doilea
        robot.motorIntake.setPower(1);
        robot.ridicareShooter.setPosition(1);
        //robot.servoRidicare.setPosition(1);
////        robot.runUsingEncodersLongRun(robot.cmToTicks(100),0.35,7);
////        sleep(200);
////        robot.runUsingEncoders(robot.cmToTicks(60), 0.3, 4);
        robot.rotite.setPower(1);
        pickRings(robot.cmToTicks(100), 0.4, 7);
        sleep(250);
        pickRings(robot.cmToTicks(60), 0.3, 4);

        robot.motorIntake.setPower(1);
        robot.servoWobble.setPosition(0);
        sleep(750);
        robot.motorIntake.setPower(0);
        robot.servoRidicare.setPosition(1);
        robot.rotite.setPower(-1);
//
        robot.runUsingEncoders(robot.cmToTicks(-30), 1, 4);
        //robot.motorIntake.setPower(1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(-125), 1, 5);
        robot.rotite.setPower(0);
        robot.motorIntake.setPower(1);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        //robot.motorIntake.setPower(0);
        robot.motorShooter.setPower(1);
        robot.servoRidicare.setPosition(0);
        robot.rotate(126, 0.4, 6);//aci
        robot.shoot3Rings();
        robot.motorIntake.setPower(0);
        robot.servoRidicare.setPosition(0.5);
        robot.motorShooter.setPower(0);
        robot.rotate(-13, 0.4, 2);
        robot.runUsingEncodersLongRun(robot.cmToTicks(145), 1, 5);
        robot.servoWobble.setPosition(0.8);
        sleep(150);
//        robot.runUsingEncodersLongRun(robot.cmToTicks(-120), 1, 5);
        robot.runUsingEncoders(robot.cmToTicks(-85), 1, 5);



//        robot.runUsingEncoders(robot.cmToTicks(35), 1, 3);
//        robot.rotateConstantSpeed(90, 0.4, 2);
//        robot.servoWobble.setPosition(0);
//        sleep(500);
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
//        robot.runUsingEncodersLongRun(robot.cmToTicks(262),1,7);
//
//
//        //Lasa al doilea wobble
//        robot.servoWobble.setPosition(0.8);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        //Parcheaza
//        robot.runUsingEncodersLongRun(robot.cmToTicks(-85),1,5);
    }

    private void NoRing(){

        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        //robot.runUsingEncodersLongRun(robot.cmToTicks(20),1,3);
        robot.rotate(-55,0.4,3);
        robot.runUsingEncodersLongRun(robot.cmToTicks(104),0.8,3);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(0);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(-39), 0.8, 2);
        robot.rotate(-91, 0.3, 3);
//
//        //Se duce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(160),0.5,7);
        //robot.rotite.setPower(1);
//
//        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
        robot.servoRidicare.setPosition(0.2);
//
////        //Se intoarce catre patrat
//        robot.rotateConstantSpeed(160, 0.4, 5);
////
////        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(-170),1,7);
        robot.rotate(70, 0.4, 5);
        //robot.rotite.setPower(0);
        robot.runUsingEncodersLongRun(robot.cmToTicks(20), 0.8, 2);
//        robot.runUsingEncoders(robot.cmToTicks(15),1,3);

//
//
//        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
        sleep(500);

        robot.runUsingEncodersLongRun(robot.cmToTicks(-40),1,3);
////        robot.servoRidicare.setPosition(1);
//        sleep(500);
//
//        robot.runUsingEncoders(-robot.cmToTicks(30), 1, 3);
//        robot.rotateConstantSpeed(60, 0.4, 3);
//        robot.runUsingEncoders(robot.cmToTicks(45), 1, 3);

    }

    private void OneRing(){
        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        robot.rotate(-9,0.4,2);
        robot.runUsingEncodersLongRun(robot.cmToTicks(95),1,2);
//
//        //Lasat primul wobble
        robot.servoWobble.setPosition(0.8);
        robot.servoRidicare.setPosition(0);
        sleep(500);
//
//        //Mers dupa al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(-115),1,2);
        robot.rotate(-110,0.3,3);
        robot.rotite.setPower(1);
        robot.motorIntake.setPower(1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(100),1,6);
        robot.runUsingEncodersLongRun(robot.cmToTicks(45), 0.5, 4);
//
//        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
        robot.servoRidicare.setPosition(0.4);
////
////        //Se intoarce catre patrat
////        robot.rotateConstantSpeed(158, 0.65, 5);
//
//        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(-85),1,7);
//        //Se intoarce catre patrat
        robot.rotate(127, 0.3, 5);
        robot.rotite.setPower(0);
        robot.motorShooter.setPower(1);
        robot.runUsingEncodersLongRun(robot.cmToTicks(65),1,7);
        robot.lansareRing.setPosition(1);
        sleep(400);
        robot.lansareRing.setPosition(0.5);
        robot.motorIntake.setPower(0);
        robot.motorShooter.setPower(0);
        robot.servoRidicare.setPosition(0.5);
        robot.rotate(-3, 0.3, 4);

        //robot.rotate(6, 0.4, 5);
//
//
//        //Lasa al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(60),1,7);
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);
//
//        //Parcare
        robot.runUsingEncodersLongRun(robot.cmToTicks(-35),1,3);


    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum RingPosition
        {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 0, 0, 0);




        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        public void updateRegions() {
            region1_pointA = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x,
                    REGION1_TOPLEFT_ANCHOR_POINT.y);
            region1_pointB = new Point(
                    REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                    REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        }

        /*
         * Working variables
         */
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        // Volatile since accessed by OpMode thread w/o synchronization
        private volatile RingPosition position = RingPosition.FOUR;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            position = RingPosition.FOUR; // Record our analysis
            if(avg1 > FOUR_RING_THRESHOLD){
                position = RingPosition.FOUR;
            }else if (avg1 > ONE_RING_THRESHOLD){
                position = RingPosition.ONE;
            }else{
                position = RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            return input;
        }

        public int getAnalysis()
        {
            return avg1;
        }
    }
}