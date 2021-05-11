package org.firstinspires.ftc.teamcode.odometryNew;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMap;

/**
 * Created by Sarthak on 10/4/2019.
 */
@Config
@TeleOp(name = "My Odometry OpMode")
public class MyOdometryOpmode extends LinearOpMode {

    RobotMap robot;
    FtcDashboard dashboard;

    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0.3,1,0);
    ElapsedTime PIDTimer = new ElapsedTime();

    final double COUNTS_PER_INCH = 194.04;//1440 ticks, 6cm diametru

    public static double targetX = 0;
    public static double targetY = 0;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
//    String rfName = "dreaptaFata", rbName = "dreaptaSpate", lfName = "stangaFata", lbName = "stangaSpate";
//    String verticalRight = "encoderStanga", verticalLeft = "encoderDreapta", horizontalEncoderName = "motorIntake";

    OdometryGlobalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        robot = new RobotMap(hardwareMap, this);
        PIDTimer.reset();
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
//        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(robot.encoderDreapta, robot.rotite, robot.motorIntake, robot.imu, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseLeftEncoder();
        globalPositionUpdate.reverseRightEncoder();
//        globalPositionUpdate.reverseNormalEncoder();

//        goToPosition(targetX * COUNTS_PER_INCH, targetY * COUNTS_PER_INCH, 0.4, 0, 3 * COUNTS_PER_INCH);
//        goToPosition(24 * COUNTS_PER_INCH, 24 * COUNTS_PER_INCH, 0.4, 0, 1);
//        goToPosition(0 * COUNTS_PER_INCH, 0 * COUNTS_PER_INCH, 0.4, 0, 1);

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double rotation = -gamepad1.right_stick_x;

            robot.teleOpDrive(forward, rotation);

            double [] pose = {globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH, globalPositionUpdate.returnRadiansOrientation()};

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", globalPositionUpdate.returnOrientation());

            telemetry.addData("Vertical left encoder position", robot.encoderDreapta.getCurrentPosition() * globalPositionUpdate.verticalLeftEncoderPositionMultiplier);
            telemetry.addData("Vertical right encoder position", robot.rotite.getCurrentPosition() * globalPositionUpdate.verticalRightEncoderPositionMultiplier);
            telemetry.addData("horizontal encoder position", robot.motorIntake.getCurrentPosition() * globalPositionUpdate.normalEncoderPositionMultiplier);

            telemetry.addData("Thread Active", positionThread.isAlive());
            telemetry.update();
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    public void odometryPID(int distance, PIDCoefficients pidCoefficients) {
//        dreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
//        dreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
//        double integral = 0;
//        ArrayList<DcMotor> motors = new ArrayList<>();
//        motors.add(stangaFata);
//        motors.add(stangaSpate);
//        motors.add(dreaptaSpate);
//        motors.add(dreaptaFata);
//        PIDTimer.reset();
//        runtime.reset();
//
//        for (DcMotor motor : motors) {
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        double globalError = distance;
//        double errorsSum = 0;
//        double[] lastErrors = new double[5];
//
////        telemetry.addData("Global error: ", robot.stangaSpate.getCurrentPosition());
////        telemetry.update();
//        while (Math.abs(globalError) > 100 && opMode.opModeIsActive()) {
//            dashboardTelemetry.addData("Global error: ", globalError);

//            for (DcMotor motor: motors) {
//                //Getting the index of the motor
//                int index = motors.indexOf(motor);
//
//                //Calculating the errors of the motor
//
//
//                //Setting the power of the motor
//                motor.setPower(P + I + D);
////                telemetry.addData(String.format("Motor %d", motors.indexOf(motor)), motor.getPower());
//
//                //Updating the errors and timer for the next loop
//                lastErrors[index] = error;
//                errorsSum += error;
//                PIDTimer.reset();
//            }
//            globalError = errorsSum / 4;
//            telemetry.addData("Updated global error: ", globalError);
//            telemetry.update();
        }
//        dreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
//        dreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
//    }


    public void rotateWithOdometry(double desiredAngle) {
        double error = globalPositionUpdate.returnOrientation() - desiredAngle;
        while (opModeIsActive() && Math.abs(error) > 10) {
            error = globalPositionUpdate.returnOrientation() - desiredAngle;
            double power = -Math.sin(Math.toRadians(error));

            robot.stangaFata.setPower(power);
            robot.stangaSpate.setPower(power);
            robot.dreaptaSpate.setPower(power);
            robot.dreaptaFata.setPower(power);
        }
        robot.stopDriving();
    }

    public void goToPosition(double targetXPosition, double targetYPosition, double power, double desiredRobotOrientation, double allowableDistanceError) {
        double distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();
        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        while (opModeIsActive() && Math.abs(distance) > allowableDistanceError) {

            distance = Math.hypot(distanceToXTarget, distanceToYTarget);
            telemetry.addData("Distance: ", distance);
            telemetry.update();

            distanceToXTarget = targetXPosition - globalPositionUpdate.returnXCoordinate();
            distanceToYTarget = targetYPosition - globalPositionUpdate.returnYCoordinate();

            double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

//            double robotMovementXComponent = calculateX(robotMovementAngle, power);
            double robotMovementYComponent = calculateY(robotMovementAngle, power);
            double pivotCorrection = desiredRobotOrientation - globalPositionUpdate.returnOrientation();

//            robot.odometryMovementTest(power, robotMovementAngle);
        }
        rotateWithOdometry(desiredRobotOrientation);

    }

//    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName) {
//        right_front = hardwareMap.dcMotor.get(rfName);
//        right_back = hardwareMap.dcMotor.get(rbName);
//        left_front = hardwareMap.dcMotor.get(lfName);
//        left_back = hardwareMap.dcMotor.get(lbName);
//
//        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
//        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
//        horizontal = hardwareMap.dcMotor.get(hEncoderName);
//
//        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        telemetry.addData("Status", "Hardware Map Init Complete");
//        telemetry.update();
//    }

    /**
     * Calculate the power in the x direction
     *
     * @param desiredAngle angle on the x axis
     * @param speed        robot's speed
     * @return the x vector
     */
    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }

    /**
     * Calculate the power in the y direction
     *
     * @param desiredAngle angle on the y axis
     * @param speed        robot's speed
     * @return the y vector
     */
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }
}
