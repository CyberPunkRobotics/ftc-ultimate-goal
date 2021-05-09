package org.firstinspires.ftc.teamcode;

import android.sax.StartElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.odometryNew.OdometryGlobalCoordinatePosition;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;


public class RobotMap {

    public DcMotorEx stangaSpate = null;
    public DcMotorEx dreaptaSpate = null;
    public DcMotorEx stangaFata = null;
    public DcMotorEx dreaptaFata = null;
    public DcMotor motorShooter = null;
    public DcMotor motorIntake = null; //are montat encoderul din mijloc
    public DcMotor leftEncoder, rightEncoder;
    public Servo servoWobble, servoRidicare, lansareRing = null;
    public Servo ridicareShooter = null;
    public Servo servoBratStanga = null, servoBratDreapta = null;
    public CRServo rotireIntake = null;
    public DcMotorEx encoderStanga, rotite;
    public BNO055IMU imu = null;
    public RevColorSensorV3 senzorStanga, senzorDreapta;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction, prevAngle;
    private LinearOpMode opMode;
    private double imuOffset = 0;
    private double maxSpeed = 1.0;
    ElapsedTime PIDTimer = new ElapsedTime();
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0, 0);
    ElapsedTime runtime = new ElapsedTime();
    final double COUNTS_PER_INCH = 194.04;//1440 ticks, 6cm diametru
    public OdometryGlobalCoordinatePosition globalPositionUpdate;
    public Thread positionThread;
    double coefficient = 2000;


    public RobotMap (HardwareMap hardwareMap, LinearOpMode opMode) {

        stangaFata = hardwareMap.get(DcMotorEx.class, "stangaFata");
        dreaptaFata = hardwareMap.get(DcMotorEx.class, "dreaptaFata");
        stangaSpate = hardwareMap.get(DcMotorEx.class, "stangaSpate");
        dreaptaSpate = hardwareMap.get(DcMotorEx.class, "dreaptaSpate");
        servoWobble = hardwareMap.get(Servo.class, "servoWobble");
        servoRidicare = hardwareMap.get(Servo.class, "servoRidicare"); // ridicare wobble
        lansareRing = hardwareMap.get(Servo.class, "lansareShooter");
        motorShooter = hardwareMap.get(DcMotor.class, "motorShooter");
        motorIntake = hardwareMap.get(DcMotor.class, "motorIntake");
        ridicareShooter = hardwareMap.servo.get("ridicareShooter");
//        rotireIntake = hardwareMap.crservo.get("rotireIntake");
        encoderStanga = hardwareMap.get(DcMotorEx.class, "encoderStanga");
        rotite = hardwareMap.get(DcMotorEx.class, "rotite");
        servoBratDreapta = hardwareMap.get(Servo.class, "servoBratDreapta");
        servoBratStanga = hardwareMap.get(Servo.class, "servoBratStanga");
        senzorStanga = hardwareMap.get(RevColorSensorV3.class, "senzorFata");
        senzorDreapta = hardwareMap.get(RevColorSensorV3.class, "senzorSpate");


        encoderStanga.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotite.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        encoderStanga.setDirection(DcMotorSimple.Direction.REVERSE);
        rotite.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        rotite.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoderStanga.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        globalPositionUpdate = new OdometryGlobalCoordinatePosition(encoderStanga, rotite, motorIntake, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();
//        globalPositionUpdate.reverseNormalEncoder();
        globalPositionUpdate.reverseLeftEncoder();

        servoRidicare.setPosition(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        while (imu.isGyroCalibrated());

        this.opMode = opMode;
    }

    public void resetOdometryEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runUsingPID(int distance, double power, double timeout, PIDCoefficients pidCoefficients) {
        dreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        double integral = 0;
        ArrayList<DcMotor> motors = new ArrayList<>();
        motors.add(stangaFata);
        motors.add(stangaSpate);
        motors.add(dreaptaSpate);
        motors.add(dreaptaFata);
        PIDTimer.reset();
        runtime.reset();

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double globalError = distance;
        double errorsSum = 0;
        double[] lastErrors = new double[5];

//        telemetry.addData("Global error: ", robot.stangaSpate.getCurrentPosition());
//        telemetry.update();
        while (Math.abs(globalError) > 100 && runtime.seconds() < timeout && opMode.opModeIsActive()) {
//            dashboardTelemetry.addData("Global error: ", globalError);
            errorsSum = 0;
            for (DcMotor motor: motors) {
                //Getting the index of the motor
                int index = motors.indexOf(motor);

                //Calculating the errors of the motor
                double error = distance - motor.getCurrentPosition();
                double delta = lastErrors[index] - error;

                //Calculating the integral and derivative terms
                integral += delta * PIDTimer.time();
                double derivative = delta/PIDTimer.time();

                //Multiplying the errors with the PID coefficients
                double P = pidCoefficients.p * error;
                double I = pidCoefficients.i * integral;
                double D = pidCoefficients.d * derivative;

                //Setting the power of the motor
                motor.setPower(P + I + D);
//                telemetry.addData(String.format("Motor %d", motors.indexOf(motor)), motor.getPower());

                //Updating the errors and timer for the next loop
                lastErrors[index] = error;
                errorsSum += error;
                PIDTimer.reset();
            }
            globalError = errorsSum / 4;
//            telemetry.addData("Updated global error: ", globalError);
//            telemetry.update();
        }
        dreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void driveFieldRelative3(double x, double y, double rotate) {

        double r = Math.sqrt(x * x + y * y);
        double alfa = Math.toRadians(getAngle());

        double delta = alfa - Math.atan2(y, x);

        double deltaPrim = normalizeRadians(alfa + Math.PI) - Math.atan2(y, x);

        opMode.telemetry.addData("Alpha: ", alfa);
        opMode.telemetry.addData("Delta: ", delta);
        opMode.telemetry.addData("Delta prim: ", deltaPrim);
        opMode.telemetry.update();

        if(Math.abs(delta)<=Math.PI/4)
            teleOpDrive(r,rotate);
        if(Math.abs(deltaPrim)<=Math.PI/4)
            teleOpDrive(-r,rotate);
    }

    public void driveFieldRelativeAngle(double y, double angle) {
        double angle_in = angle + Math.PI;  // convert to robot coordinates
//        opMode.telemetry.addData("angle_in: ", angle_in);
//        opMode.telemetry.addData("prevAngle: ", prevAngle);
//        opMode.telemetry.update();

        double delta = normalizeRadians(Math.toRadians(getAngle()) - angle_in);

        double MAX_ROTATE = 0.3; //This is to shrink how fast we can rotate so we don't fly past the angle
//        if (y <= 0.05) {
//            delta = 0;
//        }
//        else {
////            delta = Range.clip(delta, -MAX_ROTATE, MAX_ROTATE);
//            delta *= 0.4;
//        }
        if (y <= 0.05) {
            delta = 0;
        }
        else if (y < 0.5) {
            y /= 1.5;
            delta *= 0.3;
        }
        else {
            delta *= 0.3;
        }
        teleOpDrive(y, delta);
    }

    public void odometryPID(int distance, PIDCoefficients pidCoefficients) {
        dreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        dreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        double integral = 0;
        ArrayList<DcMotor> motors = new ArrayList<>();
        motors.add(stangaFata);
        motors.add(stangaSpate);
        motors.add(dreaptaSpate);
        motors.add(dreaptaFata);
        PIDTimer.reset();
        runtime.reset();

        for (DcMotor motor: motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        double globalError = distance;
        double errorsSum = 0;
        double[] lastErrors = new double[5];

//        telemetry.addData("Global error: ", robot.stangaSpate.getCurrentPosition());
//        telemetry.update();
        while (Math.abs(globalError) > 100 && opMode.opModeIsActive()) {
//            dashboardTelemetry.addData("Global error: ", globalError);

//            for (DcMotor motor: motors) {
//                //Getting the index of the motor
//                int index = motors.indexOf(motor);
//
//                //Calculating the errors of the motor
//                double error = distance;
//                double delta = lastErrors[index] - error;
//
//                //Calculating the integral and derivative terms
//                integral += delta * PIDTimer.time();
//                double derivative = delta/PIDTimer.time();
//
//                //Multiplying the errors with the PID coefficients
//                double P = pidCoefficients.p * error;
//                double I = pidCoefficients.i * integral;
//                double D = pidCoefficients.d * derivative;
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
        dreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void odometryDrive(double forward, double rotate) {

        double frontLeftSpeed = (forward - rotate);
        double frontRightSpeed = (-forward - rotate);
        double backLeftSpeed = (forward - rotate);
        double backRightSpeed = (-forward - rotate);

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void odometryMovementTest(double power, double angle) {
        double angle_in = Math.toRadians(angle);  // convert to robot coordinates

        double delta = normalizeRadians(Math.toRadians(globalPositionUpdate.returnOrientation()) - angle_in);

//        double MAX_ROTATE = 0.3; //This is to shrink how fast we can rotate so we don't fly past the angle
//        if (y <= 0.05) {
//            delta = 0;
//        }
//        else if (y < 0.5) {
//            y /= 1.5;
//            delta *= 0.3;
//        }
//        else {
//            delta *= 0.3;
//        }
        odometryDrive(power, delta);
    }



    public void driveRelativeManualRotation(double y, double angle, double rotationSpeed) {
        double angle_in = angle + Math.PI/2;  // convert to robot coordinates
//        opMode.telemetry.addData("Unghi: ", angle);
//        opMode.telemetry.update();

        double delta = normalizeRadians(Math.toRadians(getAngle()) - angle_in);

        double MAX_ROTATE = 1; //This is to shrink how fast we can rotate so we don't fly past the angle

        if (delta <= Math.PI / 4) {
            teleOpDrive(y, rotationSpeed);
        }
    }

    void setSpeeds(double frontLeftSpeed, double frontRightSpeed, double backLeftSpeed, double backRightSpeed) {
        double largest = maxSpeed;
        largest = Math.max(largest, Math.abs(frontLeftSpeed));
        largest = Math.max(largest, Math.abs(frontRightSpeed));
        largest = Math.max(largest, Math.abs(backLeftSpeed));
        largest = Math.max(largest, Math.abs(backRightSpeed));

        stangaFata.setPower(frontLeftSpeed);
        stangaSpate.setPower(backLeftSpeed);
        dreaptaFata.setPower(frontRightSpeed);
        dreaptaSpate.setPower(backRightSpeed);
    }

    public void teleOpDrive(double forward, double rotate) {

        double frontLeftSpeed = SQRT(forward - rotate);
        double frontRightSpeed = SQRT(-forward - rotate);
        double backLeftSpeed = SQRT(forward - rotate);
        double backRightSpeed = SQRT(-forward - rotate);

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void teleOpDriveRotire(double forward,double rotate){

        double frontLeftSpeed = SQRT(0- rotate);
        double frontRightSpeed = SQRT(0- rotate);
        double backLeftSpeed = SQRT(0 - rotate);
        double backRightSpeed = SQRT( 0- rotate);

        setSpeeds(frontLeftSpeed, frontRightSpeed, backLeftSpeed, backRightSpeed);
    }

    public void runUsingEncoders (int distance, double power, double timeout) {
        ElapsedTime runtime = new ElapsedTime();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.sleep(50);

        stangaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(-distance);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;

        stangaFata.setVelocity(p * coefficient);
        stangaSpate.setVelocity(p * coefficient);
        dreaptaFata.setVelocity(p * coefficient);
        dreaptaSpate.setVelocity(p * coefficient);

        runtime.reset();

        while (opMode.opModeIsActive() && stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            if (p < power) p += 0.05;

            stangaFata.setPower(p);
            stangaSpate.setPower(p);
            dreaptaFata.setPower(p);
            dreaptaSpate.setPower(p);

        }

        stopDriving();

    }

    //Used for robot travels longer than 2500 ticks
    public void runUsingEncodersLongRun (int distance, double power, double timeout) {
        double coefficient = 2000;
        ElapsedTime runtime = new ElapsedTime();

        stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        opMode.sleep(50);

        stangaFata.setTargetPosition(distance);
        stangaSpate.setTargetPosition(distance);
        dreaptaFata.setTargetPosition(-distance);
        dreaptaSpate.setTargetPosition(-distance);

        stangaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stangaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double p = 0.05;
//        stangaFata.setPower(p);
//        stangaSpate.setPower(p);
//        dreaptaFata.setPower(p);
//        dreaptaSpate.setPower(p);

        runtime.reset();

        boolean accelerate = true;
        double minPower = 0.3;

        while (opMode.opModeIsActive() && stangaSpate.isBusy() && stangaFata.isBusy() && dreaptaSpate.isBusy() && dreaptaFata.isBusy() && runtime.seconds() < timeout) {
            if (p < power && accelerate) p += 0.03;

            if ((Math.abs(distance) - Math.abs(stangaSpate.getCurrentPosition()) < cmToTicks(45) ||
                    Math.abs(distance) - Math.abs(stangaFata.getCurrentPosition()) < cmToTicks(45) ||
                    Math.abs(distance) - Math.abs(dreaptaFata.getCurrentPosition()) < cmToTicks(45) ||
                    Math.abs(distance) - Math.abs(dreaptaSpate.getCurrentPosition()) < cmToTicks(45)) && accelerate)
            {
                if (p > minPower && accelerate) {
                    p -= 0.065;
                }
                else {
                    accelerate = false;
                }
            }
            else if (accelerate == false) {
                p = minPower;
            }

            stangaFata.setVelocity(p * coefficient);
            stangaSpate.setVelocity(p * coefficient);
            dreaptaFata.setVelocity(p * coefficient);
            dreaptaSpate.setVelocity(p * coefficient);


        }

        stopDriving();

    }

    public int cmToTicks(double cm) {
        return (int)(cm / (Math.PI * 9) * 560);
    }

    public void shoot3Rings() {
        lansareRing.setPosition(1);
        opMode.sleep(350);
        lansareRing.setPosition(0.5);
        opMode.sleep(350);
        lansareRing.setPosition(1);
        opMode.sleep(350);
        lansareRing.setPosition(0.5);
        opMode.sleep(350);
        lansareRing.setPosition(1);
        opMode.sleep(350);
        lansareRing.setPosition(0.5);
        opMode.sleep(150);
    }


    public void zeroPowerBeh() {
        stangaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaFata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaSpate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double SQRT(double x) {
        if (x < 0) return -Math.sqrt(Math.abs(x));
        return Math.sqrt(x);
    }

    public void rotateConstantSpeed(int degrees, double power, int timeout)
    {

        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        ElapsedTime runtime = new ElapsedTime();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
        }
        else if (degrees > 0)
        {   // turn left.
            power = -power;
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
        }
        else return;


        // set power to rotate.
        stangaFata.setPower(power);
        dreaptaFata.setPower(power);
        stangaSpate.setPower(power);
        dreaptaSpate.setPower(power);

        // rotate until turn is completed.
        if (degrees < 0)
        {

            while (opMode.opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        stopDriving();
        //        rightMotor.setPower(0);
        //        leftMotor.setPower(0);

        // wait for rotation to stop.
        opMode.sleep(200);
        // reset angle tracking on new heading.
        resetAngle();
    }

    public void rotate(int degrees, double power, int timeout)
    {

        stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stangaFata.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        dreaptaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ElapsedTime runtime = new ElapsedTime();

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
        }
        else if (degrees > 0)
        {   // turn left.
//            stangaFata.setPower(power);
//            dreaptaSpate.setPower(-power);
//            stangaSpate.setPower(power);
//            dreaptaFata.setPower(-power);
            power = -power;
        }
        else return;


        // set power to rotate.
//        stangaFata.setPower(power);
//        dreaptaFata.setPower(power);
//        stangaSpate.setPower(power);
//        dreaptaSpate.setPower(power);

        double coefficient = 1500;

        dreaptaFata.setVelocity(power * coefficient);
        stangaSpate.setVelocity(power * coefficient);
        dreaptaSpate.setVelocity(power * coefficient);
        stangaFata.setVelocity(power * coefficient);


        //        leftMotor.setPower(leftPower);
        //        rightMotor.setPower(rightPower);

        // rotate until turn is completed.
        boolean powerModified = false;
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opMode.opModeIsActive() && getAngle() == 0 && runtime.seconds() < timeout) { }

            while (opMode.opModeIsActive() && getAngle() > degrees && runtime.seconds() < timeout) {
//                if (getAngle() - degrees < 30 && !powerModified) {
//                    //lower the power in order to rotate with more precision
//                    power /= 2;
//                    stangaFata.setPower(power);
//                    dreaptaFata.setPower(power);
//                    stangaSpate.setPower(power);
//                    dreaptaSpate.setPower(power);
//                    powerModified = true;
//                }
            }
        }
        else    // left turn.
            while (opMode.opModeIsActive() && getAngle() < degrees && runtime.seconds() < timeout) {
//                if (degrees - getAngle() < 30 && !powerModified) {
//                    //lower the power in order to rotate with more precision
//                    power /= 2;
//                    stangaFata.setPower(power);
//                    dreaptaFata.setPower(power);
//                    stangaSpate.setPower(power);
//                    dreaptaSpate.setPower(power);
//                    powerModified = true;
//                }
            }

        // turn the motors off.
        stopDriving();
        //        rightMotor.setPower(0);
        //        leftMotor.setPower(0);

        // wait for rotation to stop.

        // reset angle tracking on new heading.
//        resetAngle();
    }

    public double getHeading(AngleUnit angleUnit) {
        Orientation angles;

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        return angleUnit.fromRadians(angles.firstAngle + imuOffset);
    }

    public double getCurrentAngle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angle.firstAngle;
        if (currentAngle < -180) {
            while (currentAngle < -180) {
                currentAngle += 360;
            }
        }
        else {
            while (currentAngle > 180) {
                currentAngle -= 360;
            }
        }
        return currentAngle;
    }

    public double get360Angle() {
        Orientation angle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angle.firstAngle;
        if (currentAngle <= -360) {
            while (currentAngle < -360) {
                currentAngle += 360;
            }
        }
        else if (currentAngle >= 360) {
            while (currentAngle > 360) {
                currentAngle -= 360;
            }
        }
        return currentAngle;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void stopDriving() {
        stangaSpate.setPower(0);
        dreaptaSpate.setPower(0);
        stangaFata.setPower(0);
        dreaptaFata.setPower(0);
    }


}