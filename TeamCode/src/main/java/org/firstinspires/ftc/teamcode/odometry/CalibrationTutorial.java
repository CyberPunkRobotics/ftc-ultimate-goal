package org.firstinspires.ftc.teamcode.odometry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.core.Mat;

import java.io.File;

/**
 * Essentially, we are saving in these files the ticks of the odometry encoders
 * that are needed for a rotation. Using the 2 values we will be able to rotate
 * the robot with a lot more accuracy, just by telling the robot how many
 * odometry ticks to rotate.
 *
 * */
@Autonomous
@Disabled
public class CalibrationTutorial extends LinearOpMode {

    DcMotor frontRight, frontLeft, backRight, backLeft;
    DcMotor leftEncoder, rightEncoder, middleEncoder;

    BNO055IMU imu;

    ElapsedTime timer = new ElapsedTime();

    static final double calibrationSpeed = .5;

    static final double TICK_PER_REVOLUTION = 1440;
    static final double WHEEL_DIAMETER = 100/25.4;
    static final double GEAR_RATIO = 1;

    static final double TICKS_PER_INCH = WHEEL_DIAMETER * Math.PI * GEAR_RATIO / TICK_PER_REVOLUTION;

    File sideWheelsSeparationFile = AppUtil.getInstance().getSettingsFile("sideWheelsSeparationFile");
    File middleTickOffsetFile = AppUtil.getInstance().getSettingsFile("middleTickOffsetFile");

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        rightEncoder = hardwareMap.dcMotor.get("rightEncoder");
        middleEncoder = hardwareMap.dcMotor.get("middleEncoder");

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        resetOdometryEncoders();

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Status: ", "Ready!");
        telemetry.update();

        waitForStart();

        while (imu.getAngularOrientation().firstAngle < 90 && opModeIsActive()) {
            frontLeft.setPower(calibrationSpeed);
            backLeft.setPower(calibrationSpeed);
            frontRight.setPower(-calibrationSpeed);
            backRight.setPower(-calibrationSpeed);

            if (imu.getAngularOrientation().firstAngle < 60) {
                frontLeft.setPower(calibrationSpeed);
                backLeft.setPower(calibrationSpeed);
                frontRight.setPower(-calibrationSpeed);
                backRight.setPower(-calibrationSpeed);
            }
            else {
                frontLeft.setPower(calibrationSpeed/2);
                backLeft.setPower(calibrationSpeed/2);
                frontRight.setPower(-calibrationSpeed/2);
                backRight.setPower(-calibrationSpeed/2);
            }
        }

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        timer.reset();
        while (timer.seconds() < 1 && opModeIsActive()) {
            //waiting for IMU to stabilize
        }

        double angle = imu.getAngularOrientation().firstAngle;
        double encoderDifference = Math.abs(Math.abs(leftEncoder.getCurrentPosition()) - Math.abs(rightEncoder.getCurrentPosition()));
        double sideEncoderTickOffset = encoderDifference / angle;
        double sideWheelSeparation =  (180 * sideEncoderTickOffset) / (TICKS_PER_INCH * Math.PI);
        double middleOffset = middleEncoder.getCurrentPosition() / Math.toRadians(imu.getAngularOrientation().firstAngle);


        ReadWriteFile.writeFile(sideWheelsSeparationFile, String.valueOf(sideWheelSeparation));
        ReadWriteFile.writeFile(middleTickOffsetFile, String.valueOf(middleOffset));

    }

    private void resetOdometryEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middleEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
