package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.ArrayList;

@Config
@Autonomous(name="test pid")
public class HectorPID extends LinearOpMode {

    RobotMap robot;
    ElapsedTime PIDTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(1, 0, 1);

    FtcDashboard dashboard;
    Telemetry dashboardTelemetry;

    private PIDFController controller;

    public static int TARGET_POS = 3000;

    void moveTestMotor (double targetPosition) {
        robot.stangaSpate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int repetitions = 0;
        double integral = 0;
        double error = targetPosition;
        double lastError = 0;
        telemetry.addData("Error: ", error);
        telemetry.update();

        while (Math.abs(error) >= 9) {
            error = -robot.stangaSpate.getCurrentPosition() + targetPosition;
            double changInError = lastError - error;
            integral += changInError * PIDTimer.time();
            double derivative = changInError / PIDTimer.time();
            double P = pidCoefficients.p * error;
            double I = pidCoefficients.i * integral;
            double D = pidCoefficients.d * derivative;
            robot.stangaSpate.setPower(Range.clip(P + I + D, -1, 1));
            lastError = error;
//            repetitions++;
            PIDTimer.reset();
        }
        sleep(2000);
    }

//    void moveTestMotor (double targetPosition) {
//        double error = testMotor.getCurrentPosition();
//        double lastError = 0;
//
//        while (Math.abs(error) <= 9 && repetitions < 40) {
//            error = testMotor.getCurrentPosition() - targetPosition;
//            double changInError = lastError - error;
//            integral += changInError * PIDTimer.time();
//            double derivative = changInError / PIDTimer.time();
//            double P = pidCoefficients.p * error;
//            double I = pidCoefficients.i * integral;
//            double D = pidCoefficients.d * derivative;
//            testMotor.setPower(P + I + D);
//            lastError = error;
//            repetitions ++;
//            PIDTimer.reset();
//        }
//    }

    public void moveRobotUsingPID(int distance, double timeout) {
        robot.dreaptaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.dreaptaFata.setDirection(DcMotorSimple.Direction.REVERSE);
        double integral = 0;
        ArrayList<DcMotor> motors = new ArrayList<>();
        motors.add(robot.stangaFata);
        motors.add(robot.stangaSpate);
        motors.add(robot.dreaptaSpate);
        motors.add(robot.dreaptaFata);
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
        while (Math.abs(globalError) > 50 && runtime.seconds() < timeout) {
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
        robot.dreaptaSpate.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.dreaptaFata.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();

        robot.stangaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaSpate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.stangaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.dreaptaFata.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Encoder position: ", robot.stangaSpate.getCurrentPosition());
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            moveTestMotor(TARGET_POS);
        }
    }
}
