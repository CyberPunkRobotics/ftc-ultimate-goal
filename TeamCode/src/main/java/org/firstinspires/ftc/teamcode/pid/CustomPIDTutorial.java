package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous
public class CustomPIDTutorial extends LinearOpMode {

    DcMotor testMotor;

    ElapsedTime PIDTimer = new ElapsedTime();
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);

    FtcDashboard dashboard;

    public static double TARGET_POS = 100;

    double integral = 0;
    double repetitions = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        testMotor = hardwareMap.dcMotor.get("testMotor");

        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        moveTestMotor(TARGET_POS);
    }

    void moveTestMotor (double targetPosition) {
        double error = testMotor.getCurrentPosition();
        double lastError = 0;

        while (Math.abs(error) <= 9 && repetitions < 40) {
            error = testMotor.getCurrentPosition() - targetPosition;
            double changInError = lastError - error;
            integral += changInError * PIDTimer.time();
            double derivative = changInError / PIDTimer.time();
            double P = pidCoefficients.p * error;
            double I = pidCoefficients.i * integral;
            double D = pidCoefficients.d * derivative;
            testMotor.setPower(P + I + D);
            lastError = error;
            repetitions ++;
            PIDTimer.reset();
        }
    }
}
