package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class PIDMovement {

    DcMotor motors[];
    ElapsedTime PIDTimer = new ElapsedTime();
    private PIDCoefficients pidCoefficients;


    double integral = 0;
    double repetitions = 0;

    public PIDMovement(DcMotor motors[], PIDCoefficients coefficients) {
        this.motors = motors;

        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        this.pidCoefficients = coefficients;
    }

    private double getError(double[] errorsArray) {
        double totalError = 0;
        for (double error : errorsArray) {
            totalError += error;
        }
        return totalError;
    }

    public void moveMotor(double targetPosition) {
        double[] errorsArray = {0, 0, 0, 0};
        double[] lastErrors = {0, 0, 0, 0};

        for (int i = 0; i < 4; i++) {
            errorsArray[i] = motors[i].getCurrentPosition();
        }


        double globalError = getError(errorsArray);
        while (Math.abs(globalError) <= 9) {
            for (int i = 0; i < 4; i ++) {
                double error = motors[i].getCurrentPosition() - targetPosition;
                errorsArray[i] = error;
                double changInError = lastErrors[i] - errorsArray[i];
                integral += changInError * PIDTimer.time();
                double derivative = changInError / PIDTimer.time();
                double P = pidCoefficients.p * error;
                double I = pidCoefficients.i * integral;
                double D = pidCoefficients.d * derivative;
                motors[i].setPower(P + I + D);
                lastErrors[i] = error;
//                repetitions++;
                PIDTimer.reset();
            }
        }

        for (int i = 0; i < 4; i ++) {
            motors[i].setPower(0);
        }
    }

}
