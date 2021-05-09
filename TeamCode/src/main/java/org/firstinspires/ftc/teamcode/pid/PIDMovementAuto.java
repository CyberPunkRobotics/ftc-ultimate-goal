package org.firstinspires.ftc.teamcode.pid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotMap;

@Config
@Autonomous(name="PID Calibration")
public class PIDMovementAuto extends LinearOpMode {

    RobotMap robot = null;

    FtcDashboard dashboard;

    public static int TARGET_POS = 1000;
    public static PIDCoefficients pidCoefficients = new PIDCoefficients(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotMap(hardwareMap, this);

//        robot.stangaSpate.setDirection(DcMotorSimple.Direction.REVERSE);
//        robot.stangaFata.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = {robot.stangaFata, robot.stangaSpate, robot.dreaptaSpate, robot.dreaptaFata};
        PIDMovement movement = new PIDMovement(motors, pidCoefficients);
        dashboard = FtcDashboard.getInstance();

        waitForStart();

        robot.runUsingPID(TARGET_POS, 0.3, 10, pidCoefficients);

    }
}
