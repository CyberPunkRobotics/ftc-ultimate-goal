package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Test senzori de culoare")
public class TestSenzoriCuloare extends LinearOpMode {
    private RobotMap robot;

    private int frontCounts = 0;
    private int backCounts = 0;

    private boolean detected = false;
    private int counts = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotMap(hardwareMap, this);
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.x) {
                robot.rotite.setPower(1);
            }
            if (gamepad1.b) {
                robot.rotite.setPower(0);
            }
//            telemetry.addData("Fata Albastru: ", robot.senzorStanga.blue());
//            telemetry.addData("Fata Rosu", robot.senzorStanga.red());
//            telemetry.addData("Fata Verde", robot.senzorStanga.green());
//            telemetry.addData("Fata Culoare overall: ", robot.senzorStanga.getNormalizedColors().toColor());
//            telemetry.addData("Front counts: ", frontCounts);
//
//            telemetry.addData("Spate Albastru: ", robot.senzorDreapta.blue());
//            telemetry.addData("Spate Rosu", robot.senzorSpate.red());
//            telemetry.addData("Spate Verde", robot.senzorSpate.green());
//            telemetry.addData("Spate Culoare overall: ", robot.senzorSpate.getNormalizedColors().toColor());
//            telemetry.addData("Back counts: ", backCounts);

            telemetry.addData("Counts: ", counts);
            telemetry.addData("Culoare Stanga: ", robot.senzorStanga.getNormalizedColors().toColor());
            telemetry.addData("Culoare Dreapta: ", robot.senzorDreapta.getNormalizedColors().toColor());
            telemetry.update();
        }
    }
}
