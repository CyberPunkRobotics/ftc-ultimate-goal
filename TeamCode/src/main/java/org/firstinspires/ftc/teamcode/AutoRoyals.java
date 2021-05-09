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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Autonomie test PID")
public class AutoRoyals extends LinearOpMode
{
//    OpenCvCamera webcam;
//    SkystoneDeterminationPipeline pipeline;
    private RobotMap robot;

    /*
     * The core values which define the location and size of the sample regions
     */
//    public static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(180,150);

    public static int REGION_WIDTH = 45;
    public static int REGION_HEIGHT = 35;

    public static int FOUR_RING_THRESHOLD = 143;
    public static int ONE_RING_THRESHOLD = 135;

    private final double TOWER_GOAL_HEIGHT = 0.61;
    private final double POWER_SHOT_HEIGHT = 0.7;

    @Override
    public void runOpMode()
    {
        robot = new RobotMap(hardwareMap, this);
        robot.servoWobble.setPosition(0);
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.lansareRing.setPosition(0.5);
        robot.zeroPowerBeh();
//        robot.servoRidicare.setPosition(1);

        waitForStart();

        if (opModeIsActive())
        {
            FourRings();
        }
    }

    private void startAndShoot() {
        //Porneste motoarele
//        robot.servoRidicare.setPosition(0.8);
        robot.motorShooter.setPower(1);

        //Se pozitioneaza la tragere
        robot.runUsingEncoders(robot.cmToTicks(160), 1, 3);


        //Se pozitioneaza la tower goal
        robot.ridicareShooter.setPosition(TOWER_GOAL_HEIGHT);
        robot.runUsingEncoders(-9, 0.3, 3);

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
        sleep(150);
        //Opreste shooterul
        robot.motorShooter.setPower(0);
    }

    private void FourRings() {


        startAndShoot();
        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-12, 0.4, 2);
        robot.runUsingEncoders(robot.cmToTicks(160), 1, 5);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(-50), 1, 5);
        robot.rotateConstantSpeed(-139, 0.4, 3);
        //Se duce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(230),1,7);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
//        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(168, 0.4, 5);

        //Duce al doilea wobble
        robot.runUsingEncoders(robot.cmToTicks(262),1,7);


        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Parcheaza
        robot.runUsingEncoders(robot.cmToTicks(-85),1,5);
    }

    private void NoRing(){

        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        //robot.runUsingEncoders(robot.cmToTicks(20),1,3);
        robot.rotateConstantSpeed(-55,0.4,3);
        robot.runUsingEncoders(robot.cmToTicks(85),1,2);

        //lasa primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Se intoarce dupa al doilea
        robot.runUsingEncoders(robot.cmToTicks(-25), 1, 2);
        robot.rotateConstantSpeed(-95, 0.4, 3);

        //Se duce dupa al doilea
        robot.runUsingEncodersLongRun(robot.cmToTicks(140),1,7);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
//        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(160, 0.65, 5);

        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(147),1,7);


        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        robot.runUsingEncoders(-robot.cmToTicks(30), 1, 3);
        robot.rotateConstantSpeed(60, 0.4, 3);
        robot.runUsingEncoders(robot.cmToTicks(45), 1, 3);

    }

    private void OneRing(){
        startAndShoot();

        //Se pozitioneaza pentru wobble si il duce
        robot.rotateConstantSpeed(-5,0.4,2);
        robot.runUsingEncoders(robot.cmToTicks(72),1,2);

        //Lasat primul wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Mers dupa al doilea wobble
        robot.runUsingEncoders(robot.cmToTicks(-32),1,2);
        robot.rotateConstantSpeed(-133,0.4,3);
        robot.runUsingEncodersLongRun(robot.cmToTicks(180),1,6);

        //Prinde al doilea wobble
        robot.servoWobble.setPosition(0);
        sleep(500);
//        robot.servoRidicare.setPosition(0.8);

        //Se intoarce catre patrat
        robot.rotateConstantSpeed(160, 0.65, 5);

        //Duce al doilea wobble
        robot.runUsingEncodersLongRun(robot.cmToTicks(205),1,7);

        //Lasa al doilea wobble
        robot.servoWobble.setPosition(0.8);
//        robot.servoRidicare.setPosition(1);
        sleep(500);

        //Parcare
        robot.runUsingEncoders(robot.cmToTicks(-35),1,3);


    }
}