/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static java.lang.Thread.sleep;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller Autonomous Mode
 * <p>
 * What we control:
 * <p>
 * Motors
 * - left_motor
 * - right_motor
 * - lifter_motor
 * <p>
 * Servos
 * - left_glyph_servo
 * - right_glyph_servo
 * - gem_servoA (autonomous mode only)
 * - gem_servoB (autonomous mode only)
 * <p>
 * Sensors
 * - gem_sensor (autonomous mode only)
 * - camera (autonomous mode only)
 * - imu (autonomous mode only)
 * <p>
 * How we control:
 * <p>
 * - left_motor and right_motor: use RUN_TO_POSITION
 * - all other motors and servos: use constants
 * - color sensor: compare red to blue, return TRUE if red, else FALSE
 * - IMU heading (from the REV Expansion Hub)
 **/
public class Robot {

    HardwareMap hardwareMap;
    Telemetry telemetry;
    //  motors
    DcMotor left_motor = null;
    DcMotor right_motor = null;
    DcMotor lifter_motor = null;
    //  servos
    Servo left_glyph_servo = null;
    Servo right_glyph_servo = null;
    Servo gem_servoA = null;
    Servo gem_servoB = null;
    RelicRecoveryVuMark cryptoKey;

    //  timer
    ElapsedTime runtime = new ElapsedTime();
    //  constants
    double GLYPH_SERVO_OPEN = 0.4;    // 0.4 * 180 =  72 degrees
    double GLYPH_SERVO_CLOSED = 0.9;  // 0.9 * 180 = 162 degrees
    int LIFTER_MIN_POS = 100;
    int LIFTER_MAX_POS = 6000;
    double LIFTER_IDLE = 0.01;
    double GEM_SERVO_A_DOWN = 0.8;
    double GEM_SERVO_A_UP = 0.1;       // starts in up position
    double GEM_SERVO_B_RETRACTED = 0.0; // starts in stowed position
    double GEM_SERVO_B_DEPLOYED = 1.0;
    int GEM_SERVO_DIRECTION = 1;
    double WHEEL_DIAMETER = 3.5; // inches
    double WHEEL_CIRC = WHEEL_DIAMETER * 3.1415;
    double GEAR_RATIO = 56 / 40;
    double TICKS_PER_REV = 757; // matrix 12v motor (value found online)
    double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_CIRC * GEAR_RATIO);
    double CRYPT_WIDTH = 7.63; //inches, width of crypt
    double D_CENTER = 36; // distance from center of pad to center of CENTER crypt
    double D_RIGHT = D_CENTER - CRYPT_WIDTH;
    double D_LEFT = D_CENTER + CRYPT_WIDTH;
    double D_HORIZONTAL = 24 - (9 + 4); //distance to wall - (half of robot + 2/3 of glyph)
    double WHEELBASE = 16; //inches
    double TURN_CIRC = 3.1415 * (WHEELBASE * 2);
    double QUARTER_TURN = (TURN_CIRC - 4) / 4; //inches
    double LEFT_SERVO_OPEN = 0.70;
    double RIGHT_SERVO_OPEN = 0.30;
    double LEFT_SERVO_CLOSED = 0.95;
    double RIGHT_SERVO_CLOSED = 0.05;

    public Robot(HardwareMap hwMap, Telemetry tm) throws InterruptedException {
        // Initialize hardware variables
        hardwareMap = hwMap;
        telemetry = tm;
        // NOTE: deviceName must match config file on phone!
        // motors
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        left_motor.setDirection(DcMotor.Direction.FORWARD);

        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        right_motor.setDirection(DcMotor.Direction.REVERSE);

        lifter_motor = hardwareMap.get(DcMotor.class, "lifter_motor");
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_motor.setDirection(DcMotor.Direction.REVERSE);
        //lifter_motor.setTargetPosition(LIFTER_MIN_POS);
        //lifter_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // start with lifter down?
        // sleep(1000); // wait 1000ms

        // servos
        left_glyph_servo = hardwareMap.get(Servo.class, "left_glyph_servo");
        left_glyph_servo.setDirection(Servo.Direction.FORWARD);
        left_glyph_servo.setPosition(LEFT_SERVO_CLOSED);

        right_glyph_servo = hardwareMap.get(Servo.class, "right_glyph_servo");
        right_glyph_servo.setDirection(Servo.Direction.FORWARD);
        right_glyph_servo.setPosition(RIGHT_SERVO_CLOSED);

        gem_servoA = hardwareMap.get(Servo.class, "gem_servoA");
        gem_servoA.setDirection(Servo.Direction.FORWARD);
        //gem_servoA.setPosition(GEM_SERVO_A_UP);

        gem_servoB = hardwareMap.get(Servo.class, "gem_servoB");
        gem_servoB.setDirection(Servo.Direction.REVERSE);
        // gem_servoB.setPosition(GEM_SERVO_B_FOLDED);

    }

//
//    public void init(Telemetry telemetry, HardwareMap hardwareMap) {
//
//        // let drivers know that initialization has begun
//        telemetry.addData("Status", "Initializing");
//
//        // Initialize hardware variables
//        // NOTE: deviceName must match config file on phone!
//        // motors
//        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
//        left_motor.setDirection(DcMotor.Direction.FORWARD);
//
//        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
//        right_motor.setDirection(DcMotor.Direction.REVERSE);
//
//        lifter_motor = hardwareMap.get(DcMotor.class, "lifter_motor");
//        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lifter_motor.setDirection(DcMotor.Direction.REVERSE);
//        //lifter_motor.setTargetPosition(LIFTER_MIN_POS);
//        //lifter_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // start with lifter down?
//        // sleep(1000); // wait 1000ms
//
//        // servos
//        left_glyph_servo = hardwareMap.get(Servo.class, "left_glyph_servo");
//        left_glyph_servo.setDirection(Servo.Direction.FORWARD);
//        left_glyph_servo.setPosition(LEFT_SERVO_CLOSED);
//
//        right_glyph_servo = hardwareMap.get(Servo.class, "right_glyph_servo");
//        right_glyph_servo.setDirection(Servo.Direction.FORWARD);
//        right_glyph_servo.setPosition(RIGHT_SERVO_CLOSED);
//
//        gem_servoA = hardwareMap.get(Servo.class, "gem_servoA");
//        gem_servoA.setDirection(Servo.Direction.FORWARD);
//        //gem_servoA.setPosition(GEM_SERVO_A_UP);
//
//        gem_servoB = hardwareMap.get(Servo.class, "gem_servoB");
//        gem_servoB.setDirection(Servo.Direction.REVERSE);
//        // gem_servoB.setPosition(GEM_SERVO_B_FOLDED);
//
//
//
//
//        // let drivers know that initialization has finished
//        telemetry.addData("Status", "Initialized");
//
//
//        // timer may be used to limit amount of time for each task
//        // double timer = runtime.time();
//        RelicRecoveryVuMark cryptoKey = RelicRecoveryVuMark.CENTER;
//
//    }

    //////////////////////////////////////////////////
    public void Task1() {

        // decode crypto-key
        telemetry.addData("Status", "Task 1 started");
        telemetry.update();

        // use vuforia code
        VuforiaCamera camera = new VuforiaCamera();
        cryptoKey = camera.run(hardwareMap, telemetry, 5);


        // display result
        runtime.reset();
        while (runtime.seconds() < 5) {
            if (cryptoKey != RelicRecoveryVuMark.UNKNOWN) {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("VuMark", "%s visible", cryptoKey.toString());
                telemetry.update();
            } else {
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("VuMark", "not visible");
                telemetry.update();
            }
        }
    }

    public void Task2(StartCodes.Color startingColor) throws InterruptedException {

        // task 2: knock off other-colored gem

        telemetry.addData("Status", "Task 2 started");
        telemetry.update();

//            gem_servoB.setDirection(Servo.Direction.FORWARD);
//            gem_servoB.setPosition(GEM_SERVO_B_DEPLOYED);
//            sleep(1000);
//            gem_servoA.setDirection(Servo.Direction.FORWARD);
//            gem_servoA.setPosition(GEM_SERVO_A_DOWN);
        DetectRed gemRed = new DetectRed();

        boolean gemRedFlag = gemRed.run(hardwareMap, telemetry);
        if (gemRedFlag) {
            telemetry.addData("GemColor", "RED");
            telemetry.update();
        } else {
            telemetry.addData("GemColor", "BLUE");
            telemetry.update();
        }

//        if (gemColor == startColor) {
//            gem_servoB.setDirection(Servo.Direction.FORWARD);
//            gem_servoB.setPosition(GEM_SERVO_B_Bump);
//            sleep(500);
//            gem_servoB.setPosition(-GEM_SERVO_B_Bump);
//        }
//        else {
//            gem_servoB.setDirection(Servo.Direction.REVERSE);
//            gem_servoB.setPosition(GEM_SERVO_B_Bump);
//            sleep(500);
//            gem_servoB.setPosition(-GEM_SERVO_B_Bump);
//        }

//            gem_servoA.setDirection(Servo.Direction.REVERSE);
//            gem_servoA.setPosition(GEM_SERVO_A_UP);
//            gem_servoB.setDirection(Servo.Direction.REVERSE);
//            gem_servoB.setPosition(GEM_SERVO_B_RETRACTED);
    }

    public void Task3(StartCodes.Position startPos) {


        telemetry.addData("Status", "Task 3 started");
        telemetry.update();

        if (startPos == StartCodes.Position.B1) {
            double d_vertical;

            switch (cryptoKey) {
                case LEFT:
                    d_vertical = D_LEFT;
                case RIGHT:
                    d_vertical = D_RIGHT;
                default:
                    d_vertical = D_CENTER;

            }

            //movement from B1 to align in front of crypt
            DriveInches(0.9, d_vertical, d_vertical, 10);
            //turn to face crypt
            DriveInches(0.4, 0, QUARTER_TURN, 3);
            //drive to crypt
            DriveInches(0.6, D_HORIZONTAL, D_HORIZONTAL, 3);
            //drop glyph
//            glyph_servo.setPosition(GLYPH_SERVO_OPEN);
            //backup 1 inch
            DriveInches(-0.3, 1, 1, 2);

        } else if (startPos == StartCodes.Position.B2) {
            //@TODO drive B2

        } else if (startPos == StartCodes.Position.R1) {
            //@TODO drive R1


        } else if (startPos == StartCodes.Position.R2) {
            //@TODO drive R2

        }
    }


    // method DriveInches
    public void DriveInches(double speed,
                            double leftInches, double rightInches,
                            double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        // Determine new target position, and pass to motor controller
        newLeftTarget = left_motor.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        newRightTarget = right_motor.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

//        runtime.reset();
//        while (runtime.seconds() < 3) {
//            telemetry.addData("calculated newLeftTarget", newLeftTarget );
//            telemetry.addData("calculated newRightTarget", newRightTarget);
//            telemetry.update();
//        }

        // Turn On RUN_TO_POSITION
        left_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_motor.setTargetPosition(newLeftTarget);
        right_motor.setTargetPosition(newRightTarget);
        // start motion
        left_motor.setPower(speed);
        right_motor.setPower(speed);
        runtime.reset();
        while ((runtime.seconds() <= timeoutS) &&
                (left_motor.isBusy() && right_motor.isBusy())) {

            // Display it for the driver.
            telemetry.addData("Target", "Running to %5d , %5d", newLeftTarget, newRightTarget);
            telemetry.addData("Status", "Motors at %5d , %5d", left_motor.getCurrentPosition(), right_motor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion;
        left_motor.setPower(0);
        right_motor.setPower(0);

        // Turn off RUN_TO_POSITION
        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
} // end of class (no code beyond this point)

