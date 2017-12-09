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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller Autonomous Mode
 * <p>
 * What we control:
 * <p>
 * Motors
 * left_motor
 * right_motor
 * lifter_motor
 * gem_motor
 * <p>
 * Servos
 * glyph_servo
 * gem_servo
 * <p>
 * Sensors
 * gem_sensor
 * camera
 * <p>
 * How we control:
 * <p>
 * Gamepad1 will control the left_motor and right_motor.
 * Gamepad2 will control everything else.
 * <p>
 * Gamepad1 definitions:
 * left_stick_y (up and down) controls forward or backward
 * right_stick_x (side to side) controls turning
 * <p>
 * Gamepad2 definitions:
 * left_stick_y (up and down) controls lifter up and down
 * X button (press) opens glyph_servo
 * B button (press) closes glyph_servo
 * <p>
 * Tracking inputs:
 * IMU heading (from the REV Expansion Hub)
 * left_motor, right_motor position (from encoders)
 **/
@Autonomous(name = "SteamMachines Autonomous Mode", group = "Linear Opmode")
//@Disabled
public class SteamMachines_Autonomous extends LinearOpMode {

    // Constructor: instantiate objects used in this class.
    //  motors
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_motor = null;
    private DcMotor gem_motor = null;
    //  servos
    private Servo glyph_servo = null;
    private Servo gem_servo = null;
    //  timer
    private ElapsedTime runtime = new ElapsedTime();
    //  constants
    static double GLYPH_SERVO_OPEN = 0.4;    // 0.4 * 180 =  72 degrees
    static double GLYPH_SERVO_CLOSED = 0.9;  // 0.9 * 180 = 162 degrees
    static int LIFTER_MIN_POS = 100;
    static int LIFTER_MAX_POS = 6000;
    static double LIFTER_IDLE = 0.01;
    static double WHEEL_CIRC = 3.5 * 3.1415;
    static int GEAR_RATIO = 40 / 56;
    static int TICKS_PER_REV = 757;
    static double COUNTS_PER_INCH = (TICKS_PER_REV * GEAR_RATIO) / WHEEL_CIRC;
    static double CRYPT_WIDTH = 7.63; //inches
    static double D_CENTER = 36;
    static double D_RIGHT = D_CENTER - CRYPT_WIDTH;
    static double D_LEFT = D_CENTER + CRYPT_WIDTH;

    //757 ticks per revolution on matrix 12v motor

    // the starting position will be one of these
    // R1 and B1 closest to Relic Recovery Zone
    enum StartingPosition {
        R1, R2, B1, B2
    }

    ;

    @Override
    public void runOpMode() {

        StartingPosition startPos = StartingPosition.B1;

        // let drivers know that initialization has begun
        telemetry.addData("Status", "Initializing");

        // Initialize hardware variables
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
        lifter_motor.setTargetPosition(LIFTER_MIN_POS);

        // servos
        glyph_servo = hardwareMap.get(Servo.class, "glyph_servo");
        glyph_servo.setDirection(Servo.Direction.FORWARD);
        glyph_servo.setPosition(GLYPH_SERVO_OPEN);

        // let drivers know that initialization has finished
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        telemetry.addData("Status", "Started!");

        // timer may be used to limit amount of time for each task
        // double timer = runtime.time();

        // task 1: decode crypto-key
        telemetry.addData("Status", "Task 1 started");
        RelicRecoveryVuMark cryptoKey = FindCryptoKey();
        telemetry.addData("Status", "Task 1 finished");
        // need if statement here!


        // task 2: knock off other-colored gem
//        GemBump(startPos);
        double d_vertical;
//            // task 3: move robot to cryptobox
        if (cryptoKey == RelicRecoveryVuMark.CENTER)
            d_vertical = D_CENTER;

        else if (cryptoKey == RelicRecoveryVuMark.RIGHT)
            d_vertical = D_RIGHT;

        else if (cryptoKey == RelicRecoveryVuMark.LEFT)
            d_vertical = D_LEFT;

        else
            d_vertical = D_CENTER;


//        MoveToCrypt(d_vertical, d_vertical);
//
//
//            // task 4: place glyph
//            TurnToFaceCryptobox();
//            PlaceBlock(cryptokey);
//
//
//            // task 5: park
//            Park();
//
//        // Telemetry: one time snap shots, not like TeloOp mode!
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);

//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive() || timer is over 30 seconds ) {
//            telemetry.addData("Status", "Waiting for driver to press STOP");
//        }
    }

    // task 1: decode cryptoKey
    public RelicRecoveryVuMark FindCryptoKey() {
        RelicRecoveryVuMark cryptoKey = RelicRecoveryVuMark.UNKNOWN; // default value

        // use vuforia code
//        VuforiaCamera camera = new VuforiaCamera();
//        camera.runOpMode();
//        camera.init();
//        camera.start(); // need to wait???
//
//        camera.
//        cryptoKey = camera.vuMark;

        VuforiaCamera camera = new VuforiaCamera();
        cryptoKey = camera.run(hardwareMap, telemetry, 2);


        return cryptoKey;
    }

    // task 2: knock off gem (that's not our color)
//    public void GemBump(StartingPosition startPos  ??? what else here Emma ???) {
//
//
//    }

    // task 3: move to crypt
    public void MoveToCrypt(double timeoutS,
                            double leftInches, double rightInches,
                            double speed) {
        int leftTarget;
        int rightTarget;
        if (opModeIsActive()) {
//            left = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
        }

    }
} // end of class (no code beyond this point)
