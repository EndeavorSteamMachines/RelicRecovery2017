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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import static android.os.SystemClock.sleep;


/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller TeleOpMode
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
 * <p>
 * Sensors
 * (none configured for TeleOp)
 * <p>
 * How we control:
 * <p>
 * Gamepad1 will control the left_motor and right_motor.
 * Gamepad2 will control everything else.
 * <p>
 * Gamepad1 definitions:
 * - left_stick_y (up and down) controls forward or backward
 * - right_stick_x (side to side) controls turning
 * <p>
 * Gamepad2 definitions:
 * - left_stick_y (up and down) controls lifter up and down
 * - X (press) opens  glyph servos
 * - B (press) closes glyph servos
 **/

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SteamMachines TeleOp Mode", group = "TeleOp")
//@Disabled
public class TeleOpmode extends OpMode {

    // Constructor: instantiate objects used in this class.
    //  motors
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_motor = null;
    //  servos
    private Servo left_glyph_servo = null;
    private Servo right_glyph_servo = null;
    //  timer
    private ElapsedTime runtime = new ElapsedTime();
    //  variables used by all methods in this class
//    static int LIFTER_MIN_POS = 100;
//    static int LIFTER_MAX_POS = 6000;
//    static double LIFTER_IDLE = 0.01;
    static float JOYSTICK_DEADZONE = 0.2f;
    static double LEFT_SERVO_OPEN = 0.70;
    static double RIGHT_SERVO_OPEN = 0.30;
    static double LEFT_SERVO_CLOSED = 0.95;
    static double RIGHT_SERVO_CLOSED = 0.05;
    double rightServoPos;
    double leftServoPos;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // let drivers know that initialization has begun
        telemetry.addData("Status", "Initializing");
        // Initialize hardware variables
        // NOTE: deviceName must match config file on phone
        //
        // motors
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        left_motor.setDirection(DcMotor.Direction.FORWARD);
        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        right_motor.setDirection(DcMotor.Direction.REVERSE);
        right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lifter_motor = hardwareMap.get(DcMotor.class, "lifter_motor");
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_motor.setDirection(DcMotor.Direction.REVERSE);
        lifter_motor.setPower(0);

        // initialize deadzone values for joysticks
//        gamepad1.setJoystickDeadzone(JOYSTICK_DEADZONE);
//        gamepad2.setJoystickDeadzone(JOYSTICK_DEADZONE);

        // move lifter to down position???
//        lifter_motor.setTargetPosition(LIFTER_MIN_POS); // or -6000?
//        runtime.reset();
//        lifter_motor.setPower(-0.5);
//        while (lifter_motor.isBusy() && runtime.seconds() < 3) {
//            telemetry.addData("Status", "resetting lifter position");
//        }
//        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lifter_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // servos
        left_glyph_servo = hardwareMap.get(Servo.class, "left_glyph_servo");
        left_glyph_servo.setDirection(Servo.Direction.FORWARD);
//        left_glyph_servo.setPosition(LEFT_SERVO_OPEN);

        right_glyph_servo = hardwareMap.get(Servo.class, "right_glyph_servo");
        right_glyph_servo.setDirection(Servo.Direction.FORWARD);
//        right_glyph_servo.setPosition(RIGHT_SERVO_OPEN);

        // let drivers know that initialization has finished
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Status", "INIT has been pressed");
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        telemetry.addData("Status", "PLAY has been pressed");
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        // ** drive motors **
        //    left_stick_y (up and down) controls forward or backward
        //    right_stick_x (side to side) controls turning

        // turn and drive depend on the current position of the joysticks
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // use turn and drive to determine amount of power to apply to motors
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        // set motor power
        left_motor.setPower(leftPower);
        right_motor.setPower(rightPower);

        // ** lifter motor **
        //    left_stick_y (up and down) controls lifter up and down
        double lift = -gamepad2.left_stick_y; // make positive up
        double lifterPower = Range.clip(lift, -1.0, 1.0);
        int lifter_pos = lifter_motor.getCurrentPosition();

        if (lifterPower < -JOYSTICK_DEADZONE)
            lifter_motor.setPower(lifterPower);
        else if (lifterPower > JOYSTICK_DEADZONE)
            lifter_motor.setPower(lifterPower);
        else
            lifter_motor.setPower(0);

//        //set parameters for lifter_motor
//        if (lifterPower > JOYSTICK_DEADZONE) // joystick positive => up
//            if (lifter_pos < LIFTER_MAX_POS)
//                lifter_motor.setPower(lifterPower);
//            else
//                lifter_motor.setPower(LIFTER_IDLE);
//        else if (lifterPower < -JOYSTICK_DEADZONE) //joystick negative => down
//            if (lifter_pos > LIFTER_MIN_POS)
//                lifter_motor.setPower(lifterPower);
//            else
//                lifter_motor.setPower(0);
//        else
//            lifter_motor.setPower(LIFTER_IDLE);

        // ** glyph servo **
        //    buttons X and B will open or close the grabber servos
        rightServoPos = right_glyph_servo.getPosition();
        leftServoPos = left_glyph_servo.getPosition();

        if (gamepad2.x) {
            right_glyph_servo.setPosition(RIGHT_SERVO_OPEN);
            left_glyph_servo.setPosition(LEFT_SERVO_OPEN);

//            rightServoPos = rightServoPos - 0.1;
//            rightServoPos = Range.clip(rightServoPos, 0, 1);
//            right_glyph_servo.setPosition(rightServoPos);
//
//            leftServoPos = leftServoPos + 0.1;
//            leftServoPos = Range.clip(leftServoPos, 0, 1);
//            left_glyph_servo.setPosition(leftServoPos);
//            sleep(500);//gives driver 1 sec to let go of button
//  //          1000 ms = 1 sec : 500 ms = 0.5 (1/2) sec

        } else if (gamepad2.b) {
            right_glyph_servo.setPosition(RIGHT_SERVO_CLOSED);
            left_glyph_servo.setPosition(LEFT_SERVO_CLOSED);

//            rightServoPos = rightServoPos + 0.1;
//            rightServoPos = Range.clip(rightServoPos, 0, 1);
//            right_glyph_servo.setPosition(rightServoPos);
//
//            leftServoPos = leftServoPos - 0.1;
//            leftServoPos = Range.clip(leftServoPos, 0, 1);
//            left_glyph_servo.setPosition(leftServoPos);
//            sleep(500);//gives driver 1 sec to let go of button

        }

        // Telemetry: show elapsed time, wheel power, lifter motor, and servo status
        // This can be whatever we want it to be.  We want info that helps the operators.
        //telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Runtime", "(%.2f)", runtime.milliseconds() / 1000); // ?? will this work ??
        telemetry.addData("Left Drive Motor", "(%.2f)", leftPower);
        telemetry.addData("Right Drive Motor", "(%.2f)", rightPower);
        telemetry.addData("Lifter Motor Power", "(%.2f)", lifterPower);
        telemetry.addData("Lifter Motor Position", lifter_pos);
        telemetry.addData("Left Servo Position", "(%.2f)", leftServoPos);
        telemetry.addData("Right Servo Position", "(%.2f)", rightServoPos);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "STOP has been pressed");
    }

}
