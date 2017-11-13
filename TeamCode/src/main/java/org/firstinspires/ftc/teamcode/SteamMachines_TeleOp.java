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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller TeleOpMode
 *
 * What we control:
 *
 *   Motors
 *    left_motor
 *    right_motor
 *    lifter_motor
 *
 *   Servos
 *    glyph_servo
 *    gem_servoA (autonomous mode only)
 *    gem_servoB (autonomous mode only)
 *
 *   Sensors
 *    gem_sensor (autonomous mode only)
 *    camera (autonomous mode only)
 *
 *
 * How we control:
 *
 *   Gamepad1 will control the left_motor and right_motor.
 *   Gamepad2 will control everything else.
 *
 *   Gamepad1 definitions:
 *    left_stick_y (up and down) controls forward or backward
 *    right_stick_x (side to side) controls turning
 *
 *   Gamepad2 definitions:
 *    left_stick_y (up and down) controls lifter up and down
 *    X button (press) opens glyph_servo
 *    B button (press) closes glyph_servo
 *
 **/

@TeleOp(name="SteamMachines TeleOp Mode", group="Iterative Opmode")
//@Disabled
public class SteamMachines_TeleOp extends OpMode
{

    // Constructor: instantiate objects used in this class.
    //  motors
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_motor = null;
    //  servos
    private Servo glyph_servo = null;
    //  timer
    private ElapsedTime runtime = new ElapsedTime();
    //  constants
    static double GLYPH_SERVO_OPEN = 0.40;    // 0 degrees
    static double GLYPH_SERVO_CLOSED = 0.9;  // 0.4 * 180 = 72 degrees
    static int LIFTER_MIN_POS = 100;
    static int LIFTER_MAX_POS = 6000;
    static double LIFTER_IDLE = 0.01;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // let drivers know that initialization has begun
        telemetry.addData("Status", "Initializing");

        // Initialize hardware variables
        // NOTE: deviceName must match config file on phone
        // motors
        left_motor = hardwareMap.get(DcMotor.class,"left_motor");
        left_motor.setDirection(DcMotor.Direction.FORWARD);

        right_motor = hardwareMap.get(DcMotor.class,"right_motor");
        right_motor.setDirection(DcMotor.Direction.REVERSE);

        lifter_motor = hardwareMap.get(DcMotor.class,"lifter_motor");
        lifter_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lifter_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lifter_motor.setDirection(DcMotor.Direction.REVERSE);
        lifter_motor.setTargetPosition(LIFTER_MIN_POS);

        // servos
        glyph_servo = hardwareMap.get(Servo.class,"glyph_servo");
        glyph_servo.setDirection(Servo.Direction.FORWARD);
        glyph_servo.setPosition(GLYPH_SERVO_OPEN);
        // how do we set limits on servo???

        // let drivers know that initialization has finished
        telemetry.addData("Status", "Initialized");

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
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

        //set paramaters for lifter_motor
        if(lifterPower > 0.1)//joysticks = positive is up
            if(lifter_pos < LIFTER_MAX_POS)
                lifter_motor.setPower(lifterPower);
            else
                lifter_motor.setPower(0);
        else if(lifterPower < -0.1)//joysticks = negative is down
            if(lifter_pos > LIFTER_MIN_POS)
                lifter_motor.setPower(lifterPower);
            else
                lifter_motor.setPower(0);
        else
            lifter_motor.setPower(0);

        // ** glyph servo **
        //    buttons X and B will open or close the grabber
        if (gamepad2.b)
            glyph_servo.setPosition(GLYPH_SERVO_CLOSED);
        else if (gamepad2.x)
            glyph_servo.setPosition(GLYPH_SERVO_OPEN);

        // Telemetry: show elapsed time, wheel power, lifter motor
        // This can be whatever we want it to be.  We want info that helps the operators.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("Lifter motor (%.2f)", lifterPower);
        telemetry.addData("lifter_motor.getCurrentPosition", lifter_motor.getCurrentPosition());
        telemetry.addData("Glyph Servo position (%.2f)", glyph_servo.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
