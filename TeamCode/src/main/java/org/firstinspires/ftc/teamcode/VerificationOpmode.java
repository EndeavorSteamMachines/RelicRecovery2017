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

import android.hardware.Sensor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller TeleOpMode
 * <p>
 * What we control:
 * <p>
 * Motors
 * left_motor
 * right_motor
 * lifter_motor
 * <p>
 * Servos
 * glyph_servo
 * gem_servoA (autonomous mode only)
 * gem_servoB (autonomous mode only)
 * <p>
 * Sensors
 * gem_sensor (autonomous mode only)
 * camera (autonomous mode only)
 * imu (autonomous mode only)
 * <p>
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
 **/

@TeleOp(name = "VerificationOpmode Mode", group = "TeleOp")
//@Disabled
public class VerificationOpmode extends OpMode {

    // Constructor: instantiate objects used in this class
    //  motors
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    private DcMotor lifter_motor = null;
    //sensors
    private Sensor IMU = null;
    //  servos
    private Servo glyph_servo = null;
    private Servo gem_servoA = null;
    private Servo gem_servoB = null;
    //  timer
    private ElapsedTime runtime = new ElapsedTime();
    //  constants
    static double GLYPH_SERVO_OPEN = 0.45;    // 0 degrees
    static double GLYPH_SERVO_CLOSED = 0.9;// 0.4 * 180 = 72 degrees

    static double GEM_SERVO_A_DOWN = 0.8;
    static double GEM_SERVO_A_UP = 0.1;

    static double GEM_SERVO_B_FOLDED = 0.0;
    static double GEM_SERVO_B_OPEN = 1.0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Overrides
    public void init() {
        // let drivers know that initialization has begun
        telemetry.addData("Status", "Initializing");

        // Initialize hardware variables
        // NOTE: deviceName must match config file on phone
        // motors
        left_motor = hardwareMap.get(DcMotor.class, "left_motor");
        left_motor.setDirection(DcMotor.Direction.FORWARD);

        right_motor = hardwareMap.get(DcMotor.class, "right_motor");
        right_motor.setDirection(DcMotor.Direction.REVERSE);

        lifter_motor = hardwareMap.get(DcMotor.class, "lifter_motor");
        lifter_motor.setDirection(DcMotor.Direction.FORWARD);

        // servos
        glyph_servo = hardwareMap.get(Servo.class, "glyph_servo");
        glyph_servo.setDirection(Servo.Direction.FORWARD);
        glyph_servo.setPosition(GLYPH_SERVO_OPEN);

        gem_servoA = hardwareMap.get(Servo.class, "gem_servoA");
        gem_servoA.setDirection(Servo.Direction.FORWARD);
        gem_servoA.setPosition(GEM_SERVO_A_UP);

        gem_servoB = hardwareMap.get(Servo.class, "gem_servoB");
        gem_servoB.setDirection(Servo.Direction.REVERSE);
        gem_servoB.setPosition(GEM_SERVO_B_FOLDED);
        // how do we set limits on servo???

//        find sensors on hw map
        // IMU = hardwareMap.get(Sensor.class,"IMU");

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
        double lift = gamepad2.left_stick_y;
        double lifterPower = Range.clip(lift, -1.0, 1.0);
        lifter_motor.setPower(lifterPower);

        // ** glyph servo **
        //    buttons X and B will open or close the grabber on gamepad2
        if (gamepad2.x)
            glyph_servo.setPosition(GLYPH_SERVO_OPEN);
        else if (gamepad2.b)
            glyph_servo.setPosition(GLYPH_SERVO_CLOSED);

//        buttons A and Y will move gem_servoA up and down on gamepad2
        if (gamepad2.a)
            gem_servoA.setPosition(GEM_SERVO_A_DOWN);
        else if (gamepad2.y)
            gem_servoA.setPosition(GEM_SERVO_A_UP);

//        buttons lb and rb will fold and unfold gem_servoB on gamepad2
        if (gamepad2.left_bumper)
            gem_servoB.setPosition(GEM_SERVO_B_OPEN);
        else if (gamepad2.right_bumper)
            gem_servoB.setPosition(GEM_SERVO_B_FOLDED);

        // Telemetry: show elapsed time, wheel power, lifter motor, servos, and possibly sensors
        // This can be whatever we want it to be.  We want info that helps the operators.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", "Lifter motor (%.2f)", leftPower, rightPower, lifterPower);
        telemetry.addData("Gem Servo A pos. (%.2f)", "Gem Servo B pos.", gem_servoA.getPosition(), gem_servoB.getPosition());
        telemetry.addData("Glyph Servo pos. (%.2f)", glyph_servo.getPosition());
//        telemetry.addData("IMU", IMU);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
