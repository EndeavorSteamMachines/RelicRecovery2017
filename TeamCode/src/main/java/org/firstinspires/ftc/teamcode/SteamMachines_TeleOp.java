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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller OpMode
 *
 * What we control:
 *
 *   Motors
 *    left_drive
 *    right_drive
 *    lifter_drive
 *
 *   Servos
 *    gem_servoA
 *    gem_servoB
 *    glyph_servo
 *
 *   Sensors
 *    gem_sensor
 *    camera
 *
 *
 * How we control:
 *
 *   Gamepad1 will control the left_drive and right_drive.
 *   Gamepad2 will control everything else.
 *
 *   Gamepad1 definitions:
 *    left_stick_y (up and down) controls forward or backward
 *    right_stick_x (side to side) controls turning
 *
 *   Gamepad2 definitions:
 *    left_stick_y (up and down) controls lifter up and down
 *    left_bumper (press) opens grabber
 *    right_bumper (press) closes grabber
 *
 *    ? button (press) gem_servoA up and down
 *    ? button (press) gem_servoB forward (knock off ball closest to front of robot)
 *    ? button (press) gem_servoB backward (knock off ball closest to back of robot)
 **/

@TeleOp(name="SteamMachines TeleOp Mode", group="Iterative Opmode")
//@Disabled
public class SteamMachines_TeleOp extends OpMode
{
    // instantiate motors, servos and sensor objects
    // motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left_drive = null;
    private DcMotor right_drive = null;
    private DcMotor lifter_drive = null;
    // servos
    private Servo gem_servoA = null;
    private Servo gem_servoB = null;
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
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
        left_drive.setDirection(DcMotor.Direction.FORWARD);

        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_drive.setDirection(DcMotor.Direction.REVERSE);

        lifter_drive = hardwareMap.get(DcMotor.class,"lifter_motor");
        lifter_drive.setDirection(DcMotor.Direction.FORWARD);

        // servos
        gem_servoA = hardwareMap.get(Servo.class,"gem_servoA");
        gem_servoB = hardwareMap.get(Servo.class,"gem_servoB");
        // sensors
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

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // drive motors
        //    left_stick_y (up and down) controls forward or backward
        //    right_stick_x (side to side) controls turning

        // turn and drive depend on the current position of the joysticks
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        // use turn and drive to determine amount of power to apply to motors
        double leftPower = Range.clip(drive + turn, -1.0, 1.0);
        double rightPower = Range.clip(drive - turn, -1.0, 1.0);
        // set motor power
        left_drive.setPower(leftPower);
        right_drive.setPower(rightPower);

        // lifter motor
        //    left_stick_y (up and down) controls lifter up and down
        double lift = gamepad2.left_stick_y;
        double lifterPower = Range.clip(lift, -1.0, 1.0);
        lifter_drive.setPower(lifterPower);

        // gem sensors
        if (gamepad2.left_bumper)
            gem_servoA.setPosition(1);
        else if (gamepad2.x)
            gem_servoA.setPosition(0);

        if (gamepad2.right_bumper)
            gem_servoB.setPosition(1);
        else if (gamepad2.y)
            gem_servoB.setPosition(0);



        // Telemetry: show elapsed time, wheel power, lifter motor
        // This can be whatever we want it to be.  We want info that helps the drivers.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("lifter motor (%.2f)", lifterPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
