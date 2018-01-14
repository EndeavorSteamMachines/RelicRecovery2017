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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * FTC ENDEAVOR STEAM MACHINE Robot Controller Autonomous Mode R1
 * <p>
 * Autonomous code for when robot is in starting position R1.
 * This is the red balancing stone closest to the relic recovery zone.
 **/

@Autonomous(name = "Autonomous R1", group = "Autonomous")
@Disabled
public class SM_Autonomous_R1 extends LinearOpMode {
    // Constructor: instantiate objects used in this class.
    SM_StartCodes.Position startingPosition = SM_StartCodes.Position.R1;
    SM_StartCodes.Color startingColor = SM_StartCodes.Color.Red;

    @Override
    public void runOpMode() throws InterruptedException {

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        telemetry.addData("Status", "PLAY has been pressed");
        telemetry.update();
        // another option for waiting?
//        while (!(isStarted() || isStopRequested())) {
//            telemetry.addData("Status", "Waiting for start");
//            telemetry.update();
//            idle();
//        }

        SM_Robot robot = new SM_Robot(hardwareMap, telemetry);
        RelicRecoveryVuMark cryptoKey;

        cryptoKey = robot.Task1();
//        robot.Task2(startingColor);
        robot.Task3(startingPosition, cryptoKey);

    }


} // end of class (no code beyond this point)