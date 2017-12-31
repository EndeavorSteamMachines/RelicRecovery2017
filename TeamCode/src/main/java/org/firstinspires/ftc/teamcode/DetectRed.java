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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This class is modified from the "SensorColor" OpMode.
 *
 * Turn on sensor light, read the sensor, compare the result of red and blue.
 * If red > blue, return true, else return false.
 *
 */
public class DetectRed {
    private ElapsedTime runtime = new ElapsedTime();
    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */
    NormalizedColorSensor colorSensor;
    /**
     * The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need something analogous when you
     * use a color sensor on your robot
     */
    View relativeLayout;
    String ball = null;

    /**
     * The runOpMode() method is the root of this LinearOpMode, as it is in all linear opModes.
     * Our implementation here, though is a bit unusual: we've decided to put all the actual work
     * in the main() method rather than directly in runOpMode() itself. The reason we do that is that
     * in this sample we're changing the background color of the robot controller screen as the
     * opmode runs, and we want to be able to *guarantee* that we restore it to something reasonable
     * and palatable when the opMode ends. The simplest way to do that is to use a try...finally
     * block around the main, core logic, and an easy way to make that all clear was to separate
     * the former from the latter in separate methods.
     */
    public boolean run(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
        boolean redFlag = false;

        try {
            redFlag = runSample(hardwareMap, telemetry); // actually execute the sample
        } finally {

        }
        return redFlag;
    }

    protected boolean runSample(HardwareMap hardwareMap, Telemetry telemetry) throws InterruptedException {

        // values is a reference to the hsvValues array.
        boolean redFlag = false;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // turn on light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        runtime.reset();
        while (runtime.seconds() < 5) {

            if (colorSensor instanceof SwitchableLight) {
                SwitchableLight light = (SwitchableLight) colorSensor;
                light.enableLight(!light.isLightOn());
            }

            // Read the sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
//              .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            if (Color.red(color) > Color.blue(color))
                ball = "Red";
            else if (Color.blue(color) > Color.red(color))
                ball = "Blue";
            else
                ball = "Neither";
            telemetry.addData("Detected Ball Color", ball);

            if (Color.red(color) > Color.blue(color))
                redFlag = true;

        }
        // turn off light
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(false);
        }
        return redFlag;
    }
}
