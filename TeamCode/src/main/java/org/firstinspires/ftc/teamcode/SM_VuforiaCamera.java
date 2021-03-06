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

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This class is modified from "ConceptVuMarkIdentification".
 *
 * Initialize phone camera, allow a few seconds to retrieve vuMark.
 */
public class SM_VuforiaCamera {

    // Constructor
    public RelicRecoveryVuMark vuMark = null;
    VuforiaLocalizer vuforia;
    Telemetry telemetry;
    //  timer
    private ElapsedTime runtime = new ElapsedTime();

    public RelicRecoveryVuMark run(HardwareMap hardwareMap, Telemetry telemetry, double waitTime) {

        // initialize vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        //Set license key to use Vuforia. License key obtained from:  https://developer.vuforia.com/license-manager
        parameters.vuforiaLicenseKey = "AY3ovAD/////AAAAGQXI9N445U53kBfK81NQP/pQpUnLvJUPXH2JmcHCVjr7wvMtXNGkpxGLrknlRCmgQofpRFzSm0vc3DDmsOYR4DRgSpDQlm1KDGZnbUWCWZP87r" +
                "A5aeKCmsOFYn98cDxxUGtjRhXQBmID47FDBLo5+YG+eaWbUJCqyfheOY3rjlLitL2IN70O3wUijWnXUNCXCtPXON9iBltMmAwAf8vjTSKBK0XE+C69c08zY9pdFSRDvciQ83zDD/gFd8OYwyUXYAr" +
                "XeS5LPilNiUiibIC4X/t0t5nTdiA7q9R2qjUcd4Sbm6T5vSJH5O9TXSWh2sroqTBEtqtFtP0MPRjP9HfYiFjtbHHQlt5beQdOUvCvPj/b";
        // Choose camera on back of phone.
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /**
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        relicTrackables.activate(); // don't know what this does, but it was in the example code

        // reset the timer
        runtime.reset();
        // retrieve vuMark
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        // wait until either VuMark is identified or waitTime is exceeded
        while (vuMark == RelicRecoveryVuMark.UNKNOWN && runtime.seconds() < waitTime) {
            telemetry.addData("VuMark", "trying to identify");
            telemetry.update();
            vuMark = RelicRecoveryVuMark.from(relicTemplate);
        }
        relicTrackables.deactivate();
        return vuMark;
    }
}

