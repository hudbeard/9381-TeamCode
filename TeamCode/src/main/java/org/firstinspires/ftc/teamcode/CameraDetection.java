/* Copyright (c) 2019 FIRST. All rights reserved.
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

/*package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class CameraDetection {
    private TfodProcessor tfod;

    private VisionPortal pixelVisionPortal;

    private final ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry;
    @SuppressLint("SdCardPath")
    public void initTfod(@NonNull HardwareMap hwmap, Telemetry t, boolean red) {
        telemetry = t;
        if (red) {

            String[] LABELS = {"RED_PROP"};
            tfod = new TfodProcessor.Builder()
                    .setModelFileName("/sdcard/FIRST/tflitemodels/RED_TSE-2.tflite")
                    .setModelLabels(LABELS)
                    .setIsModelTensorFlow2(true)
                    .setIsModelQuantized(true)
                    .setModelInputSize(300)
                    .setModelAspectRatio(16.0 / 9.0)
                    .build();
        } else {

            String[] LABELS = {"BLUE_PROP"};
            tfod = new TfodProcessor.Builder()
                    .setModelFileName("/sdcard/FIRST/tflitemodels/BLUE_TSE-2.tflite")
                    .setModelLabels(LABELS)
                    .setIsModelTensorFlow2(true)
                    .setIsModelQuantized(true)
                    .setModelInputSize(300)
                    .setModelAspectRatio(16.0 / 9.0)
                    .build();
        }

        VisionPortal.Builder pixel_builder = new VisionPortal.Builder();


        pixel_builder.setCamera(hwmap.get(WebcamName.class, "Cam 1"));


        pixel_builder.enableLiveView(false);

        pixel_builder.addProcessor(tfod);

        pixelVisionPortal = pixel_builder.build();

        tfod.setMinResultConfidence(0.7f);
        pixelVisionPortal.setProcessorEnabled(tfod, true);
    }
    public Integer detect() {
        int first = 0;
        int second = 0;
        int third = 0;
        pixelVisionPortal.resumeStreaming();
        for (int i=0; i < 50; i++) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            for (Recognition recognition : currentRecognitions) {
                double y = (recognition.getLeft() + recognition.getRight()) / 2;
                double threshold = recognition.getImageWidth() / 2.0;
                if (y < threshold) {first++;}
                else {second++;}
            }
            runtime.reset();
            while (runtime.seconds() < 0.01) {
                telemetry.update();
            }
        }
        pixelVisionPortal.stopStreaming();
        if (first > second && first > third) {return 1;}
        else if (second > first && second > third) {return 2;}
        else {return 3;}
    }
}
*/