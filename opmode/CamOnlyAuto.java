/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.opmode.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
//code
import java.util.ArrayList;

@Autonomous
public class CamOnlyAuto extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag Ids Of the custom Sleeve
    int Left = 1;
    int Middle = 2;
    int Right = 3;

    AprilTagDetection tagOfInterest = null;

    DcMotorEx motor0;
    DcMotorEx motor1;
    DcMotorEx motor2;
    DcMotorEx motor3;
    DcMotor Elev1;

    @Override
    public void runOpMode()
    {

        motor3 = hardwareMap.get(DcMotorEx.class, "motor3");
        motor2 = hardwareMap.get(DcMotorEx.class, "motor2");
        motor1 = hardwareMap.get(DcMotorEx.class, "motor1");
        motor0 = hardwareMap.get(DcMotorEx.class, "motor0");

        DcMotor Elev1 = hardwareMap.dcMotor.get("Elev1");
        DcMotor Elev2 = hardwareMap.dcMotor.get("Elev2");

        Servo GripRight = hardwareMap.servo.get("GripRight");
        Servo  GripLeft = hardwareMap.servo.get("GripLeft");

//tick 538
        //ticks 538
        motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

        motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brakes
        motor0.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Elev1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Elev2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//open
        GripRight.setPosition(0.06);
        GripLeft.setPosition(0.98);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == Left || tag.id == Middle || tag.id == Right)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        if (tagOfInterest == null){

            //Camera didnt detct, run cone delivery code and park in the middle
        } else if (tagOfInterest.id == Middle){


            //Put code here that would end up parking in the Middle (pos 2)
            //close
            GripRight.setPosition(0.5);
            GripLeft.setPosition(0.5);
            sleep(700);


            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            sleep(1300);


            Elev1.setPower(0);
            Elev2.setPower(0);
            sleep(100);

            //move fowards

            motor3.setTargetPosition(-2200);
            //motor3 was positive
            motor2.setTargetPosition(-2200);
            motor0.setTargetPosition(2200);
            motor1.setTargetPosition(2200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(0.7);
            motor1.setPower(0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(3300);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1500);

            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            // sleep(1950);
            // //reduce the time?


            // Elev1.setPower(0.7);
            // Elev2.setPower(0.7);
            //sleep(100);
            //reduce time?


            //Strafe Right to deliver cone

            motor3.setTargetPosition(-585);
            //motor3 was positive
            motor2.setTargetPosition(585);
            motor0.setTargetPosition(-585);
            motor1.setTargetPosition(585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(-0.5);
            motor0.setPower(0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);




            //move fowards

            motor3.setTargetPosition(-270);
            //motor3 was positive
            motor2.setTargetPosition(-270);
            motor0.setTargetPosition(270);
            motor1.setTargetPosition(270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(0.5);
            motor1.setPower(0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



//Drop the cone

            GripRight.setPosition(0.06);
            GripLeft.setPosition(0.98);
            sleep(300);




//Exact opposite


            //move backwards

            motor3.setTargetPosition(270);
            //motor3 was positive
            motor2.setTargetPosition(270);
            motor0.setTargetPosition(-270);
            motor1.setTargetPosition(-270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



            //Strafe Left to deliver park

            motor3.setTargetPosition(585);
            //motor3 was positive
            motor2.setTargetPosition(-585);
            motor0.setTargetPosition(585);
            motor1.setTargetPosition(-585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(0.5);
            motor3.setPower(-0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);

//move backwards

            motor3.setTargetPosition(1000);
            //motor3 was positive
            motor2.setTargetPosition(1000);
            motor0.setTargetPosition(-1000);
            motor1.setTargetPosition(-1000);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(-0.7);
            motor1.setPower(-0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(2500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);
        } else if (tagOfInterest.id == Left){
            //Put code here to deliver 2 cones and end at the left parking section (pos 1)
            //close
            GripRight.setPosition(0.5);
            GripLeft.setPosition(0.5);
            sleep(700);


            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            sleep(1300);


            Elev1.setPower(0);
            Elev2.setPower(0);
            sleep(100);

            //move fowards

            motor3.setTargetPosition(-2200);
            //motor3 was positive
            motor2.setTargetPosition(-2200);
            motor0.setTargetPosition(2200);
            motor1.setTargetPosition(2200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(0.7);
            motor1.setPower(0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(3300);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1500);

            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            // sleep(1950);
            // //reduce the time?


            // Elev1.setPower(0.7);
            // Elev2.setPower(0.7);
            //sleep(100);
            //reduce time?


            //Strafe Right to deliver cone

            motor3.setTargetPosition(-585);
            //motor3 was positive
            motor2.setTargetPosition(585);
            motor0.setTargetPosition(-585);
            motor1.setTargetPosition(585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(-0.5);
            motor0.setPower(0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);




            //move fowards

            motor3.setTargetPosition(-270);
            //motor3 was positive
            motor2.setTargetPosition(-270);
            motor0.setTargetPosition(270);
            motor1.setTargetPosition(270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(0.5);
            motor1.setPower(0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



//Drop the cone

            GripRight.setPosition(0.06);
            GripLeft.setPosition(0.98);
            sleep(300);




//Exact opposite


            //move backwards

            motor3.setTargetPosition(270);
            //motor3 was positive
            motor2.setTargetPosition(270);
            motor0.setTargetPosition(-270);
            motor1.setTargetPosition(-270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



            //Strafe Left to deliver park

            motor3.setTargetPosition(585);
            //motor3 was positive
            motor2.setTargetPosition(-585);
            motor0.setTargetPosition(585);
            motor1.setTargetPosition(-585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(0.5);
            motor3.setPower(-0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);

//move backwards

            motor3.setTargetPosition(1000);
            //motor3 was positive
            motor2.setTargetPosition(1000);
            motor0.setTargetPosition(-1000);
            motor1.setTargetPosition(-1000);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(-0.7);
            motor1.setPower(-0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(2500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);

//move Strafe to left

            motor3.setTargetPosition(1250);
            //motor3 was positive
            motor2.setTargetPosition(-1250);
            motor0.setTargetPosition(1250);
            motor1.setTargetPosition(-1250);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(-0.7);
            motor1.setPower(0.7);
            motor3.setPower(-0.7);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);



        } else if (tagOfInterest.id == Right){

            //Put code here to deliver 2 cones and end at the right parking section (pos 3)
            //close
            GripRight.setPosition(0.5);
            GripLeft.setPosition(0.5);
            sleep(700);


            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            sleep(1300);


            Elev1.setPower(0);
            Elev2.setPower(0);
            sleep(100);

            //move fowards

            motor3.setTargetPosition(-2200);
            //motor3 was positive
            motor2.setTargetPosition(-2200);
            motor0.setTargetPosition(2200);
            motor1.setTargetPosition(2200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(0.7);
            motor1.setPower(0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(3300);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1500);

            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            // sleep(1950);
            // //reduce the time?


            // Elev1.setPower(0.7);
            // Elev2.setPower(0.7);
            //sleep(100);
            //reduce time?


            //Strafe Right to deliver cone

            motor3.setTargetPosition(-585);
            //motor3 was positive
            motor2.setTargetPosition(585);
            motor0.setTargetPosition(-585);
            motor1.setTargetPosition(585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(-0.5);
            motor0.setPower(0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);




            //move fowards

            motor3.setTargetPosition(-270);
            //motor3 was positive
            motor2.setTargetPosition(-270);
            motor0.setTargetPosition(270);
            motor1.setTargetPosition(270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(0.5);
            motor1.setPower(0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



//Drop the cone

            GripRight.setPosition(0.06);
            GripLeft.setPosition(0.98);
            sleep(300);




//Exact opposite


            //move backwards

            motor3.setTargetPosition(270);
            //motor3 was positive
            motor2.setTargetPosition(270);
            motor0.setTargetPosition(-270);
            motor1.setTargetPosition(-270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



            //Strafe Left to deliver park

            motor3.setTargetPosition(585);
            //motor3 was positive
            motor2.setTargetPosition(-585);
            motor0.setTargetPosition(585);
            motor1.setTargetPosition(-585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(0.5);
            motor3.setPower(-0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);

//move backwards

            motor3.setTargetPosition(1000);
            //motor3 was positive
            motor2.setTargetPosition(1000);
            motor0.setTargetPosition(-1000);
            motor1.setTargetPosition(-1000);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(-0.7);
            motor1.setPower(-0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(2500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);

//move Strafe to Right

            motor3.setTargetPosition(-1270);
            //motor3 was positive
            motor2.setTargetPosition(1270);
            motor0.setTargetPosition(-1270);
            motor1.setTargetPosition(1270);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(-0.7);
            motor0.setPower(0.7);
            motor1.setPower(-0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);




        } else {
            //Deliver 2 cones and take the 1/3 chance


            //Put code here that would end up parking in the Middle (pos 2)
            //close
            GripRight.setPosition(0.5);
            GripLeft.setPosition(0.5);
            sleep(700);


            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            sleep(1300);


            Elev1.setPower(0);
            Elev2.setPower(0);
            sleep(100);

            //move fowards

            motor3.setTargetPosition(-2200);
            //motor3 was positive
            motor2.setTargetPosition(-2200);
            motor0.setTargetPosition(2200);
            motor1.setTargetPosition(2200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(0.7);
            motor1.setPower(0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(3300);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(1500);

            //up a bit
            Elev1.setPower(-0.7);
            Elev2.setPower(0.7);
            // sleep(1950);
            // //reduce the time?


            // Elev1.setPower(0.7);
            // Elev2.setPower(0.7);
            //sleep(100);
            //reduce time?


            //Strafe Left to deliver cone

            motor3.setTargetPosition(765);
            //motor3 was positive
            motor2.setTargetPosition(-765);
            motor0.setTargetPosition(765);
            motor1.setTargetPosition(-765);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(0.5);
            motor3.setPower(-0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);




            //move fowards

            motor3.setTargetPosition(-200);
            //motor3 was positive
            motor2.setTargetPosition(-200);
            motor0.setTargetPosition(200);
            motor1.setTargetPosition(200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(0.5);
            motor1.setPower(0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



//Drop the cone

            GripRight.setPosition(0.06);
            GripLeft.setPosition(0.98);
            sleep(300);




//Exact opposite


            //move backwards

            motor3.setTargetPosition(200);
            //motor3 was positive
            motor2.setTargetPosition(200);
            motor0.setTargetPosition(-200);
            motor1.setTargetPosition(-200);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.5);
            motor0.setPower(-0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(1500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);



            //Strafe Right to deliver park

            motor3.setTargetPosition(-585);
            //motor3 was positive
            motor2.setTargetPosition(585);
            motor0.setTargetPosition(-585);
            motor1.setTargetPosition(585);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(-0.5);
            motor0.setPower(0.5);
            motor1.setPower(-0.5);
            motor3.setPower(0.5);
            //was at 0.1

            sleep(2000);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(700);

//move backwards

            motor3.setTargetPosition(1000);
            //motor3 was positive
            motor2.setTargetPosition(1000);
            motor0.setTargetPosition(-1000);
            motor1.setTargetPosition(-1000);

            motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor0.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
            motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);

            motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            motor2.setPower(0.7);
            motor0.setPower(-0.7);
            motor1.setPower(-0.7);
            motor3.setPower(0.7);
            //was at 0.1

            sleep(2500);



            motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor0.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(800);

        }


        while (opModeIsActive()) {sleep(20);}
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}