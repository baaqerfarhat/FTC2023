package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class TrajectoryAuto1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor Elev1 = hardwareMap.dcMotor.get("Elev1");
        DcMotor Elev2 = hardwareMap.dcMotor.get("Elev2");

        Servo GripRight = hardwareMap.servo.get("GripRight");
        Servo  GripLeft = hardwareMap.servo.get("GripLeft");
        //open
        GripRight.setPosition(0.06);
        GripLeft.setPosition(0.98);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-35, -60, -4.71229);
        Pose2d PoseAfterTraj1 = new Pose2d(11.3, -50.5, -4.71229);


        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .forward(48.3)
                .addTemporalMarker(0.01, () -> {
                    //Runs 0.1 seconds into trajectory
                    GripRight.setPosition(0.5);
                    GripLeft.setPosition(0.5);
                    // Run your action in here!
                    Elev1.setPower(-0.6);
                    Elev2.setPower(0.6);

                })
                .strafeRight(9.5)
                .forward(4)
                .addTemporalMarker(3.7, () -> {
                    //Runs 3.7 seconds into trajectory
                    GripRight.setPosition(0.06);
                    GripLeft.setPosition(0.98);
                })
                //.waitSeconds(3)
                .back(6)
                .waitSeconds(1)
                .build();

        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(PoseAfterTraj1)
                .lineToLinearHeading(new Pose2d(-59, -11.5, Math.toRadians(180)))
                        .build();


        waitForStart();

        if (!isStopRequested())
            drive.followTrajectorySequence(trajSeq);
            drive.followTrajectorySequence(traj2);
    }
}