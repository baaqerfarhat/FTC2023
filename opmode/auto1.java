package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class auto1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor Elev1 = hardwareMap.dcMotor.get("Elev1");
        DcMotor Elev2 = hardwareMap.dcMotor.get("Elev2");

        Servo GripRight = hardwareMap.servo.get("GripRight");
        Servo  GripLeft = hardwareMap.servo.get("GripLeft");
        //open
        GripRight.setPosition(0.06);
        GripLeft.setPosition(0.98);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        Pose2d startpose = new Pose2d(0, 0,0 );
//        ElapsedTime timer = new ElapsedTime();
//
//        drive.setPoseEstimate(startpose);
//
//        new TrajectoryBuilder(new Pose2d())
//                .lineToLinearHeading(new Pose2d(48.3, 10.5, Math.toRadians(90)))
//                .build();
//

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .forward(48.3)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(9.5)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .forward(4)
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .back(6)
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .strafeLeft(9.5)
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(90)))
                .forward(20)
                .build();


        GripRight.setPosition(0.06);
        GripLeft.setPosition(0.98);
        sleep(300);


//        drive.trajectoryBuilder(new Pose2d())
//                .addTemporalMarker(0.1, () -> {
//                    GripRight.setPosition(0.5);
//                    GripLeft.setPosition(0.5);
//                })
//                .forward(48.3)
//                .strafeRight(10.5)
//                .forward(4)
//                .addTemporalMarker(1, () -> {
//                    Elev1.setPower(-0.7);
//                    Elev2.setPower(0.7);
//                })
//                .addTemporalMarker(4, () -> {
//                    //Drop the sercos here; (set position to open) find values to save later
//                    GripRight.setPosition(0.06);
//                    GripLeft.setPosition(0.98);
//
//                })
//                .build();

        waitForStart();

        if(isStopRequested()) return;
        GripRight.setPosition(0.5);
        GripLeft.setPosition(0.5);
        sleep(200);



        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

        GripRight.setPosition(0.06);
        GripLeft.setPosition(0.98);

        drive.followTrajectory(traj4);
        drive.followTrajectory(traj5);

        Elev1.setPower(0.5);
        Elev2.setPower(-0.5);
        sleep(900);

        Elev1.setPower(-0.25);
        Elev2.setPower(0.25);

        drive.turn(Math.toRadians(94));
        sleep(500);

        drive.followTrajectory(traj6);





    }
}

