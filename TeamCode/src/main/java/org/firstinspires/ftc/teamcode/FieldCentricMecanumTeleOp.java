package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    private final int CLIMBER_UP_POS = 2000;
    private final int CLIMBER_DOWN_POS = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        DcMotor climbMotor = hardwareMap.dcMotor.get("climbMotor");

        // Declare our servo
        Servo claw = hardwareMap.servo.get("claw");

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Resetting climber encoder to 0
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Default settings for motor
        climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double leftY = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double leftX = gamepad1.left_stick_x;
            double rightX = gamepad1.right_stick_x;
            boolean buttonY = gamepad1.y;
            boolean buttonX = gamepad1.x;
            double leftTrigger = gamepad1.left_trigger;
            double rightTrigger = gamepad1.right_trigger;
            boolean upButton = gamepad1.dpad_up;
            boolean downButton = gamepad1.dpad_down;

            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            drive.updatePoseEstimate();

            // drivetrain

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.

            // following two "imu" commands might cause a problem

            if (gamepad1.options) {
                imu.resetYaw();
            }

            // claw
            if (buttonY) {
                claw.setPosition(0.25);
            }
            if (buttonX) {
                claw.setPosition(1);
            }

            // REMEMBER YOU HAVE TO PRESS UP ON DPAD BEFORE IT WORKS
            if (upButton) {
                climbMotor.setTargetPosition(CLIMBER_UP_POS);
                climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               // climbMotor.setPower(climberPower);
            }
            if (downButton) {
                climbMotor.setTargetPosition(CLIMBER_DOWN_POS);
                climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               // climbMotor.setPower(climberPower);
            }
            if (gamepad1.right_trigger > 0) {
                climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climbMotor.setPower(gamepad1.right_trigger);
            }
            if (gamepad1.left_trigger > 0) {
                climbMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                climbMotor.setPower(-gamepad1.left_trigger);
            }

            // logging

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));

            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Left Trigger", leftTrigger);
            telemetry.addData("Right Trigger", rightTrigger);
            telemetry.addData("Climb Motor Power", climbMotor.getPower());
            telemetry.addData("Climb Current Position", climbMotor.getCurrentPosition());
            telemetry.addData("Climb Target Position", climbMotor.getTargetPosition());

            telemetry.update();
        }
    }
}