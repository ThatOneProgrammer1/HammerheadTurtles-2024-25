package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class FieldCentricMecanumTeleOp extends LinearOpMode {
    private final int CLIMBER_UP_POS = 2000;
    private final int CLIMBER_DOWN_POS = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor climbMotor = hardwareMap.dcMotor.get("climbMotor");

        // Declare our servo
        Servo claw = hardwareMap.servo.get("claw");



        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // set climber direction
        climbMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Resetting climber encoder to 0
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        // Default settings for motor
        climbMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // todo Switch to Run to position

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

            // drivetrain

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = leftX * Math.cos(-botHeading) - leftY * Math.sin(-botHeading);
            double rotY = leftX * Math.sin(-botHeading) + leftY * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rightX), 1);
            double frontLeftPower = (rotY + rotX + rightX) / denominator;
            double backLeftPower = (rotY - rotX + rightX) / denominator;
            double frontRightPower = (rotY - rotX - rightX) / denominator;
            double backRightPower = (rotY + rotX - rightX) / denominator;

            // claw
            if (buttonY) {
                claw.setPosition(0.25);
            }
            if (buttonX) {
                claw.setPosition(1);
            }

            // climb
            double climberPower = (rightTrigger - leftTrigger) * .75;
            if (upButton) {
                climbMotor.setTargetPosition(CLIMBER_UP_POS);
            }
            if (downButton) {
                climbMotor.setTargetPosition(CLIMBER_DOWN_POS);
            }

            // set motors

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
//            climbMotor.setPower(climberPower);

            // logging

            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Climb Power", climberPower);
            telemetry.addData("Left Trigger", leftTrigger);
            telemetry.addData("Right Trigger", rightTrigger);
            telemetry.addData("Climb Current Position", climbMotor.getCurrentPosition());
            telemetry.addData("Climb Target Position", climbMotor.getTargetPosition());

            telemetry.update();
        }
    }
}