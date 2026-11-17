package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BIOBUZZ Final TeleOp", group = "TeleOp")
public class FinalTeleOp extends OpMode {

    // TO DO LIST
    // MOVEMENT
    // ODOMETRY (with marwan)
    // LIMELIGHT TRACKING (with marwan)
    //
    // Motors for Movement: GoBILDA 5203 Series Yellow Jacket
    // 435 RPM @ 12V
    // 13.7:1 Ratio, 24mm Length


    //Button Configurations

// Triangle
// Circle
// Cross
// Square
// Up
// Down
// Left
// Right
// L1
// R1
// L2
// R2
// L3
// R3
// PS Button
// Create Button
// Options Button

    //Analogue
 // L Stick
 // R Stick


   // Other:
// Gyroscope
// Accelerometer
// Touchpad Press

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    @Override
    public void init() {

        // initialise movement motors
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // set motor dir
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        // brake if no input
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // run without encoder config
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "BIOBUZZ Initialized!");
        telemetry.update();
    }

    @Override
    public void loop() {

        // movement controls
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        // prevent motor power to reach 1
        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rotation),
                1.0
        );

        // mecanum calculations
        double frontLeftPower = (y + x + rotation) / denominator;
        double frontRightPower = (y - x - rotation) / denominator;
        double backLeftPower = (y - x + rotation) / denominator;
        double backRightPower = (y + x - rotation) / denominator;

        // slow down trigger
        double speedMultiplier = gamepad1.right_bumper ? 0.35 : 1.0;

        // apply motor powers
        frontLeft.setPower(frontLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);

        // telemetry
        telemetry.addData("Status", "BIOBUZZ Driving!");
        telemetry.addData("Speed", speedMultiplier);
        telemetry.addData("Front Left", frontLeftPower * speedMultiplier);
        telemetry.addData("Front Right", frontRightPower * speedMultiplier);
        telemetry.addData("Back Left", backLeftPower * speedMultiplier);
        telemetry.addData("Back Right", backRightPower * speedMultiplier);
        telemetry.update();
    }

    @Override
    public void stop() {

        // stop all
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
