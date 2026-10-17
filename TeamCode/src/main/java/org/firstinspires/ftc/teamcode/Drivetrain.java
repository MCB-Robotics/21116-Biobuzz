package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Driving", group = "TeleOp")
public class Drivetrain extends OpMode {

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    double last_axial = 0.0;
    double last_lateral = 0.0;
    double last_yaw = 0.0;
    double MAX_CHANGE = 0.3;

    public double applyDeadzone(double joystick) {
        double deadzone_boundary = 0.1;
        if (joystick < deadzone_boundary && joystick > -deadzone_boundary) {
            return 0.0;
        } else {
            return joystick;
        }
    }

    public double skew(double input, double last_input) {
        if (Math.signum(input) == 1 && input > last_input + MAX_CHANGE) {
            return last_input + MAX_CHANGE;
        } else if (Math.signum(input) == -1 && input < last_input - MAX_CHANGE) {
            return last_input - MAX_CHANGE;
        } else {
            return input;
        }
    }

    @Override
    public void init() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop() {

        double axial = applyDeadzone(-gamepad1.left_stick_y);
        double lateral = applyDeadzone(gamepad1.left_stick_x);
        double yaw = applyDeadzone(gamepad1.right_stick_x);

        axial   = Math.copySign(axial * axial, axial);
        lateral = Math.copySign(lateral * lateral, lateral);
        yaw     = Math.copySign(yaw * yaw, yaw);

        // Ramping (skew)
        axial = skew(axial, last_axial);
        lateral = skew(lateral, last_lateral);
        yaw = skew(yaw, last_yaw);

        last_axial = axial;
        last_lateral = lateral;
        last_yaw = yaw;

        // Mecanum math
        double leftfrontPower = axial + lateral + yaw;
        double rightfrontPower = axial - lateral - yaw;
        double leftbackPower = axial - lateral + yaw;
        double rightbackPower = axial + lateral - yaw;

        // NORMALIZATION
        double max = Math.max(
                Math.max(Math.abs(leftfrontPower), Math.abs(rightfrontPower)),
                Math.max(Math.abs(leftbackPower), Math.abs(rightbackPower))
        );

        if (max > 1.0) {
            leftfrontPower  /= max;
            rightfrontPower /= max;
            leftbackPower   /= max;
            rightbackPower  /= max;
        }

        // Send power to motors
        leftFrontDrive.setPower(leftfrontPower);
        rightFrontDrive.setPower(rightfrontPower);
        leftBackDrive.setPower(leftbackPower);
        rightBackDrive.setPower(rightbackPower);
    }
}
