package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "dragChain", group = "TeleOp")
public class DragChain extends OpMode{
    private DcMotor dragChain;
    double last_axial = 0.0;
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

    public void init(){
        dragChain = hardwareMap.get(DcMotor.class,"dragchain");

    }
    @Override
    public void loop() {

        double power = -gamepad1.left_stick_y;

        power = applyDeadzone(power);

        // Smooth acceleration/deceleration
        power = skew(power, last_axial);

        // Limit speed to 50%
        power *= 0.5;

        dragChain.setPower(power);

        telemetry.addData("Drag Chain Power", power);
        telemetry.update();
    }
}
