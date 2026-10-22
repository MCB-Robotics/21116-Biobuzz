package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "ServoTestTeleOp", group = "BioBuzz")
public class ServoTestTeleOp extends LinearOpMode {

    private static class ServoEntry {
        String name;
        Servo servo;
        double position = 0.5;

        ServoEntry(String name, Servo servo) {
            this.name = name;
            this.servo = servo;
        }
    }

    // Add new servo configuration names here.
    // Names must match the Robot Configuration exactly.
    private static final String[] SERVO_NAMES = {
            "servo1",
            "servo2",
            "servo3",
            "servo4",
            "servo5",
            "servo6",
            "servo7",
            "servo8"
    };

    private final List<ServoEntry> servos = new ArrayList<>();

    private int selectedServo = 0;

    private static final double ADJUST_SPEED = 0.5;
    private static final double DEADZONE = 0.10;

    @Override
    public void runOpMode() {

        // Automatically initialize every servo found in the configuration.
        for (String name : SERVO_NAMES) {
            try {
                Servo servo = hardwareMap.get(Servo.class, name);
                servos.add(new ServoEntry(name, servo));

                telemetry.addData("Found", name);

            } catch (IllegalArgumentException e) {
                telemetry.addData("Skipped", name + " (not configured)");
            }
        }

        telemetry.addData("Available servos", servos.size());

        if (servos.isEmpty()) {
            telemetry.addLine("ERROR: No configured servos found.");
            telemetry.addLine("Check the Robot Configuration names.");
            telemetry.update();
            return;
        }

        telemetry.addLine("");
        telemetry.addLine("D-pad LEFT/RIGHT: Select servo");
        telemetry.addLine("Left stick X: Move selected servo");
        telemetry.addLine("Cross (X): Center selected servo");
        telemetry.addLine("Circle: Return to first servo");
        telemetry.addLine("All servos start at commanded position 0.5");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Command an initial position to each available servo.
        for (ServoEntry entry : servos) {
            entry.servo.setPosition(entry.position);
        }

        long previousTime = System.nanoTime();

        boolean previousLeft = false;
        boolean previousRight = false;
        boolean previousCross = false;
        boolean previousCircle = false;

        while (opModeIsActive()) {

            long now = System.nanoTime();
            double dt = (now - previousTime) / 1_000_000_000.0;
            previousTime = now;

            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
            boolean cross = gamepad1.a;
            boolean circle = gamepad1.b;

            // Cycle through all available servos.
            if (right && !previousRight) {
                selectedServo =
                        (selectedServo + 1) % servos.size();
            }

            if (left && !previousLeft) {
                selectedServo =
                        (selectedServo - 1 + servos.size())
                                % servos.size();
            }

            // Return selection to the first available servo.
            if (circle && !previousCircle) {
                selectedServo = 0;
            }

            ServoEntry active = servos.get(selectedServo);

            // Center only the selected servo.
            if (cross && !previousCross) {
                active.position = 0.5;
                active.servo.setPosition(active.position);
            }

            previousLeft = left;
            previousRight = right;
            previousCross = cross;
            previousCircle = circle;

            // Adjust only the selected servo.
            double input = gamepad1.left_stick_x;

            if (Math.abs(input) < DEADZONE) {
                input = 0.0;
            }

            active.position += input * ADJUST_SPEED * dt;
            active.position = Math.max(
                    0.0, Math.min(1.0, active.position)
            );

            active.servo.setPosition(active.position);

            // Driver Station telemetry.
            telemetry.addLine("=== BioBuzz Servo Test ===");
            telemetry.addData(
                    "Available servos", servos.size()
            );
            telemetry.addData(
                    "Selected servo", "%d / %d",
                    selectedServo + 1, servos.size()
            );
            telemetry.addData("Name", active.name);
            telemetry.addData(
                    "Position", "%.3f", active.position
            );

            telemetry.addLine("");
            telemetry.addLine("All configured servos:");

            for (int i = 0; i < servos.size(); i++) {
                ServoEntry entry = servos.get(i);

                telemetry.addData(
                        (i == selectedServo ? "> " : "  ")
                                + entry.name,
                        "%.3f", entry.position
                );
            }

            telemetry.addLine("");
            telemetry.addLine("D-pad LEFT/RIGHT: Select");
            telemetry.addLine("Left stick X: Move");
            telemetry.addLine("Cross (X): Center");
            telemetry.addLine("Circle: First servo");

            telemetry.update();
        }
    }
}
