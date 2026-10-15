package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "ShootingTeleOp", group = "TeleOp")
public class ShootingTeleOp extends OpMode {

    // Ticks per revolution of the motor shaft
    static final double Flywheel_TPR = 28.0;
    static final double Feed_TPR     = 28.0 * (84.0 / 29.0);  // 3:1 cartridge (gear ratio) for now, ratio still being tested

    // Shooting speed. D-pad up/down changes the speed whenever its live and needs to be tested. Can finalise later
    double Target_RPM = 3000;
    static final double RPM_Step = 50;
    static final double Min_Shoot_RPM = 500; // minimum RPM allowed for shooting to prevent jamming into a stopped flywheel

    // Speed of the transfer motor while feeding a piece into the flywheel
    static final double Feed_RPM = 200;

    // "Ready to feed" window, in ticks per second.
    static final double Ready_Below = 50;    // allowed under target
    static final double Ready_Above = 100;   // allowed over target
    static final double Settle_Time = 0.15;  // seconds it must stay in the window

    private DcMotorEx Flywheel = null;
    private DcMotorEx Transfer = null; //intake/transfer motor

    boolean Toggle_Mode = true;   // true = press to latch, false = hold to run
    boolean Shooter_On = false;
    boolean Last_Shooter_On = false; // tracks state changes to avoid spamming setMode()
    boolean Last_Fire = false;
    boolean Last_Mode_Btn = false;
    boolean Last_Dpad_Up = false;
    boolean Last_Dpad_Down = false;
    boolean Ready = false;
    ElapsedTime Settle_Timer = new ElapsedTime();

    // Low-pass filter variable to smooth out 28 TPR encoder noise
    double Filtered_Velocity = 0;

    static double rpmToTps(double RPM, double TPR) {
        return RPM / 60.0 * TPR;
    }

    @Override
    public void init() {
        // Config names: gonna change if needed
        Flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        Transfer = hardwareMap.get(DcMotorEx.class, "intake");

        Flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Transfer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Transfer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // If a motor spins the wrong way then this code just flips it
        // Flywheel.setDirection(DcMotor.Direction.REVERSE);
        // Transfer.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        // all the shootings controls in summary, some wont be in final build once testings done
        // gamepad1: right trigger = shoot, back = swap toggle/hold mode,
        //           dpad up/down = target rpm +/- (for testing)
        boolean Fire = gamepad1.right_trigger > 0.5;
        boolean Mode_Btn = gamepad1.back;

        if (Mode_Btn && !Last_Mode_Btn) {
            Toggle_Mode = !Toggle_Mode;
            Shooter_On = false;   // avoid a latched shooter surviving a mode swap
        }

        if (Toggle_Mode) {
            if (Fire && !Last_Fire) Shooter_On = !Shooter_On;   // rising edge only
        } else {
            Shooter_On = Fire;
        }

        if (gamepad1.dpad_up && !Last_Dpad_Up) Target_RPM += RPM_Step;
        if (gamepad1.dpad_down && !Last_Dpad_Down) Target_RPM = Math.max(0, Target_RPM - RPM_Step);

        Last_Fire = Fire;
        Last_Mode_Btn = Mode_Btn;
        Last_Dpad_Up = gamepad1.dpad_up;
        Last_Dpad_Down = gamepad1.dpad_down;

        double Target_TPS = rpmToTps(Target_RPM, Flywheel_TPR);

        // Use Math.abs() to handle magnitude incase its negative or positive so it dosent break
        double Raw_Velocity = Math.abs(Flywheel.getVelocity());

        // Blends 25% of the newest velocity reading with the average smoothed speed so we dont get a false reading and start the ready timer
        Filtered_Velocity = (0.25 * Raw_Velocity) + (0.75 * Filtered_Velocity);

        // Switch RunMode ONLY on state change (when shooter turns ON or OFF) to avoid spamming the REV Hub
        if (Shooter_On != Last_Shooter_On) {
            if (Shooter_On) {
                Flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            } else {
                Flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                Filtered_Velocity = 0; // reset filter when turned off
            }
            Last_Shooter_On = Shooter_On;
        }

        // Run flywheel at target speed when ON, or cut power to 0 when OFF
        if (Shooter_On) {
            Flywheel.setVelocity(Target_TPS);
        } else {
            Flywheel.setPower(0);
        }

        // Checks if the flywheel is spinning at target speed AND target RPM is above minimum shooting speed (prevents jamming into stopped flywheel)
        boolean In_Window = Shooter_On
                && Target_RPM >= Min_Shoot_RPM
                && Filtered_Velocity >= Target_TPS - Ready_Below
                && Filtered_Velocity <= Target_TPS + Ready_Above;

        if (!In_Window) Settle_Timer.reset();
        Ready = In_Window && Settle_Timer.seconds() >= Settle_Time;

        // Feed only when the flywheel is up to speed, will make shooting more accurate
        if (Shooter_On && Ready) {
            Transfer.setVelocity(rpmToTps(Feed_RPM, Feed_TPR));
        } else {
            Transfer.setPower(0);
        }

        telemetry.addData("Shoot mode", Toggle_Mode ? "TOGGLE" : "HOLD");
        telemetry.addData("Shooter", Shooter_On ? "ON" : "OFF");
        telemetry.addData("Flywheel RPM", "%.0f (filtered) / %.0f (raw) / target %.0f",
                Filtered_Velocity / Flywheel_TPR * 60.0,
                Raw_Velocity / Flywheel_TPR * 60.0,
                Target_RPM);
        telemetry.addData("Ready to feed", Ready);
        telemetry.update();
    }

    @Override
    public void stop() {
        Flywheel.setPower(0);
        Transfer.setPower(0);
    }
}
