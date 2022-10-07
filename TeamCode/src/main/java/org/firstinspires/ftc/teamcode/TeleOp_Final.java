/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// GAMEPAD GUIDE    https://imgur.com/a/znvOv20
// WIRING GUIDE     https://imgur.com/a/4BepqJK

@TeleOp(name="TeleOp Final", group="Linear Opmode")
// @Disabled
public class TeleOp_Final extends LinearOpMode {

    // Declare OpMode objects
    // Ports in comments are ports for default config
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDriveFront;                             // motor port 0 (control hub)
    DcMotor leftDriveRear;                              // motor port 1 (control hub)
    DcMotor rightDriveFront;                            // motor port 2 (control hub)
    DcMotor rightDriveRear;                             // motor port 3 (control hub)
    DcMotor intakeMotor;                                // motor port 2 (expansion hub)
    DcMotor elevatorPulldownMotorMaster;                // motor port 0 (expansion hub)
    DcMotor elevatorPulldownMotorSlave;                 // motor port 3 (expansion hub)
    DcMotor elevatorSlideMotor;                         // motor port 1 (expansion hub)
    Servo   unloadServo;                                // servo port 0 (control hub)

    // Constants
    public double DRIVETRAIN_POWER_MODIFIER = 1;        // 0 - 1, 1 = no change
    public double ELEVATOR_PULLDOWN_POWER_UP = 1;       // 0 - 1
    public double ELEVATOR_PULLDOWN_POWER_DOWN = 1;     // 0 - 1
    public double ELEVATOR_SLIDE_POWER_UP = 1;          // 0 - 1
    public double ELEVATOR_SLIDE_POWER_DOWN = 0.55;     // 0 - 1
    public double UNLOAD_SERVO_POSITION_UP = 0.45;      // 0 - 1
    public double UNLOAD_SERVO_POSITION_DOWN = 0;       // 0 - 1

    @Override
    public void runOpMode() {

        // Variable for each motor to save power level for telemetry and setting motor power
        double leftFrontPower;                                      // left front motor power
        double leftRearPower;                                       // left rear motor power
        double rightFrontPower;                                     // right front motor power
        double rightRearPower;                                      // right rear motor power
        double intakeMotorPower = 0;                                // intake motor power
        double elevatorPulldownMotorPower = 0;                      // elevator pulldown motors power
        double elevatorSlideMotorPower = 0;                         // elevator slide motor power
        double unloadServoPosition = UNLOAD_SERVO_POSITION_DOWN;    // unloading carbon servo position (used on init)
        boolean isRightTriggerPressed;                              // saves right trigger position
        boolean isLeftTriggerPressed;                               // saves left trigger position

        // Map devices to objects
        leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        elevatorPulldownMotorMaster = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorSlideMotor = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        elevatorPulldownMotorSlave = hardwareMap.get(DcMotor.class, "elevatorMotor3");
        unloadServo = hardwareMap.get(Servo.class, "unloadServo");

        // Inverting direction of motors to go forward for >0 input
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        elevatorPulldownMotorMaster.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorPulldownMotorSlave.setDirection(DcMotorSimple.Direction.REVERSE);

        // Sets servo position on TeleOp init
        unloadServo.setPosition(unloadServoPosition);

        // Changes elevator motors' behavior after setting power to 0.0
        // FLOAT = no resistance
        // BRAKE = resistance
        elevatorPulldownMotorMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // DRIVETRAIN (GAMEPAD1)
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftFrontPower = Range.clip(drive + turn, -1.0, 1.0);
            leftRearPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - turn, -1.0, 1.0);
            rightRearPower = Range.clip(drive - turn, -1.0, 1.0);


            // INTAKE (GAMEPAD1)
            if(gamepad1.right_trigger == 0) {
                isRightTriggerPressed = false;
            } else {
                isRightTriggerPressed = true;
            }
            if(gamepad1.left_trigger == 0) {
                isLeftTriggerPressed = false;
            } else {
                isLeftTriggerPressed = true;
            }

            if(isRightTriggerPressed) {
                intakeMotorPower = gamepad1.right_trigger;
            }
            if(isLeftTriggerPressed) {
                intakeMotorPower = -gamepad1.left_trigger;
            }
            if(!isRightTriggerPressed && !isLeftTriggerPressed) {
                intakeMotorPower = 0;
            }


            // ELEVATOR (GAMEPAD2)
            // pulldown buttons     cross(down)     &   circle(up)
            // slide buttons        square(down)    &   triangle(up)

            // No button pressed => all set to 0, all BRAKE
            if(!(gamepad2.cross && gamepad2.circle && gamepad2.square && gamepad2.triangle)) {
                elevatorPulldownMotorPower = 0;
                elevatorSlideMotorPower = 0;
                elevatorPulldownMotorMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorPulldownMotorSlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // No pulldown button pressed => set slide motor to 0
            if(!(gamepad2.cross && gamepad2.circle)) {
                elevatorPulldownMotorPower = 0;
            }
            // No slide button pressed => set slide to 0
            if(!(gamepad2.square && gamepad2.triangle)) {
                elevatorSlideMotorPower = 0;
            }

            // Any pulldown button pressed => set slide to 0 and FLOAT
            if(gamepad2.cross || gamepad2.circle) {
                elevatorSlideMotorPower = 0;
                elevatorSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            // Any slide button pressed => set pulldown to 0 and FLOAT
            if(gamepad2.square || gamepad2.triangle) {
                elevatorPulldownMotorPower = 0;
                elevatorPulldownMotorMaster.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                elevatorPulldownMotorSlave.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Set power if button is pressed
            if(gamepad2.circle) {
                elevatorPulldownMotorPower = ELEVATOR_PULLDOWN_POWER_UP;
            }
            if(gamepad2.cross) {
                elevatorPulldownMotorPower = -ELEVATOR_PULLDOWN_POWER_DOWN;
            }
            if(gamepad2.triangle) {
                elevatorSlideMotorPower = ELEVATOR_SLIDE_POWER_UP;
            }
            if(gamepad2.square) {
                elevatorSlideMotorPower = -ELEVATOR_SLIDE_POWER_DOWN;
            }


            // CARBON UNLOAD SERVO (GAMEPAD1)
            if(gamepad1.dpad_up) {
                unloadServoPosition = UNLOAD_SERVO_POSITION_UP;
            }
            if(gamepad1.dpad_down) {
                unloadServoPosition = UNLOAD_SERVO_POSITION_DOWN;
            }


            // Send calculated power to motors
            leftDriveFront.setPower(leftFrontPower);
            leftDriveRear.setPower(leftRearPower);
            rightDriveFront.setPower(rightFrontPower);
            rightDriveRear.setPower(rightRearPower);
            elevatorPulldownMotorMaster.setPower(elevatorPulldownMotorPower);
            elevatorPulldownMotorSlave.setPower(elevatorPulldownMotorPower);
            elevatorSlideMotor.setPower(elevatorSlideMotorPower);
            intakeMotor.setPower(intakeMotorPower);
            unloadServo.setPosition(unloadServoPosition);


            // Add data to telemetry (runtime, motors' powers)
            telemetry.addData("Status","Run Time: (%.2f)", runtime.toString());
            telemetry.addData("Motors", "Left  Drive Power (%.2f)", leftFrontPower);
            telemetry.addData("Motors", "Right Drive Power (%.2f)", rightFrontPower);
            telemetry.addData("Motors", "Elevator Pulldown Motor Power (%.2f)", elevatorPulldownMotorPower);
            telemetry.addData("Motors", "Elevator Slide Motor Power (%.2f)", elevatorSlideMotorPower);
            telemetry.addData("Motors", "Intake Motor Power (%.2f)", intakeMotorPower);
            telemetry.addData("Servos", "Unload Servo Position", unloadServoPosition);
            telemetry.update();
        }
    }
}
