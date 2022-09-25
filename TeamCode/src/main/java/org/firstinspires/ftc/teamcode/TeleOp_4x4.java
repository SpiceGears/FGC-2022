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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp 4x4", group="Linear Opmode")
// @Disabled
public class TeleOp_4x4 extends LinearOpMode {

    // Declare OpMode members
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDriveFront;     // port 0
    DcMotor leftDriveRear;      // port 1
    DcMotor rightDriveFront;    // port 2
    DcMotor rightDriveRear;     // port 3

    DcMotor elevatorMotor1;     // port 0 expansion
    DcMotor elevatorMotor2;     // port 1 expansion

    Servo unloadServo;
    Servo elevatorServo;

    // Constants
    public double ELEVATOR_MOTOR1_POWER = 0.2;
    public double ELEVATOR_MOTOR2_POWER = 0.2;

    @Override
    public void runOpMode() {

        // Setup a variable for each motor to save power level for telemetry and setting motor power
        double leftPower;                           // left drive motors power
        double rightPower;                          // right drive motors power
        double elevator1Power = 0;                  // elevator1 motor power
        double elevator2Power = 0;                  // elevator2 motor power
        double unloadServoPosition = 0;             // unloading carbon servo
        double elevatorServoPosition = 0;           // locking robot on bar servo

        leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");

        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");

        unloadServo = hardwareMap.get(Servo.class, "unloadServo");
        elevatorServo = hardwareMap.get(Servo.class, "elevatorServo");

        // Inverting direction of drivetrain motors to go forward for >0 input
        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveRear.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.FORWARD);
        rightDriveRear.setDirection(DcMotor.Direction.FORWARD);

        // Changes motor's behavior after setting power to 0.0
        // FLOAT = no resistance
        // BRAKE = resistance
        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets servos positions on TeleOp init
        unloadServo.setPosition(unloadServoPosition);
        elevatorServo.setPosition(elevatorServoPosition);

        telemetry.addData("Status", "Initialized!");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // CARBON UNLOAD SERVO
            if(gamepad1.dpad_down) {
                unloadServoPosition = 0;
            }
            if(gamepad1.dpad_up) {
                unloadServoPosition = 1;
            }

            // ELEVATOR SERVO
            if(gamepad1.dpad_left) {
                elevatorServoPosition = 0;
            }
            if(gamepad1.dpad_right) {
                elevatorServoPosition = 1;
            }

            // DRIVETRAIN
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // ELEVATOR
            // group 1 buttons - cross & circle
            // group 2 buttons - square & triangle

            // No button pressed => all 0, all BRAKE
            if(!(gamepad1.cross && gamepad1.circle && gamepad1.square && gamepad1.triangle)) {
                elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevator1Power = 0;
                elevator2Power = 0;
            }

            // No group1 button pressed => set motor1 to 0
            if(!(gamepad1.cross && gamepad1.circle)) {
                elevator1Power = 0;
            }
            // No group2 button pressed => set motor2 to 0
            if(!(gamepad1.square && gamepad1.triangle)) {
                elevator2Power = 0;
            }

            // Any group1 button pressed => set motor2 to 0 and FLOAT
            if(gamepad1.cross || gamepad1.circle) {
                elevator2Power = 0;
                elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            // Any group2 button pressed => set motor1 to 0 and FLOAT
            if(gamepad1.square || gamepad1.triangle) {
                elevator1Power = 0;
                elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }

            // Set power if button is pressed
            if(gamepad1.cross) {
                elevator1Power = ELEVATOR_MOTOR1_POWER;
            }
            if(gamepad1.circle) {
                elevator1Power = -1*ELEVATOR_MOTOR1_POWER;
            }
            if(gamepad1.square) {
                elevator2Power = ELEVATOR_MOTOR2_POWER;
            }
            if(gamepad1.triangle) {
                elevator2Power = -1*ELEVATOR_MOTOR2_POWER;
            }

            // Makes motors have no resistance after clicking right bumper
            if(gamepad1.right_bumper) {
                elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            } else {
                elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            // Send calculated power to motors
            leftDriveFront.setPower(leftPower);
            leftDriveRear.setPower(leftPower);
            rightDriveFront.setPower(rightPower);
            rightDriveRear.setPower(rightPower);

            elevatorMotor1.setPower(elevator1Power);
            elevatorMotor2.setPower(elevator2Power);

            unloadServo.setPosition(unloadServoPosition);
            elevatorServo.setPosition(elevatorServoPosition);

            // Add data to telemetry (runtime, motors powers)
            telemetry.addData("Status","Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors", "Elevator Motor 1 (%.2f), Elevator Motor 2 (%.2f)", elevator1Power, elevator2Power);
            telemetry.addData("Servos", "Unload Servo Position: " + unloadServoPosition);
            telemetry.addData("Servos", "Elevator Servo Position: " + elevatorServoPosition);
            telemetry.update();
        }
    }
}