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

@TeleOp(name="TeleOp 4x4", group="Linear Opmode")
// @Disabled
public class TeleOp_4x4 extends LinearOpMode {

    // Declare OpMode members
    // Ports in comments are ports for default config
    ElapsedTime runtime = new ElapsedTime();
    DcMotor leftDriveFront;     // motor port 0 (control hub)
    DcMotor leftDriveRear;      // motor port 1 (control hub)
    DcMotor rightDriveFront;    // motor port 2 (control hub)
    DcMotor rightDriveRear;     // motor port 3 (control hub)
    DcMotor elevatorMotor1;     // motor port 0 (expansion hub)
    DcMotor elevatorMotor2;     // motor port 1 (expansion hub)
    DcMotor intakeMotor;        // motor port 2 (expansion hub)
    Servo unloadServo;          // servo port 0 (control hub)
    Servo elevatorServo;        // servo port 1 (control hub)

    // Constants
    public double ELEVATOR_MOTOR1_POWER_UP = 1;                           // 0 - 1
    public double ELEVATOR_MOTOR1_POWER_DOWN = 1;                         // 0 - 1
    public double ELEVATOR_MOTOR2_POWER_UP = 1;                             // 0 - 1
    public double ELEVATOR_MOTOR2_POWER_DOWN = 0.25;                        // 0 - 1
    public double DRIVETRAIN_POWER_DEFAULT_MULTIPLIER = 0.75;                // 0 - 1, 1=full speed, no change
    public double DRIVETRAIN_POWER_BOOST_MULTIPLIER = 1;                    // 0 - 1, 1=full speed

    @Override
    public void runOpMode() {

        // Variable for each motor to save power level for telemetry and setting motor power
        double leftPower;                           // left drive motors power
        double rightPower;                          // right drive motors power
        double elevator1Power = 0;                  // elevator1 motor power
        double elevator2Power = 0;                  // elevator2 motor power
        double intakeMotorPower = 0;                // intake motor power
        double unloadServoPosition = 0;             // unloading carbon servo position (defined on init)
        double elevatorServoPosition = 0.05;         // elevator servo position (defined on init)
        boolean isDriveTrainBoosted;                // false = default modifier, true = boost modifier

        // Map devices to objects
        leftDriveFront  = hardwareMap.get(DcMotor.class, "leftDriveFront");
        leftDriveRear  = hardwareMap.get(DcMotor.class, "leftDriveRear");
        rightDriveFront = hardwareMap.get(DcMotor.class, "rightDriveFront");
        rightDriveRear = hardwareMap.get(DcMotor.class, "rightDriveRear");
        elevatorMotor1 = hardwareMap.get(DcMotor.class, "elevatorMotor1");
        elevatorMotor2 = hardwareMap.get(DcMotor.class, "elevatorMotor2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        unloadServo = hardwareMap.get(Servo.class, "unloadServo");
        elevatorServo = hardwareMap.get(Servo.class, "elevatorServo");

        // Inverting direction of motors to go forward for >0 input
        leftDriveFront.setDirection(DcMotor.Direction.FORWARD);
        leftDriveRear.setDirection(DcMotor.Direction.FORWARD);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveRear.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Changes motor's behavior after setting power to 0.0
        // FLOAT = no resistance
        // BRAKE = resistance
        elevatorMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Sets servos positions on TeleOp init
        // unloadServo.setPosition(unloadServoPosition);
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
                unloadServoPosition = 0;          // not final
            }
            if(gamepad1.dpad_up) {
                unloadServoPosition = 0.45;        // not final
            }

            // ELEVATOR SERVO
            if(gamepad1.dpad_left) {
                elevatorServoPosition = 0.15;
            }
            if(gamepad1.dpad_right) {
                elevatorServoPosition = 0.3;
            }

            // INTAKE
            intakeMotorPower = gamepad1.right_trigger;

            // DRIVETRAIN
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Clicking left analog button makes the bot go faster
            if(gamepad1.left_stick_button) {
                isDriveTrainBoosted = true;
                leftPower *= DRIVETRAIN_POWER_BOOST_MULTIPLIER;
                rightPower *= DRIVETRAIN_POWER_BOOST_MULTIPLIER;
            } else {
                isDriveTrainBoosted = false;
                leftPower *= DRIVETRAIN_POWER_DEFAULT_MULTIPLIER;
                rightPower *= DRIVETRAIN_POWER_DEFAULT_MULTIPLIER;
            }

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
                elevator1Power = -ELEVATOR_MOTOR1_POWER_DOWN;
            }
            if(gamepad1.circle) {
                elevator1Power = ELEVATOR_MOTOR1_POWER_UP;
            }
            if(gamepad1.square) {
                elevator2Power = -ELEVATOR_MOTOR2_POWER_DOWN;
            }
            if(gamepad1.triangle) {
                elevator2Power = ELEVATOR_MOTOR2_POWER_UP;
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
            intakeMotor.setPower(intakeMotorPower);
            unloadServo.setPosition(unloadServoPosition);
            elevatorServo.setPosition(elevatorServoPosition);

            // Add data to telemetry (runtime, motors powers)
            telemetry.addData("Status","Run Time: " + runtime.toString());
            telemetry.addData("DriveTrain Boost", isDriveTrainBoosted);
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motors", "Elevator Motor 1 (%.2f)", elevator1Power, elevatorMotor1.getZeroPowerBehavior());
            telemetry.addData("Motors", "Elevator Motor 2 (%.2f)", elevator2Power, elevatorMotor2.getZeroPowerBehavior());
            telemetry.addData("Motors", "Intake Motor: " + intakeMotorPower);
            telemetry.addData("Servos", "Unload Servo Position: " + unloadServoPosition);
            telemetry.addData("Servos", "Elevator Servo Position: " + elevatorServoPosition);
            telemetry.update();
        }
    }
}
