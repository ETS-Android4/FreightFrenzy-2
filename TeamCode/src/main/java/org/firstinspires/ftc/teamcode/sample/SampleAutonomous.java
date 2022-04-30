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

package org.firstinspires.ftc.teamcode.sample;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SampleColorSensor;
import org.firstinspires.ftc.teamcode.hardware.SampleMotor;
import org.firstinspires.ftc.teamcode.hardware.SampleServo;
import org.firstinspires.ftc.teamcode.hardware.SampleTouchSensor;

/**
 * This particular OpMode demonstrates how to access a variety of motors and sensors
 */

@Autonomous(name="SampleAutonomous", group="Sample")
public class SampleAutonomous extends LinearOpMode {

    SampleMotor motor;
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The constructor of the hardware class does all the work here
         */
        motor = new SampleMotor( hardwareMap, true );
        SampleServo servo = new SampleServo( hardwareMap );
        SampleTouchSensor touchSensor = new SampleTouchSensor( hardwareMap );
        SampleColorSensor colorSensor = new SampleColorSensor( hardwareMap, telemetry );

        /* Declare OpMode members. */
        final double SERVO_SPEED     = 0.02 ;                   // sets rate to move servo

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        drive(0.5,12.0,20.0 );

        sleep(4000 );

        drive(0.5,-36.0,60.0 );

        sleep(4000 );
    }

    public void drive( double speed, double inches, double timeout ) {

        // Ensure that the opmode is still active
        if( opModeIsActive() ) {

            // reset the timeout time
            runtime.reset();

            motor.startMoving( speed, inches );

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while( opModeIsActive() &&
                   runtime.seconds() < timeout &&
                   !motor.hasReachedTarget() ) {

                // Display it for the driver.
                telemetry.addData("Target",  "Running to %7d", motor.getEncoderTarget() );
                telemetry.addData( "Position",  "Running at %7d", motor.getEncoderPosition() );
                telemetry.addData( "Status", "Running" );
                telemetry.update();
            }

            motor.stopMoving();

            telemetry.addData("Target",  "Running to %7d", motor.getEncoderTarget() );
            telemetry.addData( "Position",  "Running at %7d", motor.getEncoderPosition() );
            telemetry.addData( "Status", "Stopped" );
            telemetry.update();

            //  sleep(250);   // optional pause after each move
        }
    }
}
