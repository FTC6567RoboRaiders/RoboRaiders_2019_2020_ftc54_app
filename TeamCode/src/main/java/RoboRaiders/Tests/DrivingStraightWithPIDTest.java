package RoboRaiders.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import RoboRaiders.AutonomousMethods.RoboRaidersPID;
import RoboRaiders.AutonomousMethods.RRAutonomousMethods;
import RoboRaiders.Robot.Robot;
import RoboRaiders.Robot.PidUdpReceiver;
import RoboRaiders.Robot.RobotTelemetryDisplay;

@Autonomous

public class DrivingStraightWithPIDTest extends RRAutonomousMethods {
    public Robot robot = new Robot();


    private PidUdpReceiver pidUdpReceiver;
    private RoboRaidersPID rrPID;
    private RobotTelemetryDisplay rtd;
    private double kP, kI, kD, direction, degrees;


    //----------------------------------------------------------------------------------------------
    // Main logic
    //----------------------------------------------------------------------------------------------

    @Override
    public void runOpMode() throws InterruptedException {

        // Create new instance of robot telemetry display
        //rtd = new RobotTelemetryDisplay(this,"Nostromo");

        // Create new instance of receiver
        pidUdpReceiver = new PidUdpReceiver();

        //Create new rrPID
        rrPID = new RoboRaidersPID(0,0,0);

        // Start listening
        pidUdpReceiver.beginListening();

        // initialize the robot
        robot.initialize(hardwareMap);

        // set the transmission interval to 50 milliseconds
        telemetry.setMsTransmissionInterval(50);


        // Wait for start to be pushed
        waitForStart();

        if (opModeIsActive()) {

            updatePIDCoefficients();

            rrPID.setCoeffecients(kP,kI,kD);

            telemetry.addData("kP",kP);
            telemetry.addData("kI",kI);
            telemetry.addData("kD",kD);
            telemetry.addData("distance",degrees);
            telemetry.addData("direction",direction);
            telemetry.update();

            encoderDrivePIDTuner(rrPID, robot, degrees, 1.0);
        }

        pidUdpReceiver.shutdown();
    }


    public void updatePIDCoefficients() {

        kP = pidUdpReceiver.getP();
        kI = pidUdpReceiver.getI();
        kD = pidUdpReceiver.getD();
        direction = pidUdpReceiver.getDirection();
        degrees = pidUdpReceiver.getDegrees();

        //rtd.displayRobotTelemetry("kP",String.valueOf(kP));
    }
}


