// package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.json.JSONObject;

import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.charset.StandardCharsets;
import java.util.Random;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;

@TeleOp(name="SampleDataToDash", group="Linear Opmode")
public class SampleDataToDash extends LinearOpMode {

    private ServerSocket serverSocket;
    private Socket clientSocket;
//    private PrintWriter out;
    private OutputStream outputStream;

//    private DatagramSocket udpSocket;
//    private InetAddress broadcastAddress;
//    private int broadcastPort = 12345; // Default port for broadcasting (can be configured)

//    private final int bufferSize = 8192; // Increase buffer size to handle more data
    @Override
    public void runOpMode() {
        try {
            // Start the server socket on a separate port (non-blocking)
            serverSocket = new ServerSocket(12345);
            serverSocket.setSoTimeout(100); // Timeout to avoid blocking the robot operation

            // Initialize the UDP socket
//            udpSocket = new DatagramSocket();
//            udpSocket.setBroadcast(true);  // Enable broadcasting
//            udpSocket.setSendBufferSize(bufferSize);  // Increase the send buffer size
//
//            // Configure the broadcast address (use 255.255.255.255 for global broadcast, or a specific subnet)
//            broadcastAddress = InetAddress.getByName("192.168.43.255"); // Can also use a subnet broadcast address, e.g., "192.168.1.255"


            telemetry.addData("Status", "Waiting for PC to connect...");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Could not start server: " + e.getMessage());
            telemetry.update();
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        int startFrame = 0;

        // Main loop that runs during the op mode
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            
            int deltaFrame = startFrame % 80;
            double deltaDeg = 2 * 3.1415 * deltaFrame / 80.0f;
            double deltaCommand = (Math.cos(deltaDeg) * 0.5f) - 0.5f;

            // Simulate telemetry data (replace with actual robot data)
            double motorPower = 1.5 + deltaCommand;//new Random().nextInt(5); // Example motor power
            double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage(); // Example sensor data
            long currentTime = System.currentTimeMillis();

            // Try to accept a connection (non-blocking)
            try {
                if (clientSocket == null || clientSocket.isClosed() )// || out == null)
                {
                    telemetry.addData("Status", "Is a client");
                    clientSocket = serverSocket.accept(); // Accept a new connection if available
                    telemetry.addData("Status", "PC Connected!");
                    telemetry.update();
                    // Set up the output stream to send data
//                    out = new PrintWriter(new OutputStreamWriter(clientSocket.getOutputStream(),"UTF-8"), true);
                    // Set up the output stream to send data to the client
                    outputStream = clientSocket.getOutputStream();
//                    out = new PrintWriter(outputStream, true);
                }

                // Check if the client is still connected
//                if (!isClientConnected()) {
//                    //closeClientConnection(); // Handle client disconnection
//                    telemetry.addData("Error", "Closed Client.");
//                }
//                else

                    // Create JSON object with telemetry data
                    JSONObject jsonData = new JSONObject();
                    jsonData.put("time", startFrame);
                    jsonData.put("motorPower", motorPower);
                    jsonData.put("batteryVoltage", batteryVoltage);
                jsonData.put("left_stick_y", drive);
                jsonData.put("right_stick_x", turn);

                    // Convert the JSON data to bytes
                    byte[] sendData = (jsonData.toString() + "\n").getBytes("UTF-8");

                    // Create a UDP packet and broadcast it
//                    DatagramPacket packet = new DatagramPacket(sendData, sendData.length, broadcastAddress, broadcastPort);
//                    udpSocket.send(packet);
//                    telemetry.addData("Sent Data", jsonData.toString());
                    // Send the data to the client (if connected)
//                    if (out != null) {
//                        out.println(jsonData.toString());
////                        out.flush();  // Ensure the data is sent
//                        telemetry.addData("Sent Data", jsonData.toString());
//                    }
//                    else {
//                        telemetry.addData("Error", "PrintWriter error.");
//                    }

                    // Check if the client has disconnected (non-blocking way)
//                    if (clientSocket.isClosed()) {
//                        telemetry.addData("Status", "Client disconnected");
//                        closeClientConnection(); // Handle client disconnection
//                    }

                    if (outputStream != null) {
                        try {
                            outputStream.write(sendData);
                            //outputStream.flush(); // Ensure data is sent immediately
                            telemetry.addData("Sent Data", jsonData.toString());
                        } catch (IOException e) {
                            telemetry.addData("Error", "Failed to send data: Client may have disconnected. " + e.getMessage());
                            closeClientConnection(); // Handle client disconnection
                        }
                    }


            } catch (Exception e) {
                telemetry.addData("Error", "Error in communication: " + e.getMessage());
                //telemetry.update();
                closeClientConnection();
            }

            startFrame++;

            // Update telemetry on the driver station
            telemetry.addData("Current Counter", currentTime);
            telemetry.addData("Current Frame", startFrame);
            telemetry.addData("Motor Power", motorPower);
            telemetry.addData("Battery Voltage", batteryVoltage);
            telemetry.update();

            // Sleep for a short period (e.g., 1 second) to control the rate of data sending
            sleep(20);
        }

        // Close the UDP socket when the op mode stops
//        if (udpSocket != null && !udpSocket.isClosed()) {
//            udpSocket.close();
//            telemetry.addData("Status", "UDP socket closed");
//            telemetry.update();
//        }

        // Close the server and client sockets when the op mode stops
        closeClientConnection();
        closeServerSocket();

    }



    // Method to check if the client is connected
    private boolean isClientConnected() {
        return clientSocket != null && !clientSocket.isClosed();
    }

    // Method to close the client connection
    private void closeClientConnection() {
        try {
            if (clientSocket != null && !clientSocket.isClosed()) {
                clientSocket.close();
                telemetry.addData("Status", "Client disconnected");
            }
//            if (out != null) {
//                out.close();
//            }
            if (outputStream != null) {
                outputStream.close();
            }
        } catch (IOException e) {
            telemetry.addData("Error", "Error closing client socket: " + e.getMessage());
        }
        //telemetry.update();
    }

    // Method to close the server socket
    private void closeServerSocket() {
        try {
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
                telemetry.addData("Status", "Server socket closed");
            }
        } catch (IOException e) {
            telemetry.addData("Error", "Error closing server socket: " + e.getMessage());
        }
        //telemetry.update();
    }
}
