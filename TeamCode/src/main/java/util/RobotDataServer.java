package util;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.PrintWriter;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Arrays;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.json.JSONObject;

public class RobotDataServer {
    private ServerSocket serverSocket;
    private Socket clientSocket;
//    private PrintWriter out;
    private OutputStream out;
//    private BufferedReader in;
    private boolean socketActive = false;
    private Thread serverThread;
    private DataListener listener;  // Custom listener interface for data handling


    public RobotDataServer(DataListener listener) {
        this.listener = listener;
    }


    public void startServer(int port) {
        serverThread = new Thread(() -> {
            try {
                serverSocket = new ServerSocket(port);
                listener.onClientConnected();

                // Accept client connection
                while (true) {
                    try {
                        clientSocket = serverSocket.accept(); // Block until a client connects
                        listener.onClientConnected();

                        // Setup I/O streams
//                        out = new PrintWriter(clientSocket.getOutputStream(), true);
                        out = clientSocket.getOutputStream();
//                        in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));

                        socketActive = true; // Indicate that a client is connected

                        // Handle client communication in a separate thread
                        handleClient();

                    } catch (Exception e) {
                        listener.onError("Error accepting client connection: " + e.getMessage());
                    }
                }

            } catch (Exception e) {
                listener.onError("Error starting server: " + e.getMessage());
            }
        });
        serverThread.start();
    }

    private void handleClient() {
//        String inputLine;
//
//        try {
//            // Read input from the client
//            while ((inputLine = in.readLine()) != null) {
//                listener.onDataReceived(inputLine);
//                sendToClient("Received: " + inputLine);
//            }
//        } catch (Exception e) {
//            listener.onError("Client Error: " + e.getMessage());
//        } finally {
//            cleanupClient();
//        }
    }



    private void cleanupClient() {
        try {
            socketActive = false; // Mark the socket as inactive
            if (out != null) {
                out.close();
            }
//            if (in != null) {
//                in.close();
//            }
            if (clientSocket != null && !clientSocket.isClosed()) {
                clientSocket.close();
            }
            listener.onClientDisconnected();
        } catch (Exception e) {
            listener.onError("Error cleaning up client resources: " + e.getMessage());
        }
    }

    public void stopServer() {
        try {
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
            }
            if (serverThread != null && serverThread.isAlive()) {
                serverThread.interrupt();
            }
        } catch (Exception e) {
            listener.onError("Error stopping server: " + e.getMessage());
        }
    }

    private JSONObject jsonData = new JSONObject();
    public boolean AddData(String name, double value)
    {
        try {
          jsonData.put(name, value);
          return  true;
        } catch (Exception e){
            listener.onError("Error AddData: " + name);
            return false;
        }
    }public boolean AddData(String name, float value)
    {
        try {
            jsonData.put(name, value);
            return true;
        } catch (Exception e) {
            listener.onError("Error AddData: " + name);
            return false;
        }
    }public boolean AddData(String name, int value)
    {
        try {
            jsonData.put(name, value);
            return true;
        } catch (Exception e){
            listener.onError("Error AddData: " + name);
            return false;
        }
    }

    byte[] sendData;
    public void DashData() {
        sendData = null;
        try {
            if (clientSocket != null && !clientSocket.isClosed()) {
                sendData = (jsonData.toString() + "\n").getBytes("UTF-8");
                out.write(sendData);
                out.flush(); // Ensure the data is sent immediately
            }
        } catch (IOException e) {
            listener.onError("Error DashData: " + e.getMessage());
            cleanupClient();
        }

        jsonData = new JSONObject();
    }
}
