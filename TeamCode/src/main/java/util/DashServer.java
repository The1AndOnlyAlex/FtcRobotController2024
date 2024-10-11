package util;

import org.json.JSONObject;

import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

public class DashServer {
    private static final String BASE_FOLDER_NAME = "FIRST";

    private static ServerSocket serverSocket;
    private static Socket clientSocket;
    private static OutputStream outputStream;

    static JSONObject jsonData = new JSONObject();

    public static void Init()
    {
        try {
            serverSocket = new ServerSocket(12345);
            serverSocket.setSoTimeout(100);
        } catch (Exception e) {}
    }

    public static boolean Connect()
    {
        try {
            clientSocket = serverSocket.accept();
            outputStream = clientSocket.getOutputStream();
            return true;
        }
        catch (Exception e) { return false;}
    }

    public static boolean AddData(String name, double value)
    {
        try {
          jsonData.put(name, value);
          return  true;
        } catch (Exception e){ return false;}
    }public static boolean AddData(String name, float value)
    {
        try {
            jsonData.put(name, value);
            return true;
        } catch (Exception e){return false;}
    }public static boolean AddData(String name, int value)
    {
        try {
            jsonData.put(name, value);
            return true;
        } catch (Exception e){return false;}
    }

    public static void DashData()
    {
        try {
            if (clientSocket == null || clientSocket.isClosed() )
            {
                //telemetry.addData("Status", "Is a client");
                clientSocket = serverSocket.accept();
//                telemetry.addData("Status", "PC Connected!");
//                telemetry.update();
                outputStream = clientSocket.getOutputStream();
            }

            byte[] sendData = (jsonData.toString() + "\n").getBytes("UTF-8");

            if (outputStream != null) {
                try {
                    outputStream.write(sendData);
                    //outputStream.flush(); // Ensure data is sent immediately
                    //telemetry.addData("Sent Data", jsonData.toString());
                } catch (IOException e) {
//                    telemetry.addData("Error", "Failed to send data: Client may have disconnected. " + e.getMessage());
                    closeClientConnection(); // Handle client disconnection
                }
            }
        } catch (Exception e) {
//            telemetry.addData("Error", "Error in communication: " + e.getMessage());
            closeClientConnection();
        }
        jsonData = new JSONObject();
    }

    public static void Close()
    {
        closeClientConnection();
        closeServerSocket();
    }

    private static void closeClientConnection() {
        try {
            if (clientSocket != null && !clientSocket.isClosed()) {
                clientSocket.close();
                //telemetry.addData("Status", "Client disconnected");
            }
            if (outputStream != null) {
                outputStream.close();
            }
        } catch (IOException e) {
            //telemetry.addData("Error", "Error closing client socket: " + e.getMessage());
        }
    }

    private static void closeServerSocket() {
        try {
            if (serverSocket != null && !serverSocket.isClosed()) {
                serverSocket.close();
                //telemetry.addData("Status", "Server socket closed");
            }
        } catch (IOException e) {
            //telemetry.addData("Error", "Error closing server socket: " + e.getMessage());
        }
    }
}
