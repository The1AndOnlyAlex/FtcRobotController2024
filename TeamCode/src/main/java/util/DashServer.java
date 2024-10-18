package util;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    static Telemetry telemetry;
    public static void Init(Telemetry telemetry1)
    {
        try {
            // serverSocket = new ServerSocket(12345);
            // serverSocket.setSoTimeout(100000);
            telemetry = telemetry1;
        } catch (Exception e) {}
    }

    public static boolean Connect()
    {
        try {
            // clientSocket = serverSocket.accept();
            // clientSocket.setKeepAlive(true);
            // outputStream = clientSocket.getOutputStream();
            return true;
        }
        catch (Exception e) { return false;}
    }

    public static boolean AddData(String name, double value)
    {
        try {
          //jsonData.put(name, value);
          return  true;
        } catch (Exception e){ telemetry.addData("Error2", "Error in communication: ");telemetry.update();
            return false;}
    }public static boolean AddData(String name, float value)
    {
        try {
            //jsonData.put(name, value);
            return true;
        } catch (Exception e){
            telemetry.addData("Error2", "Error in communication: ");telemetry.update();return false;}
    }public static boolean AddData(String name, int value)
    {
        try {
            //jsonData.put(name, value);
            return true;
        } catch (Exception e){
            telemetry.addData("Error2", "Error in communication: ");telemetry.update();return false;}
    }

    static byte[] sendData;
    public static void DashData()
    {
        try {
            if (clientSocket == null  )
            {
                //telemetry.addData("Status", "Is a client");
                clientSocket = serverSocket.accept();
//                telemetry.addData("Status", "PC Connected!");
//                telemetry.update();
                outputStream = clientSocket.getOutputStream();
            }
        }
        catch (Exception e) {
            telemetry.addData("Error2", "Error in communication: ");
            closeClientConnection();
        }
        try {
            if ( clientSocket.isClosed() )
            {
                //telemetry.addData("Status", "Is a client");
                clientSocket = serverSocket.accept();
//                telemetry.addData("Status", "PC Connected!");
//                telemetry.update();
                outputStream = clientSocket.getOutputStream();
            }
        }
        catch (Exception e) {
            telemetry.addData("Error2", "Error in communication: ");
            closeClientConnection();
        }


        sendData = null;
        try{
            sendData = (jsonData.toString() + "\n").getBytes("UTF-8");
        }catch (IOException e) {sendData = null;}
        if(sendData == null) {
            telemetry.addData("ErrorSendData", "Error in communication: ");
            telemetry.update();
        }


        if (outputStream != null ) {

                if(sendData != null)
                try {
                        outputStream.write(sendData);
                    //outputStream.flush(); // Ensure data is sent immediately
                    //telemetry.addData("Sent Data", jsonData.toString());
                } catch (IOException e) {

                    try {
                        outputStream = clientSocket.getOutputStream();
                        outputStream.write(sendData);
                    }
                    catch (IOException e1) {
                        telemetry.addData("Error1", "Failed to send data"+e1);
                        closeClientConnection(); // Handle client disconnection}
                    }

                }
            }
        else {
            try {
                outputStream = clientSocket.getOutputStream();
                if(sendData != null)
                    outputStream.write(sendData);
            }
            catch (IOException e1) {
                telemetry.addData("Error1", "Failed to send data");
                closeClientConnection(); // Handle client disconnection}
            }
        }
        jsonData = new JSONObject();
    }

    public static void Close()
    {
        try {
            outputStream.flush();
        }catch(Exception e) {}
        closeClientConnection();
        closeServerSocket();
    }

    private static void closeClientConnection() {
        try {
            if (clientSocket != null && !clientSocket.isClosed()) {
                clientSocket.close();
                clientSocket = null;
            }
            if (outputStream != null) {
                outputStream.close();
                outputStream = null;
            }
            telemetry.addData("Status", "Client disconnected");
        } catch (IOException e) {
            telemetry.addData("Error3", "Error closing client socket: ");
        }
        telemetry.update();
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
