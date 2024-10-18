package util;


public interface DataListener {
    void onDataReceived(String data);
    void onClientConnected();
    void onClientDisconnected();
    void onError(String errorMessage);
}
