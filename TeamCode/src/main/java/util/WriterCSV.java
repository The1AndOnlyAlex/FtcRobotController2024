package util;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class WriterCSV {

    private static final String BASE_FOLDER_NAME = "FIRST";
    static FileWriter fileWriter = null;
    public static void SetWriter(String filename) {
        //String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        String directoryPath = String.format("%s/FIRST/data/mylog.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
        File directory = new File(directoryPath);
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void SetHeader(String headers)
    {
        try
        {
            fileWriter.write(headers+"\n");
        } catch (IOException ex){}
    }

    public static void AddData(String data)
    {
        try
        {
            fileWriter.write(data+"\n");
        } catch (IOException ex){}
    }

    public static void Close()
    {
        try {
            fileWriter.close();
        }catch (IOException ex) {}
    }
}
