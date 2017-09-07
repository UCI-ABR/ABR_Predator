package abr.main;

import android.os.AsyncTask;
import android.util.Log;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;

public class FileClientAsyncTask extends AsyncTask<Void,Void,Object[]> {
	Object[] objects = new Object[3];
    @Override
    protected Object[] doInBackground(Void... params) {
    	int timeout = 10000;
	    int port = 8888;
	    //InetSocketAddress socketAddress  = new InetSocketAddress("192.168.49.1", port);
		InetSocketAddress socketAddress  = new InetSocketAddress("169.234.71.200", port);
	    try {
	      Socket socket = new Socket();
	      objects[0] = socket;
	      socket.bind(null);
	      socket.connect(socketAddress, timeout);
	      DataOutputStream outputStream = new DataOutputStream(socket.getOutputStream());
	      objects[1] = outputStream;
	      DataInputStream inputStream = new DataInputStream(socket.getInputStream());
	      objects[2] = inputStream;
	    } catch (IOException e) {
	      Log.e("hahaha", "IO Exception.", e);
	    }
	    return objects;
    }
}