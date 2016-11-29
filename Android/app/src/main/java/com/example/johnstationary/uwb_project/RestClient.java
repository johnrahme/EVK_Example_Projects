package com.example.johnstationary.uwb_project;
import java.io.*;
import okhttp3.*;
import android.widget.*;

public class RestClient
{
    protected MainActivity context;
    String message = "";
    OkHttpClient client = new OkHttpClient();
    public RestClient(MainActivity _context){
        context = _context;
    }
    public void run() throws Exception {
        //final TextView text = t;
        Request request = new Request.Builder()
                .url("http://192.168.1.184/service/getPositions/1")
                .build();

        client.newCall(request).enqueue(new Callback() {
            @Override
            public void onFailure(Call call, IOException e) {
                e.printStackTrace();
            }

            @Override
            public void onResponse(Call call, Response response) throws IOException {
                if (!response.isSuccessful()) throw new IOException("Unexpected code " + response);

                Headers responseHeaders = response.headers();
                for (int i = 0, size = responseHeaders.size(); i < size; i++) {
                    System.out.println(responseHeaders.name(i) + ": " + responseHeaders.value(i));
                }
                message = response.body().string();
                System.out.println(message+" yeeey");
                context.runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        try{
                            context.helloWorldView.setText(message);
                            System.out.println("Funkade");
                        }
                        catch (Exception e){
                            System.out.println("Inteeee funk" + e.toString());
                        }
                    }
                });


            }
        });
    }
}