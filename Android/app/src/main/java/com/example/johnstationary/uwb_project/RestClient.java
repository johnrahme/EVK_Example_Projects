package com.example.johnstationary.uwb_project;
import java.net.*;
import java.io.*;
import okhttp3.*;
public class RestClient
{
    public URL url;
    private HttpURLConnection con;
    private BufferedReader reader;
    OkHttpClient client = new OkHttpClient();
    public String requestContent(String stringUrl) {
        StringBuilder sb = new StringBuilder();
        String line;
        try {
            url = new URL(stringUrl);
            con = (HttpURLConnection) url.openConnection();
            reader = new BufferedReader(new InputStreamReader(con.getInputStream()));
            while((line = reader.readLine()) != null){
                sb.append(line+"\n");
            }
            reader.close();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }


        return line;

    }
    public void run() throws Exception {
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
                
                System.out.println(response.body().string());
            }
        });
    }
}