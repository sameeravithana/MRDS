package com.umu.samtube405;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.net.HttpURLConnection;
import java.net.URL;
import java.net.URLConnection;
import java.util.Map;

// Jar file for JSON support
import com.fasterxml.jackson.core.*;
import com.fasterxml.jackson.databind.*;
import com.fasterxml.jackson.annotation.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStream;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/**
 * TestRobot interfaces to the (real or virtual) robot over a network connection.
 * It uses Java -> JSON -> HttpRequest -> Network -> DssHost32 -> Lokarria(Robulab) -> Core -> MRDS4
 * 
 * @author ens13sha
 */
public class TestRobot
{
   private String host;                // host and port numbers
   private int port;
   private ObjectMapper mapper;        // maps between JSON and Java structures
   private TestRobot robot;

   private InputStream is = null;
   
   private static JSONObject jObj = null;
   
   private static JSONArray results = null;
   
   private String json = "";
   
   private static double v,w,r,l,min_distance=Double.MAX_VALUE;
   
   private static File f=null;
   
   
   
   /**
    * Create a robot connected to host "host" at port "port"
    * @param host normally http://127.0.0.1
    * @param port normally 50000
    */
   public TestRobot(String host, int port,String fpath)
   {
      this.host = host;
      this.port = port;
      this.robot = this;

      mapper = new ObjectMapper();
      
      f = new File(fpath);     
            
      
   }

   /**
    * This simple main program creates a robot, sets up some speed and turning rate and
    * then displays angle and position for 16 seconds.
    * @param args         Use to give File name
    * @throws Exception   not caught
    */
   public static void main(String[] args) throws Exception
   {     
      System.out.println("Creating Robot");
      TestRobot robot = new TestRobot("http://127.0.0.1", 50000,args[0]);
      
      jObj = robot.getPathData(f.getAbsolutePath());

      System.out.println("Creating response");
      LocalizationResponse lr = new LocalizationResponse();
      
      
      LaserEchoesResponse ler = new LaserEchoesResponse();

      System.out.println("Creating request");
      DifferentialDriveRequest dr = new DifferentialDriveRequest();

       
      int rc;
      
      //double[] position;
      double[] laser;
      double angle;
 
      
      
      results=jObj.getJSONArray("results");

         
         double[] wcs_dest_position = new double[2];
         
         double[] rcs_dest_position = new double[2];         
         
         double [] wcs_cur_position = new double[2];
         
         
         int cur_time,next_time,ttime=0;
         int points_dis=10;
         
         double distance;
         
         long start_time = System.nanoTime();
         
      for (int i = 0; i < results.length(); i+=points_dis)
      {
         System.out.println("---------------------------------------Row: " + i);         

         // ask the robot about its position and angle
         robot.getResponse(lr);
         
         // ask the robot about laser echoes
         //robot.getResponse(ler);

         angle = robot.getBearingAngle(lr);
         System.out.println("Robot Response: bearing = " + angle);

         wcs_cur_position = robot.getPosition(lr);
         System.out.println("Robot Response: position = " + wcs_cur_position[0] + ", " + wcs_cur_position[1]);
         
         //laser = robot.getEchoes(ler);
         
         JSONObject c = results.getJSONObject(i);
         
         cur_time=c.getInt("Timestamp");       
         
         if((i+points_dis)<results.length()){
             JSONObject cnext = results.getJSONObject(i+points_dis);
         
             next_time=cnext.getInt("Timestamp");
             
             ttime=next_time-cur_time;
         }
         
         
         if(c.getInt("Status")>0){
             
             
             JSONObject t=c.getJSONObject("Pose").getJSONObject("Position");
             System.out.println("Path: position = " + t.getDouble("X") + ", " + t.getDouble("Y"));
             
                    
             wcs_dest_position[0]=t.getDouble("X");
             
             wcs_dest_position[1]=t.getDouble("Y");
             
                          
             rcs_dest_position[0]=((wcs_dest_position[0]-wcs_cur_position[0])*Math.cos(angle))+((wcs_dest_position[1]-wcs_cur_position[1])*Math.sin(angle));
             
             rcs_dest_position[1]=((wcs_dest_position[1]-wcs_cur_position[1])-(rcs_dest_position[0]*Math.sin(angle)))/Math.cos(angle);
             
             l=Math.pow(rcs_dest_position[0], 2)+Math.pow(rcs_dest_position[1], 2);
             
             //v=0.75;
             
             distance=Math.sqrt(Math.pow((wcs_dest_position[0]-wcs_cur_position[0]),2)+Math.pow((wcs_dest_position[1]-wcs_cur_position[1]),2));
             
             v=(distance/ttime)*1000;
             
             System.out.println("* PL Speed: "+v);
                        
             
             r=l/(2*rcs_dest_position[1]);
             
             w=v/r;
             
             //w = laserScan(laser, w);
             
             dr.setAngularSpeed(w);
             
             dr.setLinearSpeed(v);
             
             System.out.println("Start to move robot");
             rc = robot.putRequest(dr);
             System.out.println("Response code " + rc);
             
             while(true){
                 // ask the robot about its position and angle
                 robot.getResponse(lr);
                 
                 //robot.getResponse(ler);
                 
                 wcs_cur_position = robot.getPosition(lr);         
         
                          
                 //laser = robot.getEchoes(ler);               
                 
                             
                 
         
                 if(isNearby(wcs_cur_position,wcs_dest_position)){                     
                     break;
                 }
                 
                 
             }
            
             
             
                            
         }
         
         
         
         
      }

      // set up request to stop the robot
      dr.setLinearSpeed(0);
      dr.setAngularSpeed(0);

      long end_time = System.nanoTime();
      double difference = (end_time - start_time)/1e6;
      
      System.out.println("Stop robot");
      rc = robot.putRequest(dr);
      System.out.println("Response code " + rc);
      
      System.out.println("Time Take: "+difference);

   }
   
   private static double laserScan(double[] echoes, double w) {
        double[] e = new double[9];
        int counter = 0;
        for (int i = 90; i < 180; i += 10) {
            for (int j = 0; j < 10; j++) {
                e[counter] += echoes[i + j];
            }
            counter++;
        }
        int min = 0;
        int ind = 0;
        for (int i = 0; i < e.length; i++) {
            if (e[i] < e[min]) {
                min = i;
            }
        }
        ind = min;
        if (min > 5) {
            min = -(10 - min);
        } else {
            min++;
        }
        if (e[ind] < 10 && w * min < 0) {
            double dif = w * (min / 100.0);
            w -= dif;
        }
        return w;
    }

   /*
    * Method to get nearby position
    * @param cur_position current robot position
    * @param dest_position destination position robot has to move
    */
   static boolean isNearby(double[] cur_position,double[] dest_position){
       boolean flag=false;
       double cdx=cur_position[0]-dest_position[0];
       double cdy=cur_position[1]-dest_position[1];
       double cdl=Math.pow(cdx, 2)+Math.pow(cdy, 2);
      
       //if(cdl<=l){
           /*if(cdl<min_distance){
               min_distance=cdl;
           }else{
               flag=true;
               min_distance=Double.MAX_VALUE;
           }*/
       if(cdl<0.1) flag=true; 
       
       System.out.println("FLAG: "+flag+" CDL: "+cdl);
       
       return flag;
   }
   
   /**
    * Extract the robot bearing from the response
    * @param lr
    * @return angle in degrees
    */
   double getBearingAngle(LocalizationResponse lr)
   {
      double e[] = lr.getOrientation();
      //return e[0];
      double angle = 2 * Math.atan2(e[3], e[0]);
      return angle;
      //return angle * 180 / Math.PI;
   }

   /**
    * Extract the position
    * @param lr
    * @return coordinates
    */
   double[] getPosition(LocalizationResponse lr)
   {
      return lr.getPosition();
   }
   
   /**
    * Extract the orientation
    * @param lr
    * @return coordinates
    */
   double[] getOrientation(LocalizationResponse lr)
   {
      return lr.getOrientation();
   }
   
   /**
    * Extract the echoes
    * @param ler
    * @return laser coordinates
    */
   double[] getEchoes(LaserEchoesResponse ler)
   {
       return ler.getEchoes();
   }

   /**
    * Send a request to the robot.
    * @param r request to send
    * @return response code from the connection (the web server)
    * @throws Exception
    */
   public int putRequest(Request r) throws Exception
   {
      URL url = new URL(host + ":" + port + r.getPath());

      HttpURLConnection connection = (HttpURLConnection)url.openConnection();

      connection.setDoOutput(true);

      connection.setRequestMethod("POST");
      connection.setRequestProperty("Content-Type", "application/json");
      connection.setUseCaches (false);

      OutputStreamWriter out = new OutputStreamWriter(
            connection.getOutputStream());

      // construct a JSON string
      String json = mapper.writeValueAsString(r.getData());

      // write it to the web server
      out.write(json);
      out.close();

      // wait for response code
      int rc = connection.getResponseCode();

      return rc;
   }

   /**
    * Get a response from the robot
    * @param r response to fill in
    * @return response same as parameter
    * @throws Exception
    */
   public Response getResponse(Response r) throws Exception
   {
      URL url = new URL(host + ":" + port + r.getPath());
      System.out.println(url);

      // open a connection to the web server and then get the resulting data
      URLConnection connection = url.openConnection();
      BufferedReader in = new BufferedReader(new InputStreamReader(
            connection.getInputStream()));

      // map it to a Java Map
      Map<String, Object> data = mapper.readValue(in, Map.class);
      r.setData(data);

      in.close();

      return r;
   }

  /**
    * Get the data from JSON File
    * @param filepath
    * @return JSON Object
    */ 
  public JSONObject getPathData(String filePath) throws IOException, JSONException{
      try {
            is = new FileInputStream(f.getAbsolutePath());
            
            BufferedReader reader = new BufferedReader(new InputStreamReader(is, "iso-8859-1"));
            
            StringBuilder sb = new StringBuilder();
            String line = null;
            while ((line = reader.readLine()) != null) {
                sb.append(line + "\n");
            }
            
            json = sb.toString();
            //System.out.println(json);
        } catch (Exception e) {
            e.printStackTrace();
        }
      
      
      
      return new JSONObject(json);
  }
  
 
}

