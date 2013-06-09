import processing.serial.*; //serial reading package
import controlP5.*; //GUI library
 
 // Button variables
 ControlP5 cp5;
 int myColor = color(255);
 int c1,c2;
 float n,n1;
 Boolean dataDisplay = false;
 Boolean dataCapture = false;

 PrintWriter output;
 
 Serial myPort;        // The serial port
 int xPos = 1;         // horizontal position of the graph
 int[] roll = new int[4]; // Roll value
 int[] time = new int[4]; // Time in milliseconds as reference
 float[] calca = new float[4]; // For holding ln(alpha) of peaks over time
 float[] timea = new float[4]; // Corresponding time for those ln(alpha)'s
 int counter = 0; // Counter for counter()
 Boolean signal = false;
 Boolean swtch = false;
 float damp = 0; // Damping Ratio
 
 // Auto-normalizing variables
 float normByte1 = 20; //IR Sensor
 float normByte2 = 0; //Roll
 float[] temp1 = new float[5];
 float[] temp2 = new float[5];
 int axis1yPos = 200;
 int axis2yPos = 500;
 float norm = 0;
 
 void setup () {
   // set the window size:
   size(1000, 800); 
   // set inital background:
   background(0);  
   initialize(axis1yPos,"IR Sensor");
   initialize(axis2yPos,"Roll");
   
   // Place Buttons
   cp5 = new ControlP5(this);
   // Display Data Button
   cp5.addToggle("Display Data",false,200,600,150,19);
   // Capture Data Button
   cp5.addToggle("Capture Data",false,400,600,150,19);
   // Close Program Button
   cp5.addBang("Close Program",600,600,150,19);
   
   // List all the available serial ports
   println(Serial.list());
   
   // Open whatever port is the one you're using.
   myPort = new Serial(this, "/dev/tty.RN42-DECF-SPP", 115200);
  
   // don't generate a serialEvent() unless you get a newline character:
   myPort.bufferUntil('\n');
  
  //Create file to write to
  output = createWriter("positions.txt"); 

 }
 
 
 void draw () {
 // everything happens in the serialEvent()
 serialEvent(myPort);
 }
 
 
 void serialEvent (Serial myPort) {
   
   // get the ASCII string:
   String inString = myPort.readStringUntil('\n');

 if (inString != null) {
   if (inString.contains(",")) {
     String[] inStr = inString.split(",");
     String inString1 = inStr[0];
     print("First Data: ");
     println(inString1);
     String inString2 = inStr[1];
     print("Second Data: ");
     Boolean neg = false;
     if (inString2.contains("-")){ // Check if negative number
       neg = true;
     }
     inString2 = inString2.replaceAll("\\D+",""); // Reduce to numbers only
     if (neg) {
       String minus = "-";
       inString2 = minus.concat(inString2);
     }
     println(inString2);
     roll[0] = roll[1];
     roll[1] = roll[2];
     roll[2] = roll[3];
     roll[3] = Integer.parseInt(inString2); // Store new data value at end of array
     time[0] = time[1];
     time[1] = time[2];
     time[2] = time[3];
     time[3] = currentTimeMillis();
     damping();
     print("Damping Coefficient: ");
     println(damp);
     
     // Display Data if toggle on
     if (dataDisplay) {
       process(inString1, 1);
       process(inString2, 2);
     }
     // Print data if button toggled on
     if (dataCapture) {
     // Print data to output file
     output.print(inString1 + ",");
     output.println(inString2);
     }
   } else {
     process(inString,3);
   }
   // at the edge of the screen, go back to the beginning:
   if (xPos >= width) {
    xPos = 0;
    // set inital background:
    background(0); 
    initialize(axis1yPos,"IR Sensor");
    initialize(axis2yPos,"Roll");
   } else {
   // increment the horizontal position:
     xPos++;
   }
 }




   
 

 }
 
 void initialize (int yPos, String type) {
  smooth();
 // draw the y-axis
  stroke(126);
  line(0, yPos, width, yPos);
  
 // bars every 15 px
  for (int n=0; n<=width; n=n+15) {
     line(n,yPos-5,n,yPos+5);
  }
  
  textSize(15);
  textAlign(CENTER);
  text(type,width/2,yPos-150); // 10 -> 30
  text("Damping Coefficient: ",width*.4,700); //-40 -> 50
  
 }
 
 public float normalize(float temp[], float in, float norm) {
   //Auto-normalize if last five values are around input value
     temp[0] = temp[1];
     temp[1] = temp[2];
     temp[2] = temp[3];
     temp[3] = temp[4];
     temp[4] = (int) in;
     boolean indicator = true;
     for (int i=0; i<temp.length; i++) {
       if (temp[i] != in) {
         indicator = false;
       }
     }
     if (indicator) {
       norm = temp[0];
     }
     return norm;
 }
 
 
 void process(String string, int type) {
   float[] temp;
   int yPos = 0;
   int lowTol = 0;
   int highTol = 0;
   int mapLow = 0;
   // trim off any whitespace:
   string = trim(string);
   // convert to an int and map to the screen height:
   float inByte = float(string); 
   if (inByte != 0) {
     switch (type) {
       case 1:  norm = normByte1;  // IR Sensor
                temp = temp1;
                yPos = axis1yPos;
                lowTol = -10;
                highTol = 10;
                //inByte = random(10,30);
                mapLow = 0;
                break;
       case 2:  norm = normByte2;  // Roll
                temp = temp2;
                yPos = axis2yPos;
                lowTol = -50;
                highTol = 50;
                //inByte = random(-50,50);
                norm = normalize(temp,inByte,norm);
                mapLow = -150;
                break;
       case 3:
                break;
       default: break;
     }
   
   //println(inByte);
   int min = (int) norm + lowTol;
   int max = (int) norm + highTol;
   inByte = map(inByte, min, max, mapLow, 150);  //Change map to appropriate min and max
 
   // draw the line:
   stroke(127,34,255);
   line(xPos, yPos, xPos, yPos - inByte);
   }
 
 
 

    }
    
    void keyPressed() { // Will execute contents when any key is pressed
       
    }
    
    void damping() {
      Boolean haveTime = true;
      for (int i=0; i<3; i++) {
        if ((time[i+1]-time[i]) == 0){
          haveTime = false;
        }
      }
      if (haveTime) { // check if time is filled up first, avoid div by zero
        float drdt1 = (float) ( roll[3] - roll[2] )/( time[3] - time[2] );
        float drdt2 = (float) ( roll[2] - roll[1] )/( time[2] - time[1] );
        float drdt3 = (float) ( roll[1] - roll[0] )/( time[1] - time[0] );
        println(drdt1);
        if (drdt1 < 0 && drdt2 >= 0 && drdt3 >= 0) {
          signal = true;
        }
        if (signal) {
          count();
        } else {
          for (int i=0; i<calca.length; i++) {
            calca[i] = 0;
          }
          for (int i=0; i<timea.length; i++) {
            calca[i] = 0;
          }
          counter = 0;
        }
      }
    }
    void count() {
      if (roll[3]>0 && roll[3] < roll[2] && swtch) {
        swtch = false;
        calca[0] = calca[1];
        calca[1] = calca[2];
        calca[2] = calca[3];
        calca[3] = (float) Math.log(roll[2]);
        timea[0] = timea[1];
        timea[1] = timea[2];
        timea[2] = timea[3];
        timea[3] = currentTimeMillis();
        counter = counter + 1;
        println(calca[3]);
      } else if ( roll[3]<0 ) {
        swtch = true;
      }
      if (counter==5) {
        float[] as = new float[4];
        for (int i=0; i<3; i++) {
          as[0] = as[1];
          as[1] = as[2];
          as[2] = (calca[i+1] - calca[i])/( (timea[i+1] - timea[i])*.01 );
        }
        float a = (as[3] + as[2] + as[1] + as[0])/4; // Averaging alphas
        damp = -2*a;
        String strDamp = Float.toString(damp);
        text(strDamp,width*.5+50,700);
        signal = false;
      }
    }
    
    public static int currentTimeMillis() {
        return (int) (System.currentTimeMillis() % Integer.MAX_VALUE);
    }
    void controlEvent(ControlEvent theEvent) {
      if(theEvent.isController()) {
        if(theEvent.controller().name()=="Capture Data") {
          if(theEvent.controller().value()==1) {
            dataCapture = true;
          } else {
            dataCapture = false;
          }
        }
        if(theEvent.controller().name()=="Display Data") {
          if(theEvent.controller().value()==1) {
            dataDisplay = true;
          } else {
            dataDisplay = false;
          }
        }
        if(theEvent.controller().name()=="Close Program") {
          output.flush(); 
          output.close();
          exit();
        }
      }
    }
