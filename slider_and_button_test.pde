import processing.serial.*;    
import controlP5.*;    //import controlp5 library
Serial port;
ControlP5 control;    //create ControlP5 object
PFont buttonfont;    //create a font for button

void setup()
{
  size (600,600);    //window width and height
  buttonfont = createFont("Arial",15);    //define the font for button
  port = new Serial(this,"COM10",9600);    //Communicate with COM10 with baud rate 9600
  
  control = new ControlP5(this);    //control object 
  
  control.addButton("Save")
    .setPosition(50,500) .setSize(80,80) .setFont(buttonfont);
  
  control.addButton("Play")
    .setPosition(150,500) .setSize(80,80) .setFont(buttonfont);
  
  control.addButton("Reset")
    .setPosition(250,500) .setSize(80,80) .setFont(buttonfont);
    
  control.addButton("Home")
    .setPosition(350,500) .setSize(80,80) .setFont(buttonfont);
    
  control.addSlider("X_angle")
    .setPosition (50,100) .setSize(200,50) .setRange(0,90) .setValue(45)
    .setSliderMode(Slider.FLEXIBLE) .setId(1);
    
  control.addSlider("Y_angle")
    .setPosition (50,200) .setSize(200,50) .setRange(0,90) .setValue(90)
    .setSliderMode(Slider.FLEXIBLE) .setId(2);
  
  control.addSlider("Z_angle")
    .setPosition (50,300) .setSize(200,50) .setRange(0,180) .setValue(90)
    .setSliderMode(Slider.FLEXIBLE) .setId(3);
    
  control.addSlider("Servo")
    .setPosition (50,400) .setSize(200,50) .setRange(0,70) .setValue(70)
    .setSliderMode(Slider.FLEXIBLE) .setId(4);
  
}

void draw()
{
   background(0,0,0);    //background color r,g,b 230,253,255
   fill (255,255,255);    //text color r,g,b
   text ("3R Robot Forward Kinematics",225,30);    //("text",x-coor,y-coor)
}


//void Home(){port.write('h');}
void Save(){port.write('s');}
void Play() {port.write('p');}
void Reset() {port.write('r');}

void X_angle(int X_angle){port.write(1000+X_angle);}
void Y_angle(int Y_angle){port.write(2000+Y_angle);}
void Z_angle(int Z_angle){port.write(3000+Z_angle);}

/*void slider(float theColor) {
  myColor = color(theColor);
  println("a slider event. setting background to "+theColor);*/