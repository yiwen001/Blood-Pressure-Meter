#include <mbed.h>
// -----------------------------------------------------------------
// ECE-6483 Final Project                                          
//  
// Description:                                                     
// SPI PINS:  
// MOSI   PE_6
// MISO   PE_5
// SCLK   PE_2
// CS     PE_4                                                                 
//
// author: Ziyu.Lu<zl3392@nyu.com> Yiwen.Gao<yg2412@nyu.edu>                                                                    
//
// Usage:
// 1.press the user button when the screen shows "Press button to start/restart"
//
// 2.inflate the cuff to 150 mmHg when the screen shows "Inflate to 150mmHg"                                                      
//
// 3.press the user button again when the screen shows "Press to deflate".
// Make sure the pressure is above 150 mmHg when pressing the button
//
// 4.deflate the cuff to 30 mmHg when the screen shows "Deflate to 30 mmHg"
//
// 5.All the results will shown on the screen after the deflation,  
// now you can press the user button to restart the process.
// -----------------------------------------------------------------



//--------------------------------------------------------------------------------
// SPI
//--------------------------------------------------------------------------------
SPI spi(PE_6, PE_5, PE_2);  // mosi, miso, sclk
DigitalOut cs(PE_4);        // chip select  
//
#define CMD_PRECEDE1 0xAA
#define CMD_PRECEDE2 0x00
#define CMD_PRECEDE3 0x00

#define CMD_NOP 0xF0

#define READY_FLAG 1

uint8_t write_buf[5]; //the writing buffer of spi message
uint8_t read_buf[5];//the reading buffer of spi message
uint8_t status;//the returned status of pressure output


EventFlags flags;
//The spi.transfer() function requires that the callback
void cb(int event){
  flags.set(READY_FLAG);
  //deselect the sensor
  cs=1;
}



//--------------------------------------------------------------------------------
// LCD
//--------------------------------------------------------------------------------
//this file has all the functions for interacting
//with the screen
#include "drivers/LCD_DISCO_F429ZI.h"
#define BACKGROUND 1
#define FOREGROUND 0
#define GRAPH_PADDING 5

LCD_DISCO_F429ZI lcd;

//buffer for holding displayed text strings
char display_buf[10][60];
uint32_t graph_width=lcd.GetXSize()-2*GRAPH_PADDING;
uint32_t graph_height=graph_width;

//sets the background layer 
//to be visible, transparent, and
//resets its colors to all black
void setup_background_layer(){
  lcd.SelectLayer(BACKGROUND);
  lcd.Clear(LCD_COLOR_BLACK);
  lcd.SetBackColor(LCD_COLOR_BLACK);
  lcd.SetTextColor(LCD_COLOR_GREEN);
  lcd.SetLayerVisible(BACKGROUND,ENABLE);
  lcd.SetTransparency(BACKGROUND,0x7Fu);
}

//resets the foreground layer to
//all black
void setup_foreground_layer(){
    lcd.SelectLayer(FOREGROUND);
    lcd.Clear(LCD_COLOR_BLACK);
    lcd.SetBackColor(LCD_COLOR_BLACK);
    lcd.SetTextColor(LCD_COLOR_LIGHTGREEN);
}

//draws a rectangle with horizontal tick marks
//on the background layer. The spacing between tick
//marks in pixels is taken as a parameter
void draw_graph_window(uint32_t horiz_tick_spacing){
  lcd.SelectLayer(BACKGROUND);  
  lcd.DrawRect(GRAPH_PADDING,GRAPH_PADDING,graph_width,graph_width);
  //draw the x-axis tick marks
  for (uint32_t i = 0 ; i < graph_width;i+=horiz_tick_spacing){
    lcd.DrawVLine(GRAPH_PADDING+i,graph_height,GRAPH_PADDING);
  }
}

//maps inputY in the range minVal to maxVal, to a y-axis value pixel in the range
//minPixelY to MaxPixelY
uint16_t mapPixelY(float inputY,float minVal, float maxVal, int32_t minPixelY, int32_t maxPixelY){
  const float mapped_pixel_y=(float)maxPixelY-(inputY)/(maxVal-minVal)*((float)maxPixelY-(float)minPixelY);
  return mapped_pixel_y;
}

#define BUFFER_SIZE 230
//stack used by the draw_thread
unsigned char draw_thread_stack[4096];
Thread draw_thread(osPriorityBelowNormal1,4096,draw_thread_stack);
//circular buffer is used like a queue. The main thread pushes
//new data into the buffer, and the draw thread pops them out
//and updates the graph
CircularBuffer<float, BUFFER_SIZE> new_values;

//semaphore used to protect the new_values buffer
Semaphore new_values_semaphore(0,BUFFER_SIZE);


uint32_t graph_pixels[BUFFER_SIZE];
uint32_t next_write_index=0;

void draw_thread_proc(){
  static float next_value=0.0;
  setup_background_layer();
  setup_foreground_layer();

  
  draw_graph_window(10);
  lcd.SelectLayer(FOREGROUND);
  for(int i=0;i<graph_width;i++){
    graph_pixels[i]= GRAPH_PADDING+graph_height;
  }
  lcd.SelectLayer(FOREGROUND);
  while(1){
    //wait for main thread to release a semaphore,
    //to indicate a new sample is ready to be graphed
    new_values_semaphore.acquire();
    new_values.pop(next_value);

		snprintf(display_buf[1],60,"Pressure: %.2f mmHg       ",next_value);	
  	//display the buffered string on the screen
  	lcd.DisplayStringAt(0, LINE(16), (uint8_t *)display_buf[1], LEFT_MODE); // write buffer to display

    for(int i = 0; i<(graph_width-1);i++){
        //the x coordinate of the graph value being updated.
        //think about it like a vertical line
        //that sweeps across the graph from left to right,
        //updating each point in the graph as it travels across.
        const uint32_t target_x_coord=GRAPH_PADDING+i;
         //y coordinate of the previous function value at the target x coordinate
         const uint32_t old_pixelY=graph_pixels[(i+next_write_index)%graph_width];
        //y coordinate of the current function value at the target x coordinate
        const uint32_t new_pixelY=graph_pixels[(i+next_write_index+1)%graph_width];
         //remove (set to black) the old pixel for the current x coordinate
         //from the screen
         lcd.DrawPixel(target_x_coord,old_pixelY,LCD_COLOR_BLACK);
        //draw the new pixel for the current x coordinate on the screen
        lcd.DrawPixel(target_x_coord,new_pixelY,LCD_COLOR_RED);
    }
    //retrieve and erase the right most(last) pixel in the graph
    const uint32_t last_old_pixelY= graph_pixels[(graph_width-1+next_write_index)%graph_width]; 
    lcd.DrawPixel(GRAPH_PADDING+graph_width-1,last_old_pixelY,LCD_COLOR_BLACK);
    //map, draw and store the newest value
    graph_pixels[next_write_index]=mapPixelY(next_value,0,160,GRAPH_PADDING,GRAPH_PADDING+graph_height);
    lcd.DrawPixel(GRAPH_PADDING+graph_width-1,graph_pixels[next_write_index],LCD_COLOR_RED);
    next_write_index=(next_write_index+1)%graph_width;
  }
}


//--------------------------------------------------------------------------------
// Button
//--------------------------------------------------------------------------------
InterruptIn ButtonInt(USER_BUTTON, PullDown);     // PA_0
// Setup the debounce timer
Timer ti;
volatile uint8_t start_flag = 0;

void button_handler(void) {
  // static variable - has a local scope but is initialised on startup
  volatile static uint32_t lastPressTime = 0;
  volatile uint32_t currentTime = ti.read_ms();
  
  // check if 300ms has passed since the last real press
  if (currentTime - lastPressTime > 300) {

    start_flag = 1;
    
    // save the time of the real press
    lastPressTime = currentTime ;
  }
}


//--------------------------------------------------------------------------------
// MAIN
//--------------------------------------------------------------------------------
#define PULSE_NUM 5;
//the flag of deflation: 1 means it is in the deflating process
volatile uint8_t deflate_flag; 
//the flag of pre rising: 1 means it is in the prerisng process
uint8_t prerise_flag;
//the flag of rising: 1 means it is in the real risng process
//we check twice to eliminate the effects of noise
uint8_t rise_flag;

//uint32_t  cur_time;
//uint32_t  last_time;

uint8_t pulse_counter;//the counter of pulse
uint8_t deflate_counter;   // deflate check per 100 point

int sensor_out; //output of sensor
float pressure;
float pressure_buf;//temporary variable that save the last pressure  

//temporary variable that save the last sensor value  
int prev_data;
//record the difference of the sensor value this time and the value last time 
int delta;

float sys;//systolic pressure
float dia;//diastolic pressure
float map;//max arterial pressure


float    hrate;//heart rate
uint32_t cur_pulse_time;//current pulse time
uint32_t last_pulse_time;//last pulse time
//int8_t  flag_cal_hrate;

float total_pulse_time;
//record[0] records the peak pressure
//record[1] records pressure at prerise point
//record[2] records time between two pulses
float record[3][100];

float amp;//amplitude of the pulse
float max_amp;//maximum amplitude of the pulse
int max_pulse_index;//the index of maximum pulse

  int main() {
	pressure=0;
	sensor_out=0;
	status=0;

  // Chip must be deselected
  cs = 1;    
  // Setup the spi for 8 bit data, clock polarity 0 and clock phase 0
  spi.format(8,0);
	// 800kHz clock rate
  spi.frequency(800'000);

  // Button
  ti.start(); 
  ButtonInt.rise(&button_handler);

	// LCD
	draw_thread.start(draw_thread_proc); 
  thread_sleep_for(1000);

  snprintf(display_buf[2],60,"Press button to start   ");	
  lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[2], LEFT_MODE); // write buffer to display
  
  while (1) {
		// Read sensor
		write_buf[0]=CMD_PRECEDE1;
    write_buf[1]=CMD_PRECEDE2;
		write_buf[2]=CMD_PRECEDE3;

    // Setp 1
    cs=0;
    spi.transfer(write_buf,3,read_buf,3,cb,SPI_EVENT_COMPLETE );
    flags.wait_all(READY_FLAG);

		thread_sleep_for(5);		

		// Setp 3
		write_buf[0]=CMD_NOP;
    write_buf[1]=0x00;
		write_buf[2]=0x00;
		write_buf[3]=0x00;

    cs=0;
    spi.transfer(write_buf,4,read_buf,4,cb,SPI_EVENT_COMPLETE );
    flags.wait_all(READY_FLAG);  
    //<32:24> saves the status data, which refers to the first one of read_buf 
		status = read_buf[0];
		// printf("status: 0x%02X \n", status);

		// Pressure Calculation
    //pressur=(output-output_min)*(Pmax-Pmin)/(Output_max-Output_min)+Pmin
    //take 22.5% of counts as its output_max, and 2.5% as its output_min 
	  //<24:0> saves the status data, which refers to the second one to fourth one of read_buf 
		sensor_out = int(read_buf[1]<<16)|(read_buf[2]<<8)|read_buf[3];
    pressure = float((float(sensor_out)-419430.0f )*300.0f /3355443.0f) + 0.7f;
    delta = sensor_out - prev_data;
    prev_data = sensor_out;
    printf("Sensor_out = %d, Pressure = %.2f\n",sensor_out, pressure);

		// LCD data push
		if(!new_values.full()){
      		//push the next value into the circular buffer
      		new_values.push(pressure);
      			if(new_values_semaphore.release()!=osOK){
        			MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION,MBED_ERROR_CODE_OUT_OF_MEMORY),"semaphor overflow\r\n");
      			}
    	}
    else{
     	MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_APPLICATION,MBED_ERROR_CODE_OUT_OF_MEMORY),"circular buffer is full\r\n");
    }

    // LCD UI display
    //The process of inflating, deflating and measuring of blood pressure and heart rate
    
    if(pressure<150){//The button can trigger deflate/inflate process only when the pressure<150
      if(deflate_flag==0){//Inflating process
        if(start_flag==1){//if the start button is pressed, start inflating 
          // initial
          pulse_counter=0;        // count pulse number during deflation
          deflate_counter=0;      // count data point during deflation
          total_pulse_time=0;     // measure heart rate
          max_amp=0;
          prerise_flag=0;
          rise_flag=0;


          snprintf(display_buf[3],60,"Inflate to 150mmHg   ");	
  	      lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[3], LEFT_MODE); // write buffer to display
          lcd.ClearStringLine(17);
          lcd.ClearStringLine(18);
          lcd.ClearStringLine(19);
          start_flag=0;//After inflating starts, reset start
        }
      }
      else{//deflating 
        snprintf(display_buf[5],60,"Deflate to 30mmHg   ");	
        lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[5], LEFT_MODE); // write buffer to display
      }
    }
    else{//The pressure reaches 150, inflating stops and press the button again to start deflate
      if(deflate_flag==0){
        if(start_flag==1){
          deflate_flag = 1;//After the pressure reaches 150, set the defalting flag to 1
        }
        else{
          snprintf(display_buf[4],60,"Press to deflate     ");	
          lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[4], LEFT_MODE); // write buffer to display
        }
      }
      else {//start to deflate
        snprintf(display_buf[5],60,"Deflate to 30mmHg   ");
        lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[5], LEFT_MODE); // write buffer to display
      }
    }

    // Blood pressure Calculation
    if(deflate_flag==1){
      //if the pressure is below 30 during the deflating process
      //It means the process of inflating and deflating finished, start calculating now
      if(pressure<30){
        //reset deflating flag and start flag
        deflate_flag=0;
        start_flag=0;

        // Data process

        //calculate and find the maximum pulse amplitude
        for(int k=1;k<pulse_counter+1;k=k+1){
          amp = record[0][k]-record[1][k-1];//amplitude=peak pressure-pressure at prerise point
          if(amp>max_amp){
            max_amp=amp;
            max_pulse_index=k;//save the index of maximum pulse
          }
        }
        
        // blood pressure
        sys = record[0][1];//systolic pressure=pressure at prerise point
        // dia = record[0][pulse_counter];
        map = record[0][max_pulse_index];
        dia = (3.0f*map - sys)/2.0f;//diastolic pressure：DIA = (3*MAP - SYS)/2


        // Heart Rate Calculation
        //We choose the middle five pulse times(for avoiding the noise effect) to get the average of them, and use it to calculate heartbeat.
        for(int j=15;j<20;j=j+1){
          total_pulse_time=record[2][j]+total_pulse_time;
        }
        hrate = (float)(60000.0f/((float)total_pulse_time/(float)5.0));

        // lcd
        snprintf(display_buf[6],60,"Press to restart     ");	
        lcd.DisplayStringAt(0, LINE(15), (uint8_t *)display_buf[6], LEFT_MODE); // write buffer to display
        snprintf(display_buf[7],60,"Systolic:  %.2f       ", sys);	
        lcd.DisplayStringAt(0, LINE(17), (uint8_t *)display_buf[7], LEFT_MODE); // write buffer to display
        snprintf(display_buf[8],60,"Diastolic: %.2f       ", dia);	
        lcd.DisplayStringAt(0, LINE(18), (uint8_t *)display_buf[8], LEFT_MODE); // write buffer to display
        snprintf(display_buf[9],60,"Heart Rate:%.2f     ", hrate);	
        lcd.DisplayStringAt(0, LINE(19), (uint8_t *)display_buf[9], LEFT_MODE); // write buffer to display
        
      }
      else{

        // deflate warning

        //check the deflating speed every 0.5 seconds
        //the loop is running every 18 ms, 0.5s/18ms=28.
        // So every time the deflate_counter is bigger than 28, we will check the speed
        if(deflate_counter>28){       // current sample interval 18ms
          deflate_counter=0;          // 0.5s/18ms=28
          if(pressure_buf-pressure>2.0f){
            snprintf(display_buf[7],60,"Deflate too fast!        ");	
            lcd.DisplayStringAt(0, LINE(17), (uint8_t *)display_buf[7], LEFT_MODE); // write buffer to display
          }
          else{
            lcd.ClearStringLine(17);
          }
          pressure_buf = pressure;//save the current pressure for next checking
        }
        else{
          deflate_counter++;
        }

        // peak detection
        if(pressure<150){//check if it is in deflating process
          if(rise_flag==1) {//make sure it already has a prerise point
        	  if(delta<-400) {
        		  pulse_counter++;
        		  record[0][pulse_counter]=pressure;//record the peak pressure

              cur_pulse_time= ti.read_ms();
              //record the time between two pulses
              record[2][pulse_counter-1]=cur_pulse_time-last_pulse_time;
              last_pulse_time=cur_pulse_time;

              snprintf(display_buf[8],60,"amp=%.2f        ", (record[0][pulse_counter]-record[1][pulse_counter-1]));	
              lcd.DisplayStringAt(0, LINE(18), (uint8_t *)display_buf[8], LEFT_MODE); // write buffer to display

        		  rise_flag=0;
        	  }
          }

          // rise detect
          //check the rising twice to avoid the noise effect
	        if(prerise_flag==1){	
		        if(delta>1500){
			        prerise_flag=0;
			        rise_flag=1;
			      }
		        else 
              prerise_flag=0; 
  		    }

          // pre rise detection
          if(rise_flag==0&&prerise_flag==0&&delta>1500){
            prerise_flag=1; 
            record[1][pulse_counter]=pressure;    // record pressure at pre rise point
          }
        }

        
        snprintf(display_buf[9],60,"pulse=%d rf:%d          ", pulse_counter,rise_flag);	
        lcd.DisplayStringAt(0, LINE(19), (uint8_t *)display_buf[9], LEFT_MODE); // write buffer to display

      }
    }

    thread_sleep_for(10);

    // // Cycle time calculation
    // int mscnt;
    // cur_time = ti.read_ms();
    // mscnt = cur_time-last_time;
    // printf("speed=%d \n",mscnt);
    // last_time = cur_time; 

  }

}


