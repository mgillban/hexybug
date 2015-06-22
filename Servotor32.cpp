#include "Servotor32.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "Arduino.h"

#include "Servotor32_SPI.h"
#include "Servotor32_TimerOne.h"

Servotor32::Servotor32()
{  

}

//stores information about the servos and groups
signed short servo_positions[SERVOS]; // where the servos are currently (supposed to be) at
signed char servos_sorted[GROUPS][SERVOS_PER_GROUP]; // index in servo_timings to where the servo ends
signed char servos_active_in_group[GROUPS]; // the number of servos in a group currently active
uint8_t active_servos_hex[GROUPS];

// all updates to shift registers in order of their updates
signed short servo_timings[MAX_TIMINGS]; // the timing where the change occurs
uint8_t  shift_output[MAX_TIMINGS];  // the output of the shift register
uint8_t  shift_latch[MAX_TIMINGS];   // the shift register latch used

// keeps track of whether its safe or not to update the servos
uint8_t update_reg_flag = 0;

// variables for the callback
uint16_t timer;
uint8_t  counter = 0;
uint8_t  pwm_active = 1;

uint16_t group_offsets[4] = {0,251,502,753};
uint8_t group_latches[4] = {5,6,7,4};
uint8_t pin_2_num[8] = {0x08,0x04,0x02,0x01, 0x80,0x40,0x20,0x10};

void Servotor32::begin(){
  //setup pin modes
  DDRF |= 0xF0;  // sets pins F7 to F4 as outputs
  DDRB = 0xFF;  // sets pins B0 to B7 as outputs
  
  //setup PC serial port
  Serial.begin(9600);
  // reconfigure bluetooth module to 9600 baud id needed
  Serial1.begin(115200);     // Changed from 9600 baud
  Serial1.print("AT+BAUD4"); // Tell the module to change the baud rate to 9600
  delay(1100); // Wait a notch over 1 second to make sure the setting "sticks"
  Serial1.begin(9600);     // Changed from 9600 baud
  
  SPI.begin(); 
  SPI.setClockDivider(SPI_CLOCK_DIV2); 

  Timer1.initialize(10);
  Timer1.attachInterrupt(callback);

  for(byte i=0; i<SERVOS; i++){
    servo_positions[i] = -1;
  }
  for(byte i=0; i<GROUPS; i++){
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      servos_sorted[i][j] = -1;
    }
  }
  
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  } 

  TIMSK0 &= ~(_BV(TOIE0)); // disables the arduino delay function, but also
                           // all but eliminates servo jitter 
  TIMSK2 &= ~(_BV(TOIE2)); // disable the arduino tone  function, but also
                           // also helps eliminate some jitter
  TIMSK3 &= ~(_BV(TOIE3)); // for good measure
  TIMSK4 &= ~(_BV(TOIE4)); // for good measure 
}

long unsigned int us_counter = 0;
long unsigned int startTime = 0; 
long unsigned int currentTime = 0; 
long unsigned int last_update = 0;

long unsigned int Servotor32::micros_new(){
  return us_counter;
}

long unsigned int Servotor32::millis_new(){
  return us_counter/1000;
}

void Servotor32::delay_ms(long unsigned int delay_time){
  startTime = millis_new();
  currentTime = millis_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = millis_new() - startTime;
  }
}

void Servotor32::delay_us(long unsigned int delay_time){
  startTime = micros_new();
  currentTime = micros_new() - startTime;
  while(currentTime < delay_time){
    delayMicroseconds(10);
    currentTime = micros_new() - startTime;
  }
}

void Servotor32::callback(){
  cli();
  if(timer < 1100){ // keep it from updating servos mid-array change by some weird coincidence
    if(timer == servo_timings[counter]){ // if the time has arrived to update a shift reg
      SPDR = shift_output[counter]; // push the byte to be loaded to the SPI register
      while(!(SPSR & (1<<SPIF))); //wait till the register completes
      PORTF &= ~(shift_latch[counter]); // clock the shift register latch pin low, setting the register
      PORTF |= shift_latch[counter];  // clock the shift register latch pin high, ready to be set low next time
      counter++;
    }
  }

  timer++;
  us_counter += 10;
  if(timer == 1100){ // all servo timing completed
    update_reg_flag = 1; // allow updates to the timing arrays
  }
  if(timer == 1900){ // getting close to servo start-up again,
    update_reg_flag = 0; // don't allow any new timing array updates
  }
  if(timer == 2000){
    timer=0;
    counter=0;
  }
  sei();
}

void Servotor32::delete_from_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<servos_active_in_group[group]; i++){ // go through all the servos
    if(servos_sorted[group][i] == servo){ // find its place
       for(signed char j=i; j<servos_active_in_group[group]-1; j++){//move all servos in front of it back by one
         servos_sorted[group][j] = servos_sorted[group][j+1];
       }
       servos_sorted[group][servos_active_in_group[group]-1] = -1; //insert a -1 at the end of the move
       break; //break out of previous for loop, now that the job is done
    }
  }
  active_servos_hex[group] &= ~pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] -= 1;// decrease the number of active servos in the group by 1
}

void Servotor32::add_to_sorted_array(byte servo, byte group, signed short pos){
  for(byte i=0; i<=servos_active_in_group[group]; i++){ // find the servo
     if(servos_sorted[group][i] == -1){ // if no servos yet entered, set as first
       servos_sorted[group][i] = servo; //insert the servo in its sorted place
       break; //stop the for loop, as the job is done
     }
     else{
       if(servo_positions[servos_sorted[group][i]] > pos){ // if this servo should go before this one
         for(signed char j=servos_active_in_group[group]-1; j>=i; j--){// move all others forward one
           servos_sorted[group][j+1] = servos_sorted[group][j];
         }
         servos_sorted[group][i] = servo; //insert the servo in its sorted place
         break;
       }
     }
  }
  active_servos_hex[group] |= pin_2_num[servo-group*SERVOS_PER_GROUP];
  servos_active_in_group[group] += 1;
}

void Servotor32::update_registers_fast(byte servo, signed short pos){
  byte group = servo/8;
  while(update_reg_flag == 0){ // wait for the servos to stop pulsing before updating the timing arrays
    delayMicroseconds(10);
  }
  // ----- put the servo into, or take it out of its sorted array ------
  
  if(pos > 0){ // if the sevo isn't a kill command, then its an add/change
    if(servo_positions[servo] == -1){// if the servo is inactive
      // insert the servo into the array sorted
      add_to_sorted_array(servo,group,pos);
    }
    else{
      // updating the servo. First delete its existing entry, then insert it

      delete_from_sorted_array(servo,group,pos);
      add_to_sorted_array(servo,group,pos);
    }
  }
  else{ // servo is a kill command
    if(servo_positions[servo] != -1){ // make sure its even on first
      delete_from_sorted_array(servo,group,pos);
    }
  }
  
  servo_positions[servo] = pos;
  
  // ----- create timing idicies from servo/group data -------
  
  // clear the timing arrays for fresh start
  for(uint8_t i=0; i<MAX_TIMINGS; i++){
    servo_timings[i] = 0;
    shift_output[i] = 0xFF;
    shift_latch[i] = 0xFF;
  }

  uint8_t counter_index=0;
  uint8_t current_timing=0;
  uint8_t current_shift_output=0; 
  
  for(byte group=0; group<GROUPS; group++){ //go through each group
    if(servos_active_in_group[group] > 0){ // skip it if the group is active, otherwise:
      servo_timings[counter_index] = group_offsets[group];
      shift_output[counter_index] = active_servos_hex[group];
      shift_latch[counter_index] = (1<<group_latches[group]);
      counter_index +=1;
      
      
      //create additional timings
      for(byte i=0; i<servos_active_in_group[group]; i++){ //create the timings for each servo after that, using the previous output
        if(servo_positions[servos_sorted[group][i]] == servo_positions[servos_sorted[group][i-1]]){ // if this servo's time is the same as the last's
          if(i != 0){
            counter_index -= 1; //reverse the index count-up
          }
          else{
            current_shift_output = shift_output[counter_index-1];
            servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
            shift_latch[counter_index] = (1<<group_latches[group]);
          }
        }
        else{
          current_shift_output = shift_output[counter_index-1];
          servo_timings[counter_index] = servo_positions[servos_sorted[group][i]]+ group_offsets[group];
          shift_latch[counter_index] = (1<<group_latches[group]);
        }
        
        //subtract the current servo from the shift register output
        current_shift_output &= ~pin_2_num[servos_sorted[group][i]-group*SERVOS_PER_GROUP]; 
        shift_output[counter_index] = current_shift_output;
        counter_index +=1;
      }
    }      
  }
  
}


void Servotor32::printStatus(Stream *serial){
  serial->println("--------------------- Registers ----------------------");
  
  serial->println("Servo Data:");
  serial->println("Servo\tPos\tTimeEnd\t");
  for(byte i=0; i<SERVOS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servo_positions[i]);
    serial->println("");
  }
  serial->println("");

  serial->println("Sorted Groups");
  for(byte i=0; i<GROUPS; i++){
    serial->print("Group: ");
    serial->println(i);
    for(byte j=0; j<SERVOS_PER_GROUP; j++){
      serial->print("Servo: ");
      serial->print(servos_sorted[i][j]);
      serial->print("\t");
      serial->println(servo_positions[servos_sorted[i][j]]);
      
    }
  }  

  serial->println("Group Data:");
  serial->println("#\tActive\tHex");
  for(byte i=0; i<GROUPS; i++){
    serial->print(i);
    serial->print("\t");
    serial->print(servos_active_in_group[i]);
    serial->print("\t");
    serial->println(active_servos_hex[i],HEX);
  }
  serial->println("");
  
  serial->println("Timings:");
  serial->println("Pos\tTiming\tOutput\tLatch");
  for(uint8_t i=0; i<MAX_TIMINGS; i++){ // clear existing registers, so they can be cleanly written
    serial->print(i);
    serial->print(":\t");
    serial->print(servo_timings[i]);
    serial->print(",\t");
    serial->print(shift_output[i],HEX);
    serial->print(",\t");
    serial->println(shift_latch[i],HEX);
  }
  serial->println("----------------------------------------------------");
}

/* ************************************************************************************************************** */
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// modify the state of a servo
void Servotor32::changeServo(byte servo, short pos){
  if(pos == 0){
    pos = -1;
  }
  if(pos == -1){
    update_registers_fast(servo, pos);
  }
  else{
    update_registers_fast(servo, pos/10);
  }
}

void Servotor32::standUp()//////////////////////////////////////////////////////////////////////////
{
  changeServo(7,1950);//Hip position
changeServo(11,1500);
changeServo(15,1250);
changeServo(24,1050);
changeServo(20,1500);
changeServo(16,1750);
delay_ms(200);
for(int i=0;i<32;i++)//KillServos
{
    changeServo(i,-1);
}

changeServo(6,900);//Knee position
changeServo(10,900);
changeServo(14,900);
changeServo(25,900);
changeServo(21,900);
changeServo(17,900);
delay_ms(200);
for(int i=0;i<32;i++)//KillServos
{
    changeServo(i,-1);
}

changeServo(5,600);//Ankle position
changeServo(9,600);
changeServo(13,600);
changeServo(26,600);
changeServo(22,600);
changeServo(18,600);
delay_ms(500);
for(int i=0;i<32;i++)//KillServos
{
    changeServo(i,-1);
}

int knee[6] = {6,21,14,25,10,17};
int ankle[6] = {5,22,23,26,9,18};

//Stand
int ankAngle = 600;
for(int angle=1300; angle<2100; angle+=30)
{
  for(int h=0;h<6;h++)
  {
    changeServo(knee[h],angle);
    changeServo(ankle[h],ankAngle);
    delay_ms(100);
    changeServo(ankle[h],-1);
  }
  ankAngle+=15;
}

changeServo(17,1500);//RB
changeServo(16,1500);
delay_ms(200);
changeServo(18,500);
delay_ms(100);
changeServo(17,2100);
changeServo(18,1050);
delay_ms(200);
changeServo(18,-1);
changeServo(16,-1);
//changeServo(17,-1);
delay_ms(100);

changeServo(6,1500);//LF
changeServo(7,1500);
delay_ms(200);
changeServo(5,600);
delay_ms(100);
changeServo(6,2100);
changeServo(5,1050);
delay_ms(200);
changeServo(5,-1);
//changeServo(6,-1);
changeServo(7,-1);
delay_ms(100);
  
changeServo(25,1500);//RF
changeServo(24,1500);
delay_ms(200);
changeServo(26,600);
delay_ms(100);
changeServo(25,2100);
changeServo(26,1050);
delay_ms(200);
changeServo(24,-1);
changeServo(26,-1);
//changeServo(25,-1);
delay_ms(100);

changeServo(14,1500);//LB
changeServo(15,1500);
delay_ms(200);
changeServo(13,500);
delay_ms(100);
changeServo(14,2100);
changeServo(13,1050);
delay_ms(200);
changeServo(13,-1);
changeServo(15,-1);
//changeServo(14,-1);
delay_ms(100);

for(int i=0;i<32;i++)//KillServos
{
    changeServo(i,-1);
}

  
}

void Servotor32::center()////////////////////////////////////////////////////////////
{
  for(int i=0;i<32;i++)//CenterServos
{
  changeServo(i,1500);
  delay_ms(200);  
  changeServo(i,-1);
}
}

void Servotor32::killServos()///////////////////////////////////////////////////////////////////////////////
{
  for(int i=0;i<32;i++)//KillServos
{  
  changeServo(i,-1);
}
}

void Servotor32::reset()/////////////////////////////////////////////////////////////////////////////////
{
 changeServo(6,1950); //Stabalizes knees
 changeServo(10,1950); 
 changeServo(14,1950); 
 changeServo(25,1950); 
 changeServo(21,1950); 
 changeServo(17,1950); 
 delay_ms(100);
 
changeServo(17,1500);//RB
changeServo(16,1500);
delay_ms(200);
changeServo(18,500);
delay_ms(100);
changeServo(17,1950);
changeServo(18,1050);
delay_ms(200);
changeServo(18,-1);
changeServo(16,-1);
//changeServo(17,-1);
delay_ms(100);

changeServo(6,1500);//LF
changeServo(7,1500);
delay_ms(200);
changeServo(5,600);
delay_ms(100);
changeServo(6,1950);
changeServo(5,1050);
delay_ms(200);
changeServo(5,-1);
//changeServo(6,-1);
changeServo(7,-1);
delay_ms(100);

changeServo(21,1500);//RM
changeServo(20,1500);
delay_ms(200);
changeServo(22,500);
delay_ms(100);
changeServo(21,1950);
changeServo(22,1050);
delay_ms(200);
changeServo(22,-1);
changeServo(20,-1);
//changeServo(21,-1);
delay_ms(100);
  
changeServo(25,1500);//RF
changeServo(24,1500);
delay_ms(200);
changeServo(26,600);
delay_ms(100);
changeServo(25,1950);
changeServo(26,1050);
delay_ms(200);
changeServo(24,-1);
changeServo(26,-1);
//changeServo(25,-1);
delay_ms(100);

changeServo(14,1500);//LB
changeServo(15,1500);
delay_ms(200);
changeServo(13,500);
delay_ms(100);
changeServo(14,1950);
changeServo(13,1050);
delay_ms(200);
changeServo(13,-1);
changeServo(15,-1);
//changeServo(14,-1);
delay_ms(100);
changeServo(6,-1); 
 changeServo(10,-1); 
 changeServo(14,-1); 
 changeServo(25,-1); 
 changeServo(21,-1); 
 changeServo(17,-1);
 delay_ms(10);
 
 changeServo(10,1500);//LM
changeServo(11,1500);
delay_ms(200);
changeServo(9,500);
delay_ms(100);
changeServo(10,1950);
changeServo(9,1050);
delay_ms(200);
changeServo(9,-1);
changeServo(10,-1);
//changeServo(11,-1);
delay_ms(100);
}

void Servotor32::forward()///////////////////////////////////////////////////////////////////
{
//Two tripod system, denoted 1<LB,LF,RM> and 2<RB,RF,LM>
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(14,1950);//Stabalizer Feet 1
  changeServo(6,1950);
  changeServo(21,1950);
  delay_ms(200);
  
  changeServo(25,1750);//Pick Up Knees 2
  changeServo(17,1750);
  changeServo(10,1750);
  delay_ms(200);
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(11,1250);//Reposition Hips 2
  changeServo(24,1750);
  changeServo(16,1750);
  delay_ms(200);
    changeServo(11,-1);//Kill Hips 2
  changeServo(24,-1);
  changeServo(16,-1);
  delay_ms(200);

  changeServo(9,950);//Reposition ankles 2
  changeServo(18,950);
  changeServo(26,950);
  delay_ms(300);
  changeServo(9,-1);//Kill ankles 2
  changeServo(18,-1);
  changeServo(26,-1);
  delay_ms(20);

  changeServo(25,1950);//Put down Knees 2, stay on
  changeServo(17,1950);
  changeServo(10,1950);
  delay_ms(200);
  
  changeServo(14,1750);//Pick up knees 1
  changeServo(6,1750);
  changeServo(21,1750);
  delay_ms(100);
  changeServo(14,-1);//Kill knees 1
  changeServo(6,-1);
  changeServo(21,-1);
  delay_ms(10);
  
  changeServo(22,950);//Reposition ankles 1
  changeServo(13,950);
  changeServo(5,950);
  delay_ms(300);
  changeServo(22,-1);//Kill ankles 1
  changeServo(13,-1);
  changeServo(5,-1);
  delay_ms(20);
  
  changeServo(7,1250);//Move hips
  changeServo(20,1750);
  changeServo(15,1250);
  delay_ms(100);
  changeServo(24,1250);
  changeServo(11,1750);
  changeServo(16,1250);
  delay_ms(100);
  changeServo(7,-1);
  changeServo(11,-1);
  changeServo(15,-1);
  changeServo(24,-1);
  changeServo(20,-1);
  changeServo(16,-1);
  delay_ms(10);
  
  changeServo(14,1950);//Replant knees 1
  changeServo(6,1950);
  changeServo(21,1950);
  delay_ms(100);
  
  changeServo(25,1750);//Pick Up Knees 2
  changeServo(17,1750);
  changeServo(10,1750);
  delay_ms(200);
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(9,950);//Reposition ankles 2
  changeServo(18,950);
  changeServo(26,950);
  delay_ms(300);
  changeServo(9,-1);//Kill ankles 2
  changeServo(18,-1);
  changeServo(26,-1);
  delay_ms(20);
  
  changeServo(7,1750);//Move hips
  changeServo(20,1250);
  changeServo(15,1750);
  delay_ms(100);
  changeServo(24,1750);
  changeServo(11,1250);
  changeServo(16,1750);
  delay_ms(100);
  changeServo(7,-1);
  changeServo(11,-1);
  changeServo(15,-1);
  changeServo(24,-1);
  changeServo(20,-1);
  changeServo(16,-1);
  delay_ms(10);
  
  changeServo(25,1950);//Put down Knees 2, stay on
  changeServo(17,1950);
  changeServo(10,1950);
  delay_ms(200);
  changeServo(25,-1);//Kill knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  }
  
  void Servotor32::backward()////////////////////////////////////////////////////////////////////////////////////
{
//Two tripod system, denoted 1<LB,LF,RM> and 2<RB,RF,LM>
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(14,1950);//Stabalizer Feet 1
  changeServo(6,1950);
  changeServo(21,1950);
  delay_ms(200);
  
  changeServo(25,1750);//Pick Up Knees 2
  changeServo(17,1750);
  changeServo(10,1750);
  delay_ms(200);
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(11,1750);//Reposition Hips 2
  changeServo(24,1250);
  changeServo(16,1250);
  delay_ms(200);
    changeServo(11,-1);//Kill Hips 2
  changeServo(24,-1);
  changeServo(16,-1);
  delay_ms(200);

  changeServo(9,950);//Reposition ankles 2
  changeServo(18,950);
  changeServo(26,950);
  delay_ms(300);
  changeServo(9,-1);//Kill ankles 2
  changeServo(18,-1);
  changeServo(26,-1);
  delay_ms(20);

  changeServo(25,1950);//Put down Knees 2, stay on
  changeServo(17,1950);
  changeServo(10,1950);
  delay_ms(200);
  
  changeServo(14,1750);//Pick up knees 1
  changeServo(6,1750);
  changeServo(21,1750);
  delay_ms(100);
  changeServo(14,-1);//Kill knees 1
  changeServo(6,-1);
  changeServo(21,-1);
  delay_ms(10);
  
  changeServo(22,950);//Reposition ankles 1
  changeServo(13,950);
  changeServo(5,950);
  delay_ms(300);
  changeServo(22,-1);//Kill ankles 1
  changeServo(13,-1);
  changeServo(5,-1);
  delay_ms(20);
  
  changeServo(7,1750);//Move hips
  changeServo(20,1350);
  changeServo(15,1750);
  delay_ms(100);
  changeServo(24,1750);
  changeServo(11,1250);
  changeServo(16,1750);
  delay_ms(100);
  changeServo(7,-1);
  changeServo(11,-1);
  changeServo(15,-1);
  changeServo(24,-1);
  changeServo(20,-1);
  changeServo(16,-1);
  delay_ms(10);
  
  changeServo(14,1950);//Replant knees 1
  changeServo(6,1950);
  changeServo(21,1950);
  delay_ms(100);
  
  changeServo(25,1750);//Pick Up Knees 2
  changeServo(17,1750);
  changeServo(10,1750);
  delay_ms(200);
  changeServo(25,-1);//Kill Knees 2
  changeServo(17,-1);
  changeServo(10,-1);
  delay_ms(20);
  
  changeServo(7,1250);//Move hips
  changeServo(20,1750);
  changeServo(15,1250);
  delay_ms(100);
  changeServo(24,1250);
  changeServo(11,1750);
  changeServo(16,1250);
  delay_ms(100);
  changeServo(7,-1);
  changeServo(11,-1);
  changeServo(15,-1);
  changeServo(24,-1);
  changeServo(20,-1);
  changeServo(16,-1);
  delay_ms(10);
  }
  
 void Servotor32::rotateRight(){
    
    // Picks up the knees from tripod 1, rotates them to the front most position, then places them down. Picks up tripod 2, then rotates tripod 1 (while still on the ground) to middle position, then replaces tripod 2
   
    //tripod 1 == RF, LM, RB
    //tripod 2 == LF, RM, LB
    
    //put the servos in the correct position from stand up function
    //Final positions: hips 1500, knees ~1950, ankles 1050
    reset();
    
    //pick up tripod 1
    changeServo(25, 2000);
    delay_ms(25);
    changeServo(10, 2000);
    delay_ms(25);
    changeServo(17, 2000);
    delay_ms(25);
    
    //move tripod to forward most position
    changeServo(24, 2200);
    delay_ms(25);
    changeServo(11, 2200);
    delay_ms(25);
    changeServo(16, 2200);
    delay_ms(25);
    
    //move tripod 1 down
    changeServo(25, 1950);
    delay_ms(25);
    changeServo(10, 1950);
    delay_ms(25);
    changeServo(17, 1950);
    delay_ms(25);
    
    //move tripod 2 up
    changeServo(6, 2200);
    delay_ms(25);
    changeServo(21, 2200);
    delay_ms(25);
    changeServo(14, 2200);
    delay_ms(25);
    
    //rotate tripod 1 back to original position
    changeServo(24, 1500);
    delay_ms(25);
    changeServo(11, 1500);
    delay_ms(25);
    changeServo(16, 1500);
    delay_ms(25);
    
    // move tripod 2 back down
      changeServo(6, 1950);
    delay_ms(25);
    changeServo(21, 1950);
    delay_ms(25);
    changeServo(14, 1950);
    delay_ms(25);
    
    reset();
    
  }


void Servotor32::kickRight()////////////////////////////////////////////////////////////////////////////////////
{
  changeServo(21,700);
  changeServo(22,700);
  delay_ms(500);
  changeServo(21,1500);
  changeServo(22,1500);
  delay_ms(1000);
  changeServo(21,1950);
  changeServo(22,1050);
  delay_ms(400);
  changeServo(21,-1);
  changeServo(22,-1);
  delay_ms(100);
}

void Servotor32::kickLeft()////////////////////////////////////////////////////////////////////////////////////
{
  changeServo(10,700);
  changeServo(9,700);
  delay_ms(500);
  changeServo(10,1500);
  changeServo(9,1500);
  delay_ms(1000);
  changeServo(10,1950);
  changeServo(9,1050);
  delay_ms(400);
  changeServo(10,-1);
  changeServo(9,-1);
  delay_ms(100);
}
// /////////////////////////////////////////////////////////////////////////////////////////////////////

/* *************************************************************************************************************** */
boolean debug = false;
boolean testMode = false;
boolean servoCounting = false;
boolean posCounting = false;

byte numString[6];
int powers[] = {1,10,100,1000};

byte numCount = 0;
unsigned short total = 0;
short inServo = -1;
short inPos = -1;

void Servotor32::process(Stream *serial){
  if(serial->available()) { //process input from the USB
    char inChar = (char)serial->read();
    switch(inChar){
      case '#':
        servoCounting = true;
        numCount = 0;
        inServo = -1;
        inPos = -1;
        break;
      case 'D':
        printStatus(serial);
        break; 
      case 'P':
        if(servoCounting){
          inServo = tallyCount();
          servoCounting = false;
        }
        posCounting =  true;
        numCount = 0;
        break; 
      case '\r':
      case '\n':
        if(posCounting){
          inPos = tallyCount();
          posCounting = false;
        }
        if((inServo >=0)&&(inServo <=31)&&(((inPos >= 500)&&(inPos <= 2500))||(inPos == -1))){
          changeServo(inServo,inPos);        
          inServo = -1;
          inPos = -1;
        }
        numCount = 0;
        break;
      case 'V':
        serial->println("SERVOTOR32_v2.0");
        break;
      case 'C':
        for(int i=0; i<32; i++){
          changeServo(i, 1500);
        }
        serial->println("All Centered");
        break;
      case 'K':
        for(int i=0; i<32; i++){
          changeServo(i,-1);
        }
        serial->println("All Turned Off");
        break;
      case 'L':
        if(servoCounting){
          inServo = tallyCount();
          servoCounting = false;
        }
        changeServo(inServo, -1);
        break;
      default:
        if((inChar > 47)&&(inChar < 58)){
          if(numCount<4){
            numString[numCount] = inChar-48;
            numCount++;
          }
        }
        break;
    }
  }
}

short Servotor32::tallyCount(){
   total=0;
   for(int i=0; i<numCount; i++){  
     total += powers[i]*numString[(numCount-1)-i];  
   }
   if(numCount == 0){
     total = -1;
   }
   return total;
}

#define MAX_TIME 1000000

float Servotor32::ping(){
  //PB0 for Trigger (17)
  //PB7 for Echo (11)
  
  pinMode(17,OUTPUT);
  pinMode(11,INPUT);

  long duration; 
  float cm;
  digitalWrite(17, LOW); 
  delayMicroseconds(2); 
  digitalWrite(17, HIGH); 
  delayMicroseconds(10); 
  digitalWrite(17, LOW); 
  

  uint8_t bit = digitalPinToBitMask(11);
  uint8_t port = digitalPinToPort(11);
  uint8_t stateMask = (HIGH ? bit : 0);
  
  unsigned long startCount = 0;
  unsigned long endCount = 0;
  unsigned long width = 0; // keep initialization out of time critical area
  
  // convert the timeout from microseconds to a number of times through
  // the initial loop; it takes 16 clock cycles per iteration.
  unsigned long numloops = 0;
  unsigned long maxloops = 500;
	
  // wait for any previous pulse to end
  while ((*portInputRegister(port) & bit) == stateMask)
    if (numloops++ == maxloops)
      return 0;
	
  // wait for the pulse to start
  while ((*portInputRegister(port) & bit) != stateMask)
    if (numloops++ == maxloops)
      return 0;
  
  startCount = micros_new();
  // wait for the pulse to stop
  while ((*portInputRegister(port) & bit) == stateMask) {
    if (numloops++ == maxloops)
      return 0;
    delayMicroseconds(10); //loop 'jams' without this
    if((micros_new() - startCount) > 58000 ){ // 58000 = 1000CM
      return 0;
      break;
    }
  }
  duration = micros_new() - startCount;
  //--------- end pulsein
  cm = (float)duration / 29.0 / 2.0; 
  return cm;
}

float Servotor32::multiPing(unsigned short attempts=5){
  float distances [attempts];
  for(int i=0; i<attempts; i++){
    distances[i] = ping();
  }
  
  // sort them in order
  int i, j;
  float temp;
 
  for (i = (attempts - 1); i > 0; i--)
  {
    for (j = 1; j <= i; j++)
    {
      if (distances[j-1] > distances[j])
      {
        temp = distances[j-1];
        distances[j-1] = distances[j];
        distances[j] = temp;
      }
    }
  }
  
  // return the middle entry
  return distances[(int)ceil((float)attempts/2.0)];
  
}
