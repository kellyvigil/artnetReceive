/*
ARTNET RECEIVER v3.1

This SCRIPT allows you to use arduino with ethernet shield or wifi shield and recieve artnet data. Up to you to use channels as you want.

Tested with Arduino 1.0.5, so this code should work with the new EthernetUdp library (instead of the depricated Udp library)

If you have implemented improvements to this sketch, please contribute by sending back the modified sketch. It will be a pleasure to let them accessible to community

Original code by (c)Christoph Guillermet, designed to be used with the free open source lighting software WhiteCat: http://www.le-chat-noir-numerique.fr
karistouf@yahoo.fr

v3, modifications by David van Schoorisse <d.schoorisse@gmail.com>
Ported code to make use of the new EthernetUdp library used by Arduino 1.0 and higher.

V3.1 by MSBERGER 130801
- performance gain by shrinking buffer sizes from "UDP_TX_PACKET_MAX_SIZE" to 768
- implementation of selction / filtering SubnetID and UniverseID (was already prepared by karistouf)
- channel count starts at 0 instead of 1 (the digital and vvvv way)
- artnet start_address+n is now mapped to "arduino-channel" 0+n (was also start_address+n bevore), now it is similar to lighting fixtures



*/

#include <Dhcp.h>
#include <Dns.h>
#include <Ethernet2.h>
#include <EthernetClient.h>
#include <EthernetServer.h>
#include <EthernetUdp2.h>
#include <Twitter.h>
#include <util.h>
#include <SPI.h>

#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x)  (LOW_BYTE & x)
#define bytes_to_short(h,l) ( ((h << 8) & 0xff00) | (l & 0x00FF) );

byte mac[] = {0x90, 0xA2, 0xDA, 0x10, 0x18, 0x4D} ; //the mac adress in HEX of ethernet shield or uno shield board
byte ip[] = {172, 20, 0, 51}; // the IP adress of your device, that should be in same universe of the network you are using

// the next two variables are set when a packet is received
byte remoteIp[4];        // holds received packet's originating IP
unsigned int remotePort; // holds received packet's originating port

//customisation: Artnet SubnetID + UniverseID
//edit this with SubnetID + UniverseID you want to receive
byte SubnetID = {0};
byte UniverseID = {0};
short select_universe = ((SubnetID * 16) + UniverseID);

//customisation: edit this if you want for example read and copy only 4 or 6 channels from channel 12 or 48 or whatever.
const int number_of_channels = 512; //512 for 512 channels
const int start_address = 0; // 0 if you want to read from channel 1

//buffers
const int MAX_BUFFER_UDP = 768;
char packetBuffer[MAX_BUFFER_UDP]; //buffer to store incoming data
byte buffer_channel_arduino[number_of_channels]; //buffer to store filetered DMX data

// art net parameters
unsigned int localPort = 6454;      // artnet UDP port is by default 6454
const int art_net_header_size = 17;
const int max_packet_size = 576;
char ArtNetHead[8] = "Art-Net";
char OpHbyteReceive = 0;
char OpLbyteReceive = 0;
//short is_artnet_version_1=0;
//short is_artnet_version_2=0;
//short seq_artnet=0;
//short artnet_physical=0;
short incoming_universe = 0;
boolean is_opcode_is_dmx = 0;
boolean is_opcode_is_artpoll = 0;
boolean match_artnet = 1;
short Opcode = 0;
EthernetUDP Udp;


//////////NeoPixel Stuff
#include <Adafruit_NeoPixel.h>
#include <avr/power.h>

#define NUMPIXELS      60

#define PIN 3

// Parameter 1 = number of pixels in strip
// Parameter 2 = Arduino pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

// IMPORTANT: To reduce NeoPixel burnout risk, add 1000 uF capacitor across
// pixel power leads, add 300 - 500 Ohm resistor on first pixel's data input
// and minimize distance between Arduino and first pixel.  Avoid connecting
// on a live circuit...if you must, connect GND first.


/*
///////DotStar Shit
#include <Adafruit_DotStar.h>

#define NUMPIXELS 30 // Number of LEDs in strip

// Here's how to control the LEDs from any two pins:
#define DATAPIN    4
#define CLOCKPIN   5
Adafruit_DotStar pixel = Adafruit_DotStar(
  NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);
// The last parameter is optional -- this is the color data order of the
// DotStar strip, which has changed over time in different production runs.
// Your code just uses R,G,B colors, the library then reassigns as needed.
// Default is DOTSTAR_BRG, so change this if you have an earlier strip.

// Hardware SPI is a little faster, but must be wired to specific pins
// (Arduino Uno = pin 11 for data, 13 for clock, other boards are different).
//Adafruit_DotStar strip = Adafruit_DotStar(NUMPIXELS, DOTSTAR_BRG);

*/
int delayval = 20; // delay for half a second



void setup() {
  //setup pins as PWM output
  //Serial.begin(115200);
  //pinMode(3, OUTPUT);  //check with leds + resistance in pwm, this will not work with pins 10 and 11, used by RJ45 shield
  //pinMode(5, OUTPUT);  //check with leds + resistance in pwm, this will not work with pins 10 and 11, used by RJ45 shield
  //pinMode(6, OUTPUT);  //check with leds + resistance in pwm, this will not work with pins 10 and 11, used by RJ45 shield

  //setup ethernet and udp socket
  Ethernet.begin(mac, ip);
  Udp.begin(localPort);
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000L)
  clock_prescale_set(clock_div_1); // Enable 16 MHz on Trinket
#endif

  strip.begin(); // Initialize pins for output
  strip.show();  // Turn all LEDs off ASAP
}

void loop() {
  int packetSize = Udp.parsePacket();

  //FIXME: test/debug check
  if (packetSize > art_net_header_size && packetSize <= max_packet_size) { //check size to avoid unneeded checks
    //if(packetSize) {

    IPAddress remote = Udp.remoteIP();
    remotePort = Udp.remotePort();
    Udp.read(packetBuffer, MAX_BUFFER_UDP);

    //read header
    match_artnet = 1;
    for (int i = 0; i < 7; i++) {
      //if not corresponding, this is not an artnet packet, so we stop reading
      if (char(packetBuffer[i]) != ArtNetHead[i]) {
        match_artnet = 0; break;
      }
    }

    //if its an artnet header
    if (match_artnet == 1) {
      //artnet protocole revision, not really needed
      //is_artnet_version_1=packetBuffer[10];
      //is_artnet_version_2=packetBuffer[11];*/

      //sequence of data, to avoid lost packets on routeurs
      //seq_artnet=packetBuffer[12];*/

      //physical port of  dmx N°
      //artnet_physical=packetBuffer[13];*/

      //operator code enables to know wich type of message Art-Net it is
      Opcode = bytes_to_short(packetBuffer[9], packetBuffer[8]);

      //if opcode is DMX type
      if (Opcode == 0x5000) {
        is_opcode_is_dmx = 1; is_opcode_is_artpoll = 0;
      }

      //if opcode is artpoll
      else if (Opcode == 0x2000) {
        is_opcode_is_artpoll = 1; is_opcode_is_dmx = 0;
        //( we should normally reply to it, giving ip adress of the device)
      }

      //if its DMX data we will read it now
      if (is_opcode_is_dmx = 1) {

        //read incoming universe
        incoming_universe = bytes_to_short(packetBuffer[15], packetBuffer[14])
                            //if it is selected universe DMX will be read
        if (incoming_universe == select_universe) {

          //getting data from a channel position, on a precise amount of channels, this to avoid to much operation if you need only 4 channels for example
          //channel position
          for (int i = start_address; i < number_of_channels; i++) {
            buffer_channel_arduino[i - start_address] = byte(packetBuffer[i + art_net_header_size + 1]);
          }
        }
      }
    }//end of sniffing


  //pixels.setPixelColor(1, buffer_channel_arduino[0], buffer_channel_arduino[1], buffer_channel_arduino[2]);
 
    //stuff to do on PWM or whatever
//    Serial.print(buffer_channel_arduino[0]);
//    Serial.print("//");
//    Serial.print(buffer_channel_arduino[1]);
//    Serial.print("//");
//    Serial.println(buffer_channel_arduino[2]);
    //    Serial.print("//");
    //    Serial.print(buffer_channel_arduino[3]);
    //    Serial.print("//");
    //    Serial.print(buffer_channel_arduino[4]);
    //    Serial.print("//");
    //    Serial.println(buffer_channel_arduino[5]);
    
    int ledNum;
    for(ledNum = 0; ledNum < NUMPIXELS; ledNum++){
      strip.setPixelColor(ledNum, buffer_channel_arduino[ledNum*3], buffer_channel_arduino[ledNum*3+1], buffer_channel_arduino[ledNum*3+2]);
   
    }




  }

  strip.show(); // This sends the updated pixel color to the hardware.
  delay(delayval); // Delay for a period of time (in milliseconds)


}
