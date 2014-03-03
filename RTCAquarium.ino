#include <ledChannel.h>
#include <rtc_clock.h>
#include <DigiFi.h>
#include <genieArduino.h>
#include <simpleTimer.h>

#define AVERAGEVALUE 5
#define TIMEOUT 10000
#define SENSOR_READ 10000
#define CPU_READ 3000
#define TIME_READ 500
#define LED_READ 15000
#define CHANNELS 16

LEDChannel ledArray[CHANNELS] = {LEDChannel(1), LEDChannel(2), LEDChannel(3), LEDChannel(4), LEDChannel(5), LEDChannel(6),
                                 LEDChannel(7), LEDChannel(8), LEDChannel(9), LEDChannel(10), LEDChannel(11), LEDChannel(12), 
                                 LEDChannel(13), LEDChannel(14), LEDChannel(15), LEDChannel(16)};
DigiFi client;
SimpleTimer timer;

RTC_clock rtc_clock(XTAL);
char* dayNames[] = {"", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday", "Sunday"};
char* monthNames[] = {"January", "February", "March", "April", "May", "June", "July", "August", "Septembser", "October", "November", "December"};
int hh, mm, ss, dow, dd, mon, yyyy, currentMM;
char timeServer[] = "time.nist.gov"; // time.nist.gov NTP server
const int NTP_PACKET_SIZE = 48; // NTP time stamp is in the first 48 bytes of the message
uint8_t packetBuffer[NTP_PACKET_SIZE]; // Buffer to hold incoming and outgoing packets
int timeZone = -18; // GMT to Local Time Zone
bool screenSaver = true;
int pos = 0;

float cpuAverage[AVERAGEVALUE];
float orpAverage[AVERAGEVALUE];
float phAverage[AVERAGEVALUE];
float salinityAverage[AVERAGEVALUE];
float sumpTempAverage[AVERAGEVALUE];
float heatsinkTempAverage[AVERAGEVALUE];
float tankTempAverage[AVERAGEVALUE];

uint32_t pwmPinArray[CHANNELS] = {2, 3, 4, 5, 6, 7, 8, 9, 2, 3, 4, 5, 6, 7, 8, 9};
uint32_t startTimeArray[CHANNELS] = {775, 790, 600, 600, 600, 800, 825, 815, 820, 600, 600, 600, 600, 600, 600, 600};
uint32_t fadeTimeArray[CHANNELS] = {60, 60, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120, 120};
uint32_t totalTimeArray[CHANNELS] = {240, 240, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600, 600};
uint32_t intensityArray[CHANNELS] = {100, 100, 100, 40, 40, 70, 90, 100, 100, 100, 100, 40, 40, 30, 100, 100};

//****************************************************
// Time and String Manipulation                      *
//****************************************************

String hour;
String minute;
String second;
String colon = String(":");
String zero = String("0");
String space = String(" ");
String comma = String(",");

char buf[30] = { "" };

void setup()
{
  genieBegin(GENIE_SERIAL_0, 115200); // Digix Serial0
  genieAttachEventHandler(myGenieEventHandler);

  //Reset the Display (change D4 to D2 if you have original 4D Arduino Adaptor)
  pinMode(4, OUTPUT);  // Set D4 on Arduino to Output (4D Arduino Adaptor V2 - Display Reset)
  digitalWrite(4, 1);  // Reset the Display via D4
  delay(100);
  digitalWrite(4, 0);  // unReset the Display via D4
  delay(3500); //let the display start up

  genieWriteContrast(15); // 1 = Display ON, 0 = Display OFF
  genieWriteStr(0, "Initializing Wi-Fi");
  delay(500);
  client.begin(115200); //Wifi Connect

  rtc_clock.init();
  client.setMode(UDP); //must come before connect

  genieWriteStr(0, "Initializing RTC via NTP");
  delay(500);
  client.connect(timeServer, 123);

  unsigned long ntpUnixTime = 0;

  while (ntpUnixTime == 0)
  {
    genieWriteStr(0, "Sending packet");
    sendNTPpacket(); // send an NTP packet to a time server
    delay(2000);
    genieWriteStr(0, "Reading packet");
    ntpUnixTime = getNTPpacket();
  }

  rtc_clock.set_timestamp(ntpUnixTime);

  checkTimeZone();
  printTime();
  printDate();
  cpuTemp(); 
  setupLED();

  timer.setInterval(TIME_READ, timeLord);
  timer.setInterval(CPU_READ, cpuTemp);
  timer.setInterval(TIMEOUT, checkTimeout);
  timer.setInterval(LED_READ, checkLED);
}

void loop()
{
  genieDoEvents();
  timer.run();
}

void myGenieEventHandler(void)
{
  bool screenSaver = false;
  timer.restartTimer(2);

  genieFrame Event;
  genieDequeueEvent(&Event);

  if (genieEventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_FORM, 0)) // Returning to the home screen reprint time/date.
  {
    cpuTemp();
    printDate();
    printTime();
  }

  if (genieEventIs(&Event, GENIE_REPORT_EVENT, GENIE_OBJ_FORM, 1))
  {
  }

  genieWriteContrast(15);
}

void timeLord()
{
  currentMM = mm;
  rtc_clock.get_time(&hh, &mm, &ss);

  if (currentMM != mm) //Prevent redraw on every timer interval
  {
    printTime();
  }

  if (hh == 0 && mm < 1 && ss < 5) //Print date at midnight
  {
    rtc_clock.get_date(&dow, &dd, &mon, &yyyy);
    printDate();
    updateNTP();
  }
}

void setupLED()
{  
  for (int i = 0; i < CHANNELS; i++)
  {
    ledArray[i].setLEDChannel(i, pwmPinArray[i], startTimeArray[i], fadeTimeArray[i], totalTimeArray[i], intensityArray[i]);
  }
}

void checkLED()
{  
  rtc_clock.get_time(&hh, &mm, &ss);
  int minutes = (hh * 60) + mm;
  
  int test = ledArray[5].pwm(minutes);
  int test2 = ledArray[6].pwm(minutes);
}

void updateNTP()
{

}

void checkTimeout()
{
  if (screenSaver = true)
  {
    genieWriteContrast(1);
  }

  screenSaver = true;
}

unsigned long sendNTPpacket()
{
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12] = 49;
  packetBuffer[13] = 0x4E;
  packetBuffer[14] = 49;
  packetBuffer[15] = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  client.write(packetBuffer, NTP_PACKET_SIZE);
}

unsigned long getNTPpacket() {
  if (client.available())
  {
    // We've received a packet, read the data from it
    client.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer

    //the timestamp starts at byte 40 of the received packet and is four bytes,
    // or two words, long. First, esxtract the two words:

    unsigned long highWord = word(packetBuffer[40], packetBuffer[41]);
    unsigned long lowWord = word(packetBuffer[42], packetBuffer[43]);
    // combine the four bytes (two words) into a long integer
    // this is NTP time (seconds since Jan 1 1900):
    unsigned long secsSince1900 = highWord << 16 | lowWord;
    //Serial.print("Seconds since Jan 1 1900 = " );
    //Serial.println(secsSince1900);

    // now convert NTP time into everyday time:
    //Serial.print("Unix time = ");
    // Unix time starts on Jan 1 1970. In seconds, that's 2208988800:
    const unsigned long seventyYears = 2208988800UL;
    // subtract seventy years:
    unsigned long epoch = secsSince1900 - seventyYears;
    // print Unix time:
    return epoch;
  }
  else
  {
    return 0;
  }
}

void printTime()
{
  String temp;
  if (hh < 10)
  {
    hour = String(zero + hh);
  }
  else
  {
    hour = String(hh);
  }

  if (mm < 10)
  {
    minute = String(zero + mm);
  }
  else
  {
    minute = String(mm);
  }

  temp = String(hour + colon + minute);
  temp.toCharArray(buf, 10);

  genieWriteStr(1, buf);
}

void printDate()
{
  String temp;
  
  if (dd > 31)
  {
     dd = 0;
     rtc_clock.get_date(&dow, &dd, &mon, &yyyy);
  }
  
  temp = String(dayNames[rtc_clock.get_day_of_week()] + comma + space + monthNames[mon - 1] + space + dd + comma + space + yyyy);
  temp.toCharArray(buf, 30);

  genieWriteStr(0, buf);
}

void checkTimeZone()
{
  rtc_clock.get_time(&hh, &mm, &ss);
  rtc_clock.get_date(&dow, &dd, &mon, &yyyy);
  uint16_t year = yyyy;
  
  hh = hh + timeZone;
  if (hh < 0)
  {
    hh = hh + 24;
    dd = dd - 1;
    dow = dow - 1;
    
    if (dd < 1)
    {
      mon = mon - 1;
      if (mon < 0)
      {
        mon = 12;
        yyyy = yyyy - 1;
      }
    }
  }
  rtc_clock.set_time(hh, mm, ss);
  rtc_clock.set_date(dd, mon, yyyy);
}

void cpuTemp()
{
  String temp;
  float currTemp = 0;

  adc_enable_ts(ADC);
  adc_enable_channel( ADC, adc_channel_num_t(15) );
  adc_start(ADC);
  while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY);
  currTemp = (0.304025390625 * adc_get_latest_value(ADC)) - 274, 89;

  temp = String(takeAverage(currTemp, pos, cpuAverage));
  temp.toCharArray(buf, 5);
  genieWriteStr(5, buf);

  if (pos > 4)
  {
    pos = 0;
  }
  else
  {
    pos++;
  }
}

float takeAverage(int value, int index, float average[])
{
  float holder = 0.0;
  String test = "";
  int division = AVERAGEVALUE;

  if (value < 0)
  {
    average[index] = 0.0;
  }
  else
  {
    average[index] = value;
  }

  for (int i = 0; i < AVERAGEVALUE; i++)
  {
    if (average[i] == 0.0 && division > 1)
    {
      division--;
    }

    holder += average[i];
  }

  holder = holder / division;

  return holder;
}
