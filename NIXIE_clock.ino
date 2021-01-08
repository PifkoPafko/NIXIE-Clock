 ///////////// BIBLIOTEKI ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <TinyGPS++.h>                 //////// obsluga GPS
#include <SoftwareSerial.h>           //////// programowa obsluga UART
#include <EEPROM.h>                  //////// obsluga pamieci EEPROM
#include <PCF8574.h>                //////// ekspander wyprowadzen
#include <Wire.h>                  //////// ekspander wyprowadzen - PCF8574.h korzysta z tej biblioteki
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// STAŁE ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EEPROM_SIZE 512                     //////// rozmiar pamieci EEPROM
#define PCF8574_ADDR (0x20)                //////// adres I ekspandera wyjsc
#define PCF8574_ADDR2 (0x21)              //////// adres II ekspandera wyjsc
#define CZAS_ZAPALENIA (60LU)            //////// czas palenia sie lampy w mikrosekundach
#define CZAS_WYGASZENIA (800LU)         //////// czas wygaszenia lampy w mikrosekundach
#define CZAS_PRZYCISK (100LU)          //////// czas czekania az przestana drgac styki podczas wciskania przyciskow
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// KONFIGURACJA /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////// KATODY-CYFRY /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int K0 = 19;     
static const int K1 = 17;
static const int K2 = 4;
static const int K3 = 2;
static const int K4 = 23;
static const int K5 = 18;
static const int K6 = 15;
static const int K7 = 0;
static const int K8 = 16;
static const int K9 = 5;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// KATODY-PRZECINKI ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int PRL = 32;     
static const int PRP = 33;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// KREJESTR PRZESUWNY //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int  SER = 25;   
static const int  RCLK = 26;
static const int  SRCLK = 27;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// PRZYCISKI ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int buttonPin = 39;      
const int button2Pin = 36;
const int button3Pin = 34;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// PRZYCISKI ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int RXPin_GPS = 3, TXPin_GPS = 1;      ////// UART do GPS'a
static const int RXPin_mp3 = 14, TXPin_mp3 = 12;   ////// UART do odtwarzacza MP3
static const uint32_t GPSBaud = 9600;             ////// Baudrate komunikacji z GPS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// DIODA /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
const int dioda = 13;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////OBIEKTY/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
PCF8574 expander(PCF8574_ADDR);                     ///////// ekspander wyprowadzeń I
PCF8574 expander2(PCF8574_ADDR2);                  ///////// ekspander wyprowadzeń II

TinyGPSPlus gps;                                 ///////// moduł GPS
SoftwareSerial ss(RXPin_GPS, TXPin_GPS);        ///////// komunikacja UART z modułem GPS
SoftwareSerial mp3(RXPin_mp3, TXPin_mp3);      ///////// komunikacja UART z modułem MP3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////////DEFINICJA STRUKTUR////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////// STAN CZASU - LICZBY ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct czas {                 
  int8_t godzina=0;
  int8_t minuta=0;
  int8_t sekunda=0;
  int16_t milisekunda=0;
  int8_t dzien=0;
  int8_t miesiac=0;
  int16_t rok=0;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////// STAN CZASU - CYFRY /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct czas_cyfry {                 
  int8_t godzina_dziesiatki=0;
  int8_t godzina_jednosci=0;
  int8_t minuta_dziesiatki=0;
  int8_t minuta_jednosci=0;
  int8_t sekunda_dziesiatki=0;
  int8_t sekunda_jednosci=0;
  int8_t setnasekunda_dziesiatki=0;
  int8_t setnasekunda_jednosci=0;
  int8_t dzien_dziesiatki=0;
  int8_t dzien_jednosci=0;
  int8_t miesiac_dziesiatki=0;
  int8_t miesiac_jednosci=0;
  int8_t rok_tysiace=0;
  int8_t rok_setki=0;
  int8_t rok_dziesiatki=0;
  int8_t rok_jednosci=0;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////// STAN MINUTNIKA - LICZBY ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct minutnik_str {           
  int8_t godzina=0;
  int8_t minuta=0;
  int8_t sekunda=0;
  int16_t milisekunda=0;
  
  int8_t piosenka = 1;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////// STAN MINUTNIKA - CYFRY /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct minutnik_cyfry_str{        
  int8_t godzina_dziesiatki=0;
  int8_t godzina_jednosci=0;
  int8_t minuta_dziesiatki=0;
  int8_t minuta_jednosci=0;
  int8_t sekunda_dziesiatki=0;
  int8_t sekunda_jednosci=0;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////// ZAPISANE BUDZIKI - CYFRY ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct budzik {                 
  int8_t godzina_dziesiatki=0;
  int8_t godzina_jednosci=0;
  int8_t minuta_dziesiatki=0;
  int8_t minuta_jednosci=0;
  int8_t dzien_dziesiatki=0;
  int8_t dzien_jednosci=0;
  int8_t miesiac_dziesiatki=0;
  int8_t miesiac_jednosci=0;
  int8_t rok_dziesiatki=0;
  int8_t rok_jednosci=0;
  int8_t gps=0;
  int8_t tryb=0;
  uint8_t piosenka = 1;
  int8_t gotowy = 1;
};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////ZMIENNE//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t tryb=0;                                           /////////  aktualnie wyświetlany tryb
int8_t gps_on_czas = 0;                                  ///////// dostępnosc syngalu GPS

uint8_t lampy[16];                                     ///////// tablica zawierające aktualnie wyswietlane cyfry
int8_t katody[10];                                    ///////// tablica zawierajaca informacje czy katody sa zwarte do masy - cyfry
int8_t lewy_przecinek[16], prawy_przecinek[16];      ///////// tablica zawierajaca informacje czy katody sa zwarte do masy - przecinki

uint32_t nixie_micros = 0;                         ///////// ilosc mikrosekund, które upłyneły do aktualnego obiegu pętli - zapalanie lamp NIXIE
uint8_t wybrana_nixie = 0;                        ///////// numer aktaulnie zapalanej lampy
uint8_t nixie_stan = 0;                          ///////// stan automatu obslugujacego zapalanie lamp
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// PRZYCISKI ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t przelacznik_obrotowy[12];                    //////// stan wejsc przelacznika obrotowego
uint32_t przelacznik_obrotowy_millis = 0;           //////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - przelacznik obrotowy
uint32_t wcz_przelacznik_obrotowy_millis = 0;      //////// ilosc milisekund, które upłyneły do poprzedniego obiegu pętli - przelacznik obrotowy
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// PRZYCISKI ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t przycisk1_stan = 0;                 ///////// stan przycisku BTN1
uint8_t przycisk2_stan = 0;                ///////// stan przycisku BTN2
uint8_t przycisk3_stan = 0;               ///////// stan przycisku BTN3
uint32_t przycisk1_milis;                ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - przycisk BTN1
uint32_t przycisk2_milis;               ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - przycisk BTN2
uint32_t przycisk3_milis;              ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - przycisk BTN3
uint8_t p1=0, p2=0, p3=0, p4=0;       ///////// zmienna pomocnicza do programowego wyeliminowania problemu drgania stykow
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////// TRYB - 0/1/2 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct czas czas_GPS;                       ///////// stan czasu z synchronizacją GPS - liczby
struct czas_cyfry czas_cyfry_GPS;          ///////// stan czasu z synchronizacją GPS - cyfry
int8_t czas_GPS_sekunda_wcz = 0;          ///////// stan zmiennej przechowującej sekundę czasu w poprzednim obiegu głównej pętli
int8_t UTC_offset = 2;                   /////////
int8_t UTC_EEPROM = 14;                 /////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 3 //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct budzik budziki[10];                 ///////// zapisane budziki
struct budzik budziki_pom;                ///////// zmienna pomocnicza, zapisująca stan budzików przed wejsciem w sekwencje ustawiania
int8_t wybrany_budzik=0;                 ///////// aktualnie wyświetlany budzik
int8_t budzik_stan=0;                   ///////// stan automatu - ustawianie budzika
uint8_t budzik_odtwarzanie = 0;        ///////// pokazuje czy aktualnie trwa odtwarzanie alarmu budzika
uint32_t budzik_millis;               ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - budzik
uint8_t ktory_odtwarzany = 0;        ///////// numer budzika, ktorego alarm jest odtwarzany
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 4 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct czas pomiary[10];                         ///////// tablica przechowująca pomiary wykonane w trakcie odmierzania stopera
struct czas stoper;                             ///////// stan czasu stopera - liczby
struct czas_cyfry stoper_cyfry;                ///////// stan czasu stopera - cyfry
int8_t wybrany_pomiar=0;                      ///////// aktualnie wyświetlany pomiar (0 - wyświetlanie czasu stopera)
uint8_t ilosc_pomiarow=0;                    ///////// ilosc wykonanych pomiarów
uint32_t stoper_millis = 0;                 ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - stoper
uint32_t wcz_stoper_millis = 0;            ///////// ilosc miliskund, ktore uplynely do poprzedniego obiegu głównej pętli - stoper
uint8_t stoper_start=0;                   ///////// stan stopera (start/stop)
uint32_t roznica_czasu_stoper = 0;       ///////// czas jaki upłynął miedzy pętlami - stoper
uint8_t godzina_stoper_pom = 0;         ///////// zmienna pomocnicza do obliczenia godzin w stoperze
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 5 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct minutnik_str minutnik;                      //////// stan minutnika - liczny
struct minutnik_cyfry_str minutnik_cyfry;         //////// stan minutnika - cyfry
int8_t minutnik_stan=0;                          //////// stan automatu - ustawianie minutnika
uint32_t minutnik_millis = 0;                   //////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - minutnik
uint32_t wcz_minutnik_millis = 0;              //////// ilosc milisekund, które upłyneły do poprzedniego obiegu pętli - minutnik
uint8_t minutnik_start=0;                     //////// stan minutnika (start/stop)
uint8_t minutnik_odtwarzanie = 0;            //////// pokazuje czy aktualnie trwa odtwarzanie alarmu minutnika
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 6 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int8_t budziki_wybor_piosenek_stan = 0;         //////// stan automatu wyboru alarmow budzikow
int8_t budziki_wybor_piosenek_wybrany = 0;     //////// numer budzika, ktorego alarm jest aktualnie ustawiany
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 7 /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t piosenka = 1;                        ///////// numer piosenki do odtworzenia w trybie 7
uint8_t wybor_piosenek_odtwarzanie = 0;     ///////// 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////// TRYB - 8/9/10 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct czas czas_BRAK_GPS, czas_BRAK_GPS_pom;                       /////////
struct czas_cyfry czas_cyfry_BRAK_GPS, czas_cyfry_BRAK_GPS_pom;    /////////
uint32_t czas_millis = 0;                                         ///////// ilosc milisekund, które upłyneły do aktualnego obiegu pętli - czas ręcznie ustawiony
uint32_t roznica_czasu_glowna_petla = 0;                         ///////// czas jaki upłynął miedzy pętlami - czas ręcznie ustawiony
uint32_t wcz_czas_millis = 0;                                   ///////// ilosc miliskund, ktore uplynely do poprzedniego obiegu głównej pętli - czas ręcznie ustawiony
uint8_t czas_BRAK_GPS_stan = 0;                                ///////// stan automatu - ręczne ustawianie czasu
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////// MRUGANIE LAMP PODCZAS USTAWIANIA /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t mruganie = 0;
uint32_t mruganie_millis = 0;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup()
{
//////// ROZPOCZECIE KOMUNIKACJI ZE WSZYSTKIMI PODZESPOLAMI //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  Serial.begin(115200);
  Wire.begin();
  EEPROM.begin(EEPROM_SIZE);
  ss.begin(GPSBaud);
  mp3.begin(9600);
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  

///////// USTAWIENIE WEJSC/WYJSC ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  pinMode(SER, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SRCLK, OUTPUT);

  pinMode(K0, OUTPUT);
  pinMode(K1, OUTPUT);
  pinMode(K2, OUTPUT);
  pinMode(K3, OUTPUT);
  pinMode(K4, OUTPUT);
  pinMode(K5, OUTPUT);
  pinMode(K6, OUTPUT);
  pinMode(K7, OUTPUT);
  pinMode(K8, OUTPUT);
  pinMode(K9, OUTPUT);
  pinMode(PRP, OUTPUT);
  pinMode(PRL, OUTPUT);

  pinMode(dioda, OUTPUT);

  digitalWrite(K0, LOW);
  digitalWrite(K1, LOW);
  digitalWrite(K2, LOW);
  digitalWrite(K3, LOW);
  digitalWrite(K4, LOW);
  digitalWrite(K5, LOW);
  digitalWrite(K6, LOW);
  digitalWrite(K7, LOW);
  digitalWrite(K8, LOW);
  digitalWrite(K9, LOW);
  digitalWrite(PRP, LOW);
  digitalWrite(PRL, LOW);

  pinMode(buttonPin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);

  for(int i=0; i<8; i++)
  {
    expander.pinMode(i, INPUT);
    expander2.pinMode(i, INPUT);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////// ODCZYTANIE WARTOSCI Z PAMIECI EEPROM - STREFA CZASOWA I BUDZIKI /////////////////////////////////////////////////////////////////////////////////////////////////
  UTC_EEPROM = EEPROM.read(0);
  UTC_offset = UTC_EEPROM - 12;

  for(int i=0; i<10; i++)
  {
    budziki[i].godzina_dziesiatki = EEPROM.read(1 + 13*i);
    budziki[i].godzina_jednosci = EEPROM.read(2 + 13*i);
    budziki[i].minuta_dziesiatki = EEPROM.read(3 + 13*i);
    budziki[i].minuta_jednosci = EEPROM.read(4 + 13*i);
    budziki[i].dzien_dziesiatki = EEPROM.read(5 + 13*i);
    budziki[i].dzien_jednosci = EEPROM.read(6 + 13*i);
    budziki[i].miesiac_dziesiatki = EEPROM.read(7 + 13*i);
    budziki[i].miesiac_jednosci = EEPROM.read(8 + 13*i);
    budziki[i].rok_dziesiatki = EEPROM.read(9 + 13*i);
    budziki[i].rok_jednosci = EEPROM.read(10 + 13*i);
    budziki[i].gps = EEPROM.read(11 + 13*i);
    budziki[i].tryb = EEPROM.read(12 + 13*i);
    budziki[i].piosenka = EEPROM.readUChar(13 + 13*i);
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////// POCZATKOWE WARTOSCI TABLIC /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  for(int i=0; i<16; i++)
  {
    lampy[i]=10;
  }

  for(int i=0; i<10; i++)
  {
    katody[i]=0;
  }

  
  for(int i=0; i<12; i++)
  {
    przelacznik_obrotowy[i] = 1;
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


///////////// POCZATKOWA WARTOSC CZASU RECZNIE USTAWIANEGO I PO ZANIKU ZASILANIA //////////////////////////////////////////////////////////////////////////////////////////////
  czas_BRAK_GPS.godzina = 12;
  czas_BRAK_GPS.minuta = 0;
  czas_BRAK_GPS.sekunda = 0;
  czas_BRAK_GPS.milisekunda = 0;
  czas_BRAK_GPS.dzien = 1;
  czas_BRAK_GPS.miesiac = 1;
  czas_BRAK_GPS.rok = 2020;
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////// OPOZNIENIE I MELODIA STARTOWA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  delay(500);
  budzik_odtworz(5, 3);
  sekwencja_startowa();
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

void loop()
{
  przelacznik_obrotowy_fun();       
  
  przycisk1_fun();
  przycisk2_fun();
  przycisk3_fun();
  
  gps_dostepnosc();

  datagodzina();
  datagodzina_brak_gps();
  
  tryby();

  budzik_fun();
  minutnik_fun();
  
  nixie();
}


/////////////////// OBSLUGA PRZELACZNIKA OBROTOWEGO /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void przelacznik_obrotowy_fun()
{
  if(millis() - przelacznik_obrotowy_millis > 10LU)
  {
    przelacznik_obrotowy[0] = expander.digitalRead(P4);
    przelacznik_obrotowy[1] = expander.digitalRead(P3);
    przelacznik_obrotowy[2] = expander.digitalRead(P5);
    przelacznik_obrotowy[3] = expander.digitalRead(P2);
    przelacznik_obrotowy[4] = expander.digitalRead(P6);
    przelacznik_obrotowy[5] = expander.digitalRead(P1);
    przelacznik_obrotowy[6] = expander.digitalRead(P7);
    przelacznik_obrotowy[7] = expander.digitalRead(P0);
    przelacznik_obrotowy[8] = expander2.digitalRead(P4);
    przelacznik_obrotowy[9] = expander2.digitalRead(P3);
    przelacznik_obrotowy[10] = expander2.digitalRead(P5);
    
    if(tryb==6 && przelacznik_obrotowy[6] == HIGH)
    {
      EEPROM.commit();
    }
    
    for(int i=0; i<11; i++)
    {
      if(przelacznik_obrotowy[i] == 0) 
      {
        tryb = i;
      }
    }
    przelacznik_obrotowy_millis = millis();
  }
  
  if(budzik_odtwarzanie)
  {
    if(budziki[ktory_odtwarzany].gps && gps_on_czas)
    {
      tryb = 0;
    }
    else
    {
      tryb = 10;
    }
    
  }
  if(minutnik_odtwarzanie) tryb = 5;
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// OBSLUGA PRZYCISKU BTN1 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void przycisk1_fun()
{
  przycisk1_stan = digitalRead(buttonPin);
  if (przycisk1_stan == LOW) 
  {
    
    if(p1==0 && (millis() - przycisk1_milis > CZAS_PRZYCISK))
    {
      p1=1;
      przycisk1_milis = millis();

      if(tryb == 0 || tryb == 1 || tryb == 2)
      {
        if(budzik_odtwarzanie)
        {
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          UTC_offset++;
          if(UTC_offset > 14) UTC_offset = -12;
        } 
      }
      else if(tryb == 3)
      {
        switch(budzik_stan)
        {
          case 0:
            wybrany_budzik++;
            if(wybrany_budzik > 9)
            {
              wybrany_budzik = 0;
            }
            break;

          case 1:
            budziki_pom.tryb++;
            if( budziki_pom.tryb > 3 )
            {
              budziki_pom.tryb = 0;
            }
            break;

          case 2:
            budziki_pom.gps++;
            if( budziki_pom.gps > 1 )
            {
              budziki_pom.gps = 0;
            }
            break;

          case 3:
            budziki_pom.godzina_dziesiatki++;
            if( budziki_pom.godzina_dziesiatki > 2 )
            {
              budziki_pom.godzina_dziesiatki = 0;
            }
            break;

          case 4:
            budziki_pom.godzina_jednosci++;
            if( budziki_pom.godzina_jednosci > 9 )
            {
              budziki_pom.godzina_jednosci = 0;
            }
            break;

          case 5:
            budziki_pom.minuta_dziesiatki++;
            if( budziki_pom.minuta_dziesiatki > 5 )
            {
              budziki_pom.minuta_dziesiatki = 0;
            }
            break;

          case 6:
            budziki_pom.minuta_jednosci++;
            if( budziki_pom.minuta_jednosci > 9 )
            {
              budziki_pom.minuta_jednosci = 0;
            }
            break;

          case 7:
            budziki_pom.dzien_dziesiatki++;
            if( budziki_pom.dzien_dziesiatki > 3 )
            {
              budziki_pom.dzien_dziesiatki = 0;
            }
            break;

          case 8:
            budziki_pom.dzien_jednosci++;
            if( budziki_pom.dzien_jednosci > 9 )
            {
              budziki_pom.dzien_jednosci = 0;
            }
            break;
            
          case 9:
            budziki_pom.miesiac_dziesiatki++;
            if( budziki_pom.miesiac_dziesiatki > 1 )
            {
              budziki_pom.miesiac_dziesiatki = 0;
            }
            break;
            
          case 10:
            budziki_pom.miesiac_jednosci++;
            if( budziki_pom.miesiac_jednosci > 9 )
            {
              budziki_pom.miesiac_jednosci = 0;
            }
            break;
            
          case 11:
            budziki_pom.rok_dziesiatki++;
            if( budziki_pom.rok_dziesiatki > 9 )
            {
              budziki_pom.rok_dziesiatki = 0;
            }
            break;
            
          case 12:
            budziki_pom.rok_jednosci++;
            if( budziki_pom.rok_jednosci > 9 )
            {
              budziki_pom.rok_jednosci = 0;
            }
            break;
        }
      }
      else if(tryb == 4)
      {
        if(stoper_start == 0)
        {
          stoper_start = 1;
          wcz_stoper_millis = millis();
          wybrany_pomiar=0;
        }
        else
        {
          wybrany_pomiar=0;
          stoper_start = 0;
        }
      }
      else if(tryb == 5)
      {
        if(minutnik_odtwarzanie)
        {
          minutnik_stan = 0;
          zatrzymaj_odtwarzanie();
          minutnik_odtwarzanie = 0;
        }
        else
        {
          switch(minutnik_stan)
          {
          case 0:
            if(minutnik_start == 0)
            {
              if(minutnik.godzina || minutnik.minuta || minutnik.sekunda || minutnik.milisekunda)
              {
                minutnik_start = 1;
                wcz_minutnik_millis = millis();
              }
            }
            else
            {
              minutnik_start = 0;
            }
            break;

          case 1:
            minutnik.godzina += 10;
            if( minutnik.godzina > 99 )
            {
              minutnik.godzina -= 100;
            }
            break;

          case 2:
            minutnik.godzina += 1;
            if(minutnik.godzina % 10 == 0)
            {
              minutnik.godzina -= 10;
            }
            break;

          case 3:
            minutnik.minuta += 10;
            if( minutnik.minuta > 59 )
            {
              minutnik.minuta -= 60;
            }
            break;

          case 4:
            minutnik.minuta += 1;
            if(minutnik.minuta % 10 == 0)
            {
              minutnik.minuta -= 10;
            }
            break;

          case 5:
            minutnik.sekunda += 10;
            if( minutnik.sekunda > 59 )
            {
              minutnik.sekunda -= 60;
            }
            break;

          case 6:
            minutnik.sekunda += 1;
            if(minutnik.sekunda % 10 == 0)
            {
              minutnik.sekunda -= 10;
            }
            break;

          case 7:
            minutnik.piosenka += 100;
            if(minutnik.piosenka > 999)
            {
              minutnik.piosenka -= 1000;
            }
            break;

          case 8:
            minutnik.piosenka += 10;
            if((minutnik.piosenka / 10) % 10 == 0)
            {
              minutnik.piosenka -= 100;
            }
            break;
            
          case 9:
            minutnik.piosenka += 1;
            if(minutnik.piosenka % 10 == 0)
            {
              minutnik.piosenka -= 10;
            }
            break;
          }
        }
      }
      else if(tryb == 6)
      {
        if(budziki_wybor_piosenek_stan) 
        {
          if(budziki[budziki_wybor_piosenek_wybrany].piosenka == 255)
          {
            budziki[budziki_wybor_piosenek_wybrany].piosenka = 1;
          }
          else
          {
             budziki[budziki_wybor_piosenek_wybrany].piosenka++;
          }
          EEPROM.writeUChar(13*budziki_wybor_piosenek_wybrany+13, budziki[budziki_wybor_piosenek_wybrany].piosenka);
          
        }
        else
        {
          budziki_wybor_piosenek_wybrany++;
          if(budziki_wybor_piosenek_wybrany > 9)
          {
            budziki_wybor_piosenek_wybrany = 0;
          }
        }
      }
      else if(tryb == 7)
      {
        zatrzymaj_odtwarzanie();
        wybor_piosenek_odtwarzanie = 0;
        if(piosenka==255) 
        {
          piosenka = 1;
        }
        else
        {
          piosenka++;
        }
      }
      else if(tryb == 10)
      {
        if(budzik_odtwarzanie)
        {
          czas_BRAK_GPS_stan = 0;
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          switch(czas_BRAK_GPS_stan)
        {
          case 0:
            break;
            
          case 1:
            czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki > 2)
            {
              czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki = 0;
            }
            break;
            
            

          case 2:
            czas_cyfry_BRAK_GPS_pom.godzina_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.godzina_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.godzina_jednosci = 0;
            }
            break;
            

          case 3:
            czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki > 5)
            {
              czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki = 0;
            }
            break;
            
            

          case 4:
            czas_cyfry_BRAK_GPS_pom.minuta_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.minuta_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.minuta_jednosci = 0;
            }
            break;
            
          case 5:
            czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki > 5)
            {
              czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki = 0;
            }
            break;
            

          case 6:
            czas_cyfry_BRAK_GPS_pom.sekunda_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.sekunda_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.sekunda_jednosci = 0;
            }
            break;
            
            

          case 7:
            czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki > 3)
            {
              czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki = 0;
            }
            break;

          case 8:
            czas_cyfry_BRAK_GPS_pom.dzien_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.dzien_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.dzien_jednosci = 0;
            }
            break;
            
          case 9:
            czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki > 1)
            {
              czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki = 0;
            }
            break; 
            
            

          case 10:
            czas_cyfry_BRAK_GPS_pom.miesiac_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.miesiac_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.miesiac_jednosci = 0;
            }
            break; 
            

          case 11:
            czas_cyfry_BRAK_GPS_pom.rok_tysiace++;
            if(czas_cyfry_BRAK_GPS_pom.rok_tysiace > 9)
            {
              czas_cyfry_BRAK_GPS_pom.rok_tysiace = 0;
            }
            
            break;
            
        
          case 12:
            czas_cyfry_BRAK_GPS_pom.rok_setki++;
            if(czas_cyfry_BRAK_GPS_pom.rok_setki > 9)
            {
              czas_cyfry_BRAK_GPS_pom.rok_setki = 0;
            }
                   
            break;
            

          case 13:
            czas_cyfry_BRAK_GPS_pom.rok_dziesiatki++;
            if(czas_cyfry_BRAK_GPS_pom.rok_dziesiatki > 9)
            {
              czas_cyfry_BRAK_GPS_pom.rok_dziesiatki = 0;
            }   
            break;
            

          case 14:
            czas_cyfry_BRAK_GPS_pom.rok_jednosci++;
            if(czas_cyfry_BRAK_GPS_pom.rok_jednosci > 9)
            {
              czas_cyfry_BRAK_GPS_pom.rok_jednosci = 0;
            }
            break;
            
          }
        } 
      }
    }
  }

  if(p1)
    if(millis() - przycisk1_milis >= CZAS_PRZYCISK)
      if(przycisk1_stan == HIGH)
      {
        p1=0;
        przycisk1_milis = millis();
      }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// OBSLUGA PRZYCISKU BTN2 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void przycisk2_fun()
{
  przycisk2_stan = digitalRead(button2Pin);
  if (przycisk2_stan == LOW) 
  {
    if(p2==0 && (millis() - przycisk2_milis > CZAS_PRZYCISK))
    {
      if(tryb == 0 || tryb == 1 || tryb == 2)
      {
        if(budzik_odtwarzanie)
        {
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          UTC_offset--;
          if(UTC_offset < -12) UTC_offset = 14;
        }
      }
      else if(tryb == 3)
      {
        switch(budzik_stan)
        {
          case 0:
            wybrany_budzik--;
            if(wybrany_budzik < 0)
            {
              wybrany_budzik = 9;
            }
            break;

          case 1:
            budziki_pom.tryb--;
            if( budziki_pom.tryb < 0 )
            {
              budziki_pom.tryb = 3;
            }
            break;

          case 2:
            budziki_pom.gps--;
            if( budziki_pom.gps < 0 )
            {
              budziki_pom.gps = 1;
            }
            break;

          case 3:
            budziki_pom.godzina_dziesiatki--;
            if( budziki_pom.godzina_dziesiatki < 0 )
            {
              budziki_pom.godzina_dziesiatki = 2;
            }
            break;

          case 4:
            budziki_pom.godzina_jednosci--;
            if( budziki_pom.godzina_jednosci < 0 )
            {
              budziki_pom.godzina_jednosci = 9;
            }
            break;

          case 5:
            budziki_pom.minuta_dziesiatki--;
            if( budziki_pom.minuta_dziesiatki < 0 )
            {
              budziki_pom.minuta_dziesiatki = 5;
            }
            break;

          case 6:
            budziki_pom.minuta_jednosci--;
            if( budziki_pom.minuta_jednosci < 0 )
            {
              budziki_pom.minuta_jednosci = 9;
            }
            break;

          case 7:
            budziki_pom.dzien_dziesiatki--;
            if( budziki_pom.dzien_dziesiatki < 0 )
            {
              budziki_pom.dzien_dziesiatki = 3;
            }
            break;

          case 8:
            budziki_pom.dzien_jednosci--;
            if( budziki_pom.dzien_jednosci < 0 )
            {
              budziki_pom.dzien_jednosci = 9;
            }
            break;
            
          case 9:
            budziki_pom.miesiac_dziesiatki--;
            if( budziki_pom.miesiac_dziesiatki < 0 )
            {
              budziki_pom.miesiac_dziesiatki = 1;
            }
            break;
            
          case 10:
            budziki_pom.miesiac_jednosci--;
            if( budziki_pom.miesiac_jednosci < 0 )
            {
              budziki_pom.miesiac_jednosci = 9;
            }
            break;
            
          case 11:
            budziki_pom.rok_dziesiatki--;
            if( budziki_pom.rok_dziesiatki < 0 )
            {
              budziki_pom.rok_dziesiatki = 9;
            }
            break;
            
          case 12:
            budziki_pom.rok_jednosci--;
            if( budziki_pom.rok_jednosci < 0 )
            {
              budziki_pom.rok_jednosci = 9;
            }
            break;
        }
      }
      else if(tryb == 4)
      {
        stoper_start = 0;
        
        stoper.godzina  = 0;
        stoper.minuta = 0;
        stoper.sekunda = 0;
        stoper.milisekunda = 0;
        
  
        for(int i=0; i< ilosc_pomiarow; i++)
        {
          pomiary[i].godzina=0;
          pomiary[i].minuta=0;
          pomiary[i].sekunda=0;
          pomiary[i].milisekunda=0;
        }
        wybrany_pomiar = 0;
        ilosc_pomiarow = 0;
      }
      else if(tryb == 5)
      {
        if(minutnik_odtwarzanie)
        {
          minutnik_stan = 0;
          zatrzymaj_odtwarzanie();
          minutnik_odtwarzanie = 0;
        }
        else
        {
          switch(minutnik_stan)
        {
          case 0:
            minutnik_start = 0;
            minutnik.godzina = 0;
            minutnik.minuta = 0;
            minutnik.sekunda = 0;
            minutnik.milisekunda = 0;
          break;

          case 1:
            minutnik.godzina -= 10;
            if( minutnik.godzina <0 )
            {
              minutnik.godzina += 100;
            }
            break;

          case 2:
            minutnik.godzina -= 1;
            if(minutnik.godzina % 10 == 9 || minutnik.godzina < 0)
            {
              minutnik.godzina += 10;
            }
            break;

          case 3:
            minutnik.minuta -= 10;
            if( minutnik.minuta < 0 )
            {
              minutnik.minuta += 60;
            }
            break;

          case 4:
            minutnik.minuta -= 1;
            if(minutnik.minuta % 10 == 9 || minutnik.minuta < 0)
            {
              minutnik.minuta += 10;
            }
            break;

          case 5:
            minutnik.sekunda -= 10;
            if( minutnik.sekunda < 0 )
            {
              minutnik.sekunda += 60;
            }
            break;

          case 6:
            minutnik.sekunda -= 1;
            if(minutnik.sekunda % 10 == 9 || minutnik.sekunda < 0)
            {
              minutnik.sekunda += 10;
            }
            break;

          case 7:
            minutnik.piosenka -= 100;
            if(minutnik.piosenka < 0)
            {
              minutnik.piosenka += 1000;
            }
            break;

          case 8:
            minutnik.piosenka -= 10;
            if((minutnik.piosenka / 10) % 10 == 9 || minutnik.piosenka < 0)
            {
              minutnik.piosenka += 100;
            }
            break;
            
          case 9:
            minutnik.piosenka -= 1;
            if(minutnik.piosenka % 10 == 9 || minutnik.piosenka < 0)
            {
              minutnik.piosenka += 10;
            }
            break;
          }
        }
      }
      else if(tryb == 6)
      {
        if(budziki_wybor_piosenek_stan) 
        {
          if(budziki[budziki_wybor_piosenek_wybrany].piosenka == 1)
          {
            budziki[budziki_wybor_piosenek_wybrany].piosenka = 255;
          }
          else
          {
            budziki[budziki_wybor_piosenek_wybrany].piosenka--;
          }
          
          EEPROM.writeUChar(13*budziki_wybor_piosenek_wybrany+13, budziki[budziki_wybor_piosenek_wybrany].piosenka);
        }
        else
        {
          budziki_wybor_piosenek_wybrany--;
          if(budziki_wybor_piosenek_wybrany < 0)
          {
            budziki_wybor_piosenek_wybrany = 9;
          }
        }
      }
      else if(tryb == 7)
      {
        zatrzymaj_odtwarzanie();
        wybor_piosenek_odtwarzanie = 0;
        if(piosenka==1) 
        {
          piosenka = 255;
        }
        else
        {
          piosenka--;
        }
      }
      else if(tryb == 10)
      {
        if(budzik_odtwarzanie)
        {
          czas_BRAK_GPS_stan = 0;
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          switch(czas_BRAK_GPS_stan)
          {
            case 0:
              break;
              
            case 1:
              czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki = 2;
              }
              break; 
  
            case 2:
              czas_cyfry_BRAK_GPS_pom.godzina_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.godzina_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.godzina_jednosci = 9;
              }
              break;
  
            case 3:
              czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki = 5;
              }
              break;
              
            case 4:
              czas_cyfry_BRAK_GPS_pom.minuta_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.minuta_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.minuta_jednosci = 9;
              }
              break;
              
            case 5:
              czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki = 5;
              }
              break;       
  
            case 6:
              czas_cyfry_BRAK_GPS_pom.sekunda_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.sekunda_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.sekunda_jednosci = 9;
              }
              break;
  
            case 7:
              czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki = 3;
              }
              break;
  
            case 8:
              czas_cyfry_BRAK_GPS_pom.dzien_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.dzien_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.dzien_jednosci = 9;
              }
              break;
              
            case 9:
              czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki = 1;
              }
              break;
  
            case 10:
              czas_cyfry_BRAK_GPS_pom.miesiac_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.miesiac_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.miesiac_jednosci = 9;
              }
              break;
  
            case 11:
              czas_cyfry_BRAK_GPS_pom.rok_tysiace--;
              if(czas_cyfry_BRAK_GPS_pom.rok_tysiace < 0)
              {
                czas_cyfry_BRAK_GPS_pom.rok_tysiace = 9;
              } 
              break;
                
            case 12:
              czas_cyfry_BRAK_GPS_pom.rok_setki--;
              if(czas_cyfry_BRAK_GPS_pom.rok_setki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.rok_setki = 9;
              }     
              break;
  
            case 13:
              czas_cyfry_BRAK_GPS_pom.rok_dziesiatki--;
              if(czas_cyfry_BRAK_GPS_pom.rok_dziesiatki < 0)
              {
                czas_cyfry_BRAK_GPS_pom.rok_dziesiatki = 9;
              }   
              break;      
  
            case 14:
              czas_cyfry_BRAK_GPS_pom.rok_jednosci--;
              if(czas_cyfry_BRAK_GPS_pom.rok_jednosci < 0)
              {
                czas_cyfry_BRAK_GPS_pom.rok_jednosci = 9;
              }
              break; 
          }
        }
      }

      p2=1;
      przycisk2_milis = millis();
    }
  } 

  if(p2)
    if(millis() - przycisk2_milis >= CZAS_PRZYCISK)
      if(przycisk2_stan == HIGH)
      {
        p2=0;
        przycisk2_milis = millis();
      }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// OBSLUGA PRZYCISKU BTN3 ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void przycisk3_fun()
{
  przycisk3_stan = digitalRead(button3Pin);
  if (przycisk3_stan == LOW) 
  {
    if(p3==0 && (millis() - przycisk3_milis > CZAS_PRZYCISK))    
    {
      p3=1;
      przycisk3_milis = millis();

      if(tryb==0||tryb==1||tryb==2) 
      {
        if(budzik_odtwarzanie)
        {
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          UTC_EEPROM = UTC_offset + 12;       ///////// ZAPISANIE STREFY CZASOWEJ DO EEPROM ///////////
          EEPROM.write(0, UTC_EEPROM);
          EEPROM.commit(); 
        } 
      }
      else if(tryb == 3)
      {
        switch(budzik_stan)
        {
          case 0:
            budziki_pom.tryb = budziki[wybrany_budzik].tryb;
            budziki_pom.gps = budziki[wybrany_budzik].gps;
            budziki_pom.godzina_dziesiatki = budziki[wybrany_budzik].godzina_dziesiatki;
            budziki_pom.godzina_jednosci = budziki[wybrany_budzik].godzina_jednosci;
            budziki_pom.minuta_dziesiatki = budziki[wybrany_budzik].minuta_dziesiatki;
            budziki_pom.minuta_jednosci = budziki[wybrany_budzik].minuta_jednosci;
            budziki_pom.dzien_dziesiatki = budziki[wybrany_budzik].dzien_dziesiatki;
            budziki_pom.dzien_jednosci = budziki[wybrany_budzik].dzien_jednosci;
            budziki_pom.miesiac_dziesiatki = budziki[wybrany_budzik].miesiac_dziesiatki;
            budziki_pom.miesiac_jednosci = budziki[wybrany_budzik].miesiac_jednosci;
            budziki_pom.rok_dziesiatki = budziki[wybrany_budzik].rok_dziesiatki;
            budziki_pom.rok_jednosci = budziki[wybrany_budzik].rok_jednosci;     
            break;
            
          case 6:
            if(budziki[wybrany_budzik].tryb == 2) 
            {
              if(poprawnosc_czasu(0, budziki_pom.godzina_dziesiatki*10 + budziki_pom.godzina_jednosci, budziki_pom.minuta_dziesiatki*10 + budziki_pom.minuta_jednosci, 0,
              budziki_pom.dzien_dziesiatki * 10 + budziki_pom.dzien_jednosci, budziki_pom.miesiac_dziesiatki*10 + budziki_pom.miesiac_jednosci,
              2000 + budziki_pom.rok_dziesiatki*10 + budziki_pom.rok_jednosci))
              {
                zamiana_budzikow();
                
                EEPROM.write(wybrany_budzik*13+1, budziki[wybrany_budzik].godzina_dziesiatki);
                EEPROM.write(wybrany_budzik*13+2, budziki[wybrany_budzik].godzina_jednosci);
                EEPROM.write(wybrany_budzik*13+3, budziki[wybrany_budzik].minuta_dziesiatki);
                EEPROM.write(wybrany_budzik*13+4, budziki[wybrany_budzik].minuta_jednosci);

                EEPROM.write(wybrany_budzik*13+11, budziki[wybrany_budzik].gps);
                EEPROM.write(wybrany_budzik*13+12, budziki[wybrany_budzik].tryb);
                
                EEPROM.commit();
              }

              
              budzik_stan = -1;
            }
            break;
            
          case 10:
            if(budziki[wybrany_budzik].tryb == 3) 
            {
              if(poprawnosc_czasu(1, budziki_pom.godzina_dziesiatki*10 + budziki_pom.godzina_jednosci, budziki_pom.minuta_dziesiatki*10 + budziki_pom.minuta_jednosci, 0,
              budziki_pom.dzien_dziesiatki * 10 + budziki_pom.dzien_jednosci, budziki_pom.miesiac_dziesiatki*10 + budziki_pom.miesiac_jednosci,
              2000 + budziki_pom.rok_dziesiatki*10 + budziki_pom.rok_jednosci))
              {
                zamiana_budzikow();
                EEPROM.write(wybrany_budzik*13+1, budziki[wybrany_budzik].godzina_dziesiatki);
                EEPROM.write(wybrany_budzik*13+2, budziki[wybrany_budzik].godzina_jednosci);
                EEPROM.write(wybrany_budzik*13+3, budziki[wybrany_budzik].minuta_dziesiatki);
                EEPROM.write(wybrany_budzik*13+4, budziki[wybrany_budzik].minuta_jednosci);
                EEPROM.write(wybrany_budzik*13+5, budziki[wybrany_budzik].dzien_dziesiatki);
                EEPROM.write(wybrany_budzik*13+6, budziki[wybrany_budzik].dzien_jednosci);
                EEPROM.write(wybrany_budzik*13+7, budziki[wybrany_budzik].miesiac_dziesiatki);
                EEPROM.write(wybrany_budzik*13+8, budziki[wybrany_budzik].miesiac_jednosci);

                EEPROM.write(wybrany_budzik*13+11, budziki[wybrany_budzik].gps);
                EEPROM.write(wybrany_budzik*13+12, budziki[wybrany_budzik].tryb);
                
                EEPROM.commit();
              }
              budzik_stan = -1;
            }
            break;

          case 12:
              if(budziki[wybrany_budzik].tryb == 0 || budziki[wybrany_budzik].tryb == 1) 
              {
                if(poprawnosc_czasu(2, budziki_pom.godzina_dziesiatki*10 + budziki_pom.godzina_jednosci, budziki_pom.minuta_dziesiatki*10 + budziki_pom.minuta_jednosci, 0,
                budziki_pom.dzien_dziesiatki * 10 + budziki_pom.dzien_jednosci, budziki_pom.miesiac_dziesiatki*10 + budziki_pom.miesiac_jednosci,
                2000 + budziki_pom.rok_dziesiatki*10 + budziki_pom.rok_jednosci))
                {
                  zamiana_budzikow();
                  EEPROM.write(wybrany_budzik*13+1, budziki[wybrany_budzik].godzina_dziesiatki);
                  EEPROM.write(wybrany_budzik*13+2, budziki[wybrany_budzik].godzina_jednosci);
                  EEPROM.write(wybrany_budzik*13+3, budziki[wybrany_budzik].minuta_dziesiatki);
                  EEPROM.write(wybrany_budzik*13+4, budziki[wybrany_budzik].minuta_jednosci);
                  EEPROM.write(wybrany_budzik*13+5, budziki[wybrany_budzik].dzien_dziesiatki);
                  EEPROM.write(wybrany_budzik*13+6, budziki[wybrany_budzik].dzien_jednosci);
                  EEPROM.write(wybrany_budzik*13+7, budziki[wybrany_budzik].miesiac_dziesiatki);
                  EEPROM.write(wybrany_budzik*13+8, budziki[wybrany_budzik].miesiac_jednosci);
                  EEPROM.write(wybrany_budzik*13+9, budziki[wybrany_budzik].rok_dziesiatki);
                  EEPROM.write(wybrany_budzik*13+10, budziki[wybrany_budzik].rok_jednosci);
                  EEPROM.write(wybrany_budzik*13+11, budziki[wybrany_budzik].gps);
                  EEPROM.write(wybrany_budzik*13+12, budziki[wybrany_budzik].tryb);
                  
                  EEPROM.commit();
                }

                budzik_stan = -1;
              }
            break;
        }
        budzik_stan++;
      }
      else if(tryb==4)
      {
        if(stoper_start)
        {
          ilosc_pomiarow++;
          if(ilosc_pomiarow>9)
          {
            ilosc_pomiarow=9;
          }
          else
          {
            pomiary[ilosc_pomiarow-1].godzina = stoper.godzina;
            pomiary[ilosc_pomiarow-1].minuta = stoper.minuta;
            pomiary[ilosc_pomiarow-1].sekunda = stoper.sekunda;
            pomiary[ilosc_pomiarow-1].milisekunda = stoper.milisekunda;
          }
          
        }
        else
        {
          wybrany_pomiar++;
          if(wybrany_pomiar>ilosc_pomiarow) wybrany_pomiar = 0;
        }
      }
      else if(tryb == 5)
      {
        if(minutnik_odtwarzanie)
        {
          minutnik_stan = 0;
          zatrzymaj_odtwarzanie();
          minutnik_odtwarzanie = 0;
        }
        else if(minutnik_stan == 9)
        {
          minutnik_stan = 0;
        }
        else
        {
          minutnik_stan++;
        }
      }
      else if(tryb == 6)
      {
        budziki_wybor_piosenek_stan = !budziki_wybor_piosenek_stan;
      }
      else if(tryb == 7)
      {
        if(wybor_piosenek_odtwarzanie)
        {
          wybor_piosenek_odtwarzanie = 0;
          zatrzymaj_odtwarzanie();
        }
        else
        { 
          budzik_odtworz(0 , 2);
        }
      }
      else if(tryb == 10)
      {
        if(budzik_odtwarzanie)
        {
          czas_BRAK_GPS_stan = 0;
          zatrzymaj_odtwarzanie();
          budzik_odtwarzanie = 0;
        }
        else
        {
          switch(czas_BRAK_GPS_stan)
          {
            case 0:
              czas_cyfry_BRAK_GPS_pom.rok_tysiace = czas_cyfry_BRAK_GPS.rok_tysiace;
              czas_cyfry_BRAK_GPS_pom.rok_setki = czas_cyfry_BRAK_GPS.rok_setki;
              czas_cyfry_BRAK_GPS_pom.rok_dziesiatki = czas_cyfry_BRAK_GPS.rok_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.rok_jednosci = czas_cyfry_BRAK_GPS.rok_jednosci;
              czas_cyfry_BRAK_GPS_pom.miesiac_jednosci = czas_cyfry_BRAK_GPS.miesiac_jednosci;
              czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki = czas_cyfry_BRAK_GPS.miesiac_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.dzien_jednosci = czas_cyfry_BRAK_GPS.dzien_jednosci;
              czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki = czas_cyfry_BRAK_GPS.dzien_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.godzina_jednosci = czas_cyfry_BRAK_GPS.godzina_jednosci;
              czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki = czas_cyfry_BRAK_GPS.godzina_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.minuta_jednosci = czas_cyfry_BRAK_GPS.minuta_jednosci;
              czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki = czas_cyfry_BRAK_GPS.minuta_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.sekunda_jednosci = czas_cyfry_BRAK_GPS.sekunda_jednosci;
              czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki = czas_cyfry_BRAK_GPS.sekunda_dziesiatki;
              czas_cyfry_BRAK_GPS_pom.setnasekunda_jednosci = czas_cyfry_BRAK_GPS.setnasekunda_jednosci;
              czas_cyfry_BRAK_GPS_pom.setnasekunda_dziesiatki = czas_cyfry_BRAK_GPS.setnasekunda_dziesiatki;
              break;
  
            case 14:
              czas_BRAK_GPS_pom.rok = 1000*czas_cyfry_BRAK_GPS_pom.rok_tysiace + 100*czas_cyfry_BRAK_GPS_pom.rok_setki + 10*czas_cyfry_BRAK_GPS_pom.rok_dziesiatki + czas_cyfry_BRAK_GPS_pom.rok_jednosci; 
              czas_BRAK_GPS_pom.miesiac = 10*czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki + czas_cyfry_BRAK_GPS_pom.miesiac_jednosci;
              czas_BRAK_GPS_pom.dzien = 10*czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki + czas_cyfry_BRAK_GPS_pom.dzien_jednosci;
              czas_BRAK_GPS_pom.godzina = 10*czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki + czas_cyfry_BRAK_GPS_pom.godzina_jednosci;
              czas_BRAK_GPS_pom.sekunda = 10*czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki + czas_cyfry_BRAK_GPS_pom.sekunda_jednosci;
              czas_BRAK_GPS_pom.minuta = 10*czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki + czas_cyfry_BRAK_GPS_pom.minuta_jednosci;
              
  
              if(poprawnosc_czasu(2, czas_BRAK_GPS_pom.godzina, czas_BRAK_GPS_pom.minuta, czas_BRAK_GPS_pom.sekunda, czas_BRAK_GPS_pom.dzien, czas_BRAK_GPS_pom.miesiac, czas_BRAK_GPS_pom.rok))
              {
                czas_cyfry_BRAK_GPS.rok_tysiace = czas_cyfry_BRAK_GPS_pom.rok_tysiace;
                czas_cyfry_BRAK_GPS.rok_setki = czas_cyfry_BRAK_GPS_pom.rok_setki;
                czas_cyfry_BRAK_GPS.rok_dziesiatki = czas_cyfry_BRAK_GPS_pom.rok_dziesiatki;
                czas_cyfry_BRAK_GPS.rok_jednosci = czas_cyfry_BRAK_GPS_pom.rok_jednosci;
                czas_cyfry_BRAK_GPS.miesiac_jednosci = czas_cyfry_BRAK_GPS_pom.miesiac_jednosci;
                czas_cyfry_BRAK_GPS.miesiac_dziesiatki = czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki;
                czas_cyfry_BRAK_GPS.dzien_jednosci = czas_cyfry_BRAK_GPS_pom.dzien_jednosci;
                czas_cyfry_BRAK_GPS.dzien_dziesiatki = czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki;
                czas_cyfry_BRAK_GPS.godzina_jednosci = czas_cyfry_BRAK_GPS_pom.godzina_jednosci;
                czas_cyfry_BRAK_GPS.godzina_dziesiatki = czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki;
                czas_cyfry_BRAK_GPS.minuta_jednosci = czas_cyfry_BRAK_GPS_pom.minuta_jednosci;
                czas_cyfry_BRAK_GPS.minuta_dziesiatki = czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki;
                czas_cyfry_BRAK_GPS.sekunda_jednosci = czas_cyfry_BRAK_GPS_pom.sekunda_jednosci;
                czas_cyfry_BRAK_GPS.sekunda_dziesiatki = czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki;
                czas_cyfry_BRAK_GPS.setnasekunda_jednosci = czas_cyfry_BRAK_GPS_pom.setnasekunda_jednosci;
                czas_cyfry_BRAK_GPS.setnasekunda_dziesiatki = czas_cyfry_BRAK_GPS_pom.setnasekunda_dziesiatki;
  
                czas_BRAK_GPS.rok = czas_BRAK_GPS_pom.rok;
                czas_BRAK_GPS.miesiac = czas_BRAK_GPS_pom.miesiac;
                czas_BRAK_GPS.dzien = czas_BRAK_GPS_pom.dzien;
                czas_BRAK_GPS.godzina = czas_BRAK_GPS_pom.godzina;
                czas_BRAK_GPS.sekunda = czas_BRAK_GPS_pom.sekunda;
                czas_BRAK_GPS.minuta = czas_BRAK_GPS_pom.minuta;
                czas_BRAK_GPS.milisekunda = 0;
              }
              
              czas_BRAK_GPS_stan = -1;
              
              
              break;
          }
          czas_BRAK_GPS_stan++;
        }
      }
    }
  } 

  if(p3)
    if(millis() - przycisk3_milis >= CZAS_PRZYCISK)
      if(przycisk3_stan == HIGH)
      {
        przycisk3_milis = millis();
        p3=0;
      }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////// SPRAWDZANIE DOSTEPNOSCI SYGNALU GPS //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gps_dostepnosc()
{
  while(ss.available() > 0)
  {
    gps.encode(ss.read());
  }

  if(gps.time.isValid())
  {
    gps_on_czas = HIGH;
    digitalWrite(dioda, HIGH);
  }
  else
  {
    gps_on_czas = LOW;
    digitalWrite(dioda, LOW);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// OBSLUGA CZASU I DATY Z SYNCHRONIZACJA //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void datagodzina()
{
  if (gps.time.isValid())
  {
    
    czas_GPS.godzina = gps.time.hour() + UTC_offset;
    czas_GPS.minuta = gps.time.minute();
    czas_GPS.sekunda = gps.time.second();
    czas_GPS.dzien = gps.date.day();
    czas_GPS.miesiac = gps.date.month();
    czas_GPS.rok = gps.date.year();
    
    if(czas_GPS.godzina>23) 
    {
      czas_GPS.godzina = czas_GPS.godzina - 24;
      czas_GPS.dzien = czas_GPS.dzien+1;
      if(czas_GPS.dzien == 32) 
      {
        czas_GPS.dzien=0;
        czas_GPS.miesiac = czas_GPS.miesiac+1;
      }
      else if(czas_GPS.dzien == 31 && (czas_GPS.miesiac == 4 || czas_GPS.miesiac == 6 || czas_GPS.miesiac == 9 || czas_GPS.miesiac == 11))
      {
        czas_GPS.dzien=0;
        czas_GPS.miesiac = czas_GPS.miesiac+1;
      }
      else if(czas_GPS.dzien == 30 && przestepny(czas_GPS.rok) && czas_GPS.miesiac == 2)
      {
        czas_GPS.dzien = 0;
        czas_GPS.miesiac = czas_GPS.miesiac + 1; 
      }
      else if(czas_GPS.dzien == 29 && !przestepny(czas_GPS.rok) && czas_GPS.miesiac == 2)
      {
        czas_GPS.dzien = 0;
        czas_GPS.miesiac = czas_GPS.miesiac + 1; 
      }
      if(czas_GPS.miesiac>12) czas_GPS.rok=czas_GPS.rok+1;
    }
    else if(czas_GPS.godzina < 0)
    {
      czas_GPS.godzina = 24 + czas_GPS.godzina;
      czas_GPS.dzien = czas_GPS.dzien - 1;
      if(czas_GPS.dzien == 0)
      {
        czas_GPS.miesiac = czas_GPS.miesiac - 1;
        czas_GPS.dzien = 31;
        if(czas_GPS.miesiac == 4 || czas_GPS.miesiac == 6 || czas_GPS.miesiac == 9 || czas_GPS.miesiac == 11)
        {
          czas_GPS.dzien = 30;
        }
        else if(czas_GPS.miesiac == 2 && !przestepny(czas_GPS.rok))
        {
          czas_GPS.dzien = 28;
        }
        else if(czas_GPS.miesiac == 2 && przestepny(czas_GPS.rok))
        {
          czas_GPS.dzien = 29;
        }
        else if(czas_GPS.miesiac == 0)
        {
          czas_GPS.miesiac = 12;
          czas_GPS.rok = czas_GPS.rok - 1;
        }
      }
    }
    

     czas_cyfry_GPS.godzina_dziesiatki = czas_GPS.godzina / 10;
     czas_cyfry_GPS.godzina_jednosci = czas_GPS.godzina % 10;

     czas_cyfry_GPS.minuta_dziesiatki = czas_GPS.minuta / 10;
     czas_cyfry_GPS.minuta_jednosci = czas_GPS.minuta % 10;

     czas_cyfry_GPS.sekunda_dziesiatki = czas_GPS.sekunda / 10;
     czas_cyfry_GPS.sekunda_jednosci = czas_GPS.sekunda % 10;

     czas_cyfry_GPS.dzien_dziesiatki = czas_GPS.dzien / 10;
     czas_cyfry_GPS.dzien_jednosci = czas_GPS.dzien % 10;

     czas_cyfry_GPS.miesiac_dziesiatki = czas_GPS.miesiac / 10;
     czas_cyfry_GPS.miesiac_jednosci = czas_GPS.miesiac % 10;


    czas_cyfry_GPS.rok_jednosci = czas_GPS.rok % 10;
    //czas_GPS.rok = czas_GPS.rok/10;
    czas_cyfry_GPS.rok_dziesiatki = (czas_GPS.rok / 10) % 10;
    //czas_GPS.rok = czas_GPS.rok/10;
    czas_cyfry_GPS.rok_setki = (czas_GPS.rok / 100) % 10;
    //czas_GPS.rok = czas_GPS.rok/10;
    czas_cyfry_GPS.rok_tysiace = czas_GPS.rok / 1000;
  }   
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// OBSLUGA CZASU I DATY BEZ SYNCHRONIZACJI //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void datagodzina_brak_gps()
{

    czas_BRAK_GPS.milisekunda += roznica_czasu_glowna_petla;
    if(czas_BRAK_GPS.milisekunda > 999)
    {
      czas_BRAK_GPS.milisekunda -= 1000;
      czas_BRAK_GPS.sekunda++;
      if(czas_BRAK_GPS.sekunda > 59)
      {
        czas_BRAK_GPS.sekunda -= 60;
        czas_BRAK_GPS.minuta++;
        if(czas_BRAK_GPS.minuta >59 )
        {
          czas_BRAK_GPS.minuta -= 60;
          czas_BRAK_GPS.godzina++;
          if(czas_BRAK_GPS.godzina > 23)
          {
            czas_BRAK_GPS.godzina -= 24;
            czas_BRAK_GPS.dzien++;
            if(czas_BRAK_GPS.miesiac == 2)
            {
              if(przestepny(czas_BRAK_GPS.rok))
              {
                if(czas_BRAK_GPS.dzien > 29)
                {
                  czas_BRAK_GPS.dzien = 1;
                }
              }
              else
              {
                if(czas_BRAK_GPS.dzien > 28)
                {
                  czas_BRAK_GPS.dzien = 1;
                }
              }
            }
            else if(czas_BRAK_GPS.miesiac == 4 && czas_BRAK_GPS.miesiac == 6 && czas_BRAK_GPS.miesiac == 9 && czas_BRAK_GPS.miesiac == 11)
            {
              if(czas_BRAK_GPS.dzien > 30)
              {
                czas_BRAK_GPS.dzien = 1;
              }
            }
            else
            {
              if(czas_BRAK_GPS.dzien > 31)
              {
                czas_BRAK_GPS.dzien = 1;
              }
            }
            
            czas_BRAK_GPS.miesiac++;
            if(czas_BRAK_GPS.miesiac > 12)
            {
              czas_BRAK_GPS.miesiac = 1;
              czas_BRAK_GPS.rok++;
            }
          }
        }
      }
    }


  czas_cyfry_BRAK_GPS.rok_tysiace = czas_BRAK_GPS.rok / 1000;
  czas_cyfry_BRAK_GPS.rok_setki = (czas_BRAK_GPS.rok / 100) % 10;
  czas_cyfry_BRAK_GPS.rok_dziesiatki = (czas_BRAK_GPS.rok / 10) % 10;
  czas_cyfry_BRAK_GPS.rok_jednosci = czas_BRAK_GPS.rok % 10;
  czas_cyfry_BRAK_GPS.miesiac_dziesiatki = czas_BRAK_GPS.miesiac / 10;
  czas_cyfry_BRAK_GPS.miesiac_jednosci = czas_BRAK_GPS.miesiac % 10;
  czas_cyfry_BRAK_GPS.dzien_dziesiatki = czas_BRAK_GPS.dzien / 10;
  czas_cyfry_BRAK_GPS.dzien_jednosci = czas_BRAK_GPS.dzien % 10;
  czas_cyfry_BRAK_GPS.godzina_dziesiatki = czas_BRAK_GPS.godzina / 10;
  czas_cyfry_BRAK_GPS.godzina_jednosci = czas_BRAK_GPS.godzina % 10;
  czas_cyfry_BRAK_GPS.minuta_dziesiatki = czas_BRAK_GPS.minuta / 10;
  czas_cyfry_BRAK_GPS.minuta_jednosci = czas_BRAK_GPS.minuta % 10;
  czas_cyfry_BRAK_GPS.sekunda_dziesiatki = czas_BRAK_GPS.sekunda / 10;
  czas_cyfry_BRAK_GPS.sekunda_jednosci = czas_BRAK_GPS.sekunda % 10;
  czas_cyfry_BRAK_GPS.setnasekunda_dziesiatki = czas_BRAK_GPS.milisekunda / 100;
  czas_cyfry_BRAK_GPS.setnasekunda_jednosci = (czas_BRAK_GPS.milisekunda / 10) % 10;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// OBSLUGA TRYBOW /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void tryby()
{
  
    /////// OBLICZANIE MILISEKUND CZASU Z SYNCHRONIZACJA /////////////////////////////
    czas_millis = millis();
    roznica_czasu_glowna_petla = czas_millis - wcz_czas_millis;
    czas_GPS.milisekunda += roznica_czasu_glowna_petla;
    wcz_czas_millis = czas_millis;

    if(czas_GPS.milisekunda > 999 || czas_GPS.sekunda != czas_GPS_sekunda_wcz)
    { 
      czas_GPS.milisekunda=0;
    }

    czas_GPS_sekunda_wcz = czas_GPS.sekunda;
    //////////////////////////////////////////////////////////////////////////////////



    //////////// MRUGANIE LAMP CO 300 MILISEKUND /////////////////////////////////////
    mruganie_millis += roznica_czasu_glowna_petla;
    if(mruganie_millis > 300LU) 
    {
      mruganie_millis = 0;
      mruganie = !mruganie;
    }
    //////////////////////////////////////////////////////////////////////////////////

            

  /////// STOPER - ODLICZANIE CZASU /////////////////////////////
  if(stoper_start)
  {
    stoper_millis = millis();
    roznica_czasu_stoper = stoper_millis - wcz_stoper_millis;
    stoper.milisekunda += roznica_czasu_stoper;
    wcz_stoper_millis = stoper_millis;
              
    if(stoper.milisekunda>999)
    {
      stoper.milisekunda -= 1000;
      stoper.sekunda++;
      if(stoper.sekunda>59)
      {
        stoper.sekunda -= 60;
        stoper.minuta++;
        if(stoper.minuta>59)
        {
          stoper.minuta -= 60;
          stoper.godzina++;
        }
      }
    }
  }
  ///////////////////////////////////////////////////////////////


  
  /////// MINUTNIK - ODLICZANIE CZASU /////////////////////////////
  if(minutnik_start)
  {
    minutnik_millis = millis();
    minutnik.milisekunda -= minutnik_millis - wcz_minutnik_millis;
    wcz_minutnik_millis = minutnik_millis;
              
    if(minutnik.milisekunda<0)
    {
      minutnik.milisekunda += 1000;
      minutnik.sekunda--;
      if(minutnik.sekunda<0)
      {
        minutnik.sekunda += 60;
        minutnik.minuta--;
        if(minutnik.minuta<0)
        {
          minutnik.minuta += 60;
          minutnik.godzina--;
        }
      }
    }
  }
  ///////////////////////////////////////////////////////////////


  if(tryb != 3 && budzik_stan!=0) budzik_stan = 0;
  if((tryb != 5 || (tryb == 5 && minutnik_odtwarzanie)) && minutnik_stan != 0) minutnik_stan = 0;
  if((tryb != 10 || (tryb == 10 && budzik_odtwarzanie)) && czas_BRAK_GPS_stan != 0) czas_BRAK_GPS_stan = 0;
  
  if(wybor_piosenek_odtwarzanie && tryb != 7 && wybor_piosenek_odtwarzanie != 0) 
  {
    wybor_piosenek_odtwarzanie = 0;
    zatrzymaj_odtwarzanie();
  }
  
  switch(tryb)
    {

      ////////////////////////////////////////////////////////////
      //////////////HH:MM:SS  DD:MM:YYYY///////GPS///////////////
      case 0:
      {
          if(gps_on_czas)
          {
            lampy[0] = czas_cyfry_GPS.godzina_dziesiatki;
            lampy[1] = czas_cyfry_GPS.godzina_jednosci;
            lampy[2] = czas_cyfry_GPS.minuta_dziesiatki;
            lampy[3] = czas_cyfry_GPS.minuta_jednosci;
            lampy[4] = czas_cyfry_GPS.sekunda_dziesiatki;
            lampy[5] = czas_cyfry_GPS.sekunda_jednosci;
            lampy[6] = 10;
            lampy[7] = 10;
            lampy[8] = czas_cyfry_GPS.dzien_dziesiatki;
            lampy[9] = czas_cyfry_GPS.dzien_jednosci;
            lampy[10] = czas_cyfry_GPS.miesiac_dziesiatki;
            lampy[11] = czas_cyfry_GPS.miesiac_jednosci;
            lampy[12] = czas_cyfry_GPS.rok_tysiace;
            lampy[13] = czas_cyfry_GPS.rok_setki;
            lampy[14] = czas_cyfry_GPS.rok_dziesiatki;
            lampy[15] = czas_cyfry_GPS.rok_jednosci;
          }
          else
          {
            lampy[0] = 0;
            lampy[1] = 0;
            lampy[2] = 0;
            lampy[3] = 0;
            lampy[4] = 0;
            lampy[5] = 0;
            lampy[6] = 10;
            lampy[7] = 10;
            lampy[8] = 0;
            lampy[9] = 0;
            lampy[10] = 0;
            lampy[11] = 0;
            lampy[12] = 0;
            lampy[13] = 0;
            lampy[14] = 0;
            lampy[15] = 0;
          }
          
          czysc_przecinki();
          prawy_przecinek[1] = HIGH;
          prawy_przecinek[3] = HIGH;
          prawy_przecinek[9] = HIGH;
          prawy_przecinek[11] = HIGH;

          if(budzik_odtwarzanie && gps_on_czas && mruganie == 0)
          {
            for(int i=0; i<16; i++)
            {
              lampy[i] = 10;
            }
            czysc_przecinki();
          }
          break;
      }

         
       ///////////////////////////////////////////////////////////
      //////////////DD:MM:YYYY  HH:MM:SS///////GPS///////////////
      case 1:
      {
          if(gps_on_czas)
          {
            lampy[0] = czas_cyfry_GPS.dzien_dziesiatki;
            lampy[1] = czas_cyfry_GPS.dzien_jednosci;
            lampy[2] = czas_cyfry_GPS.miesiac_dziesiatki;
            lampy[3] = czas_cyfry_GPS.miesiac_jednosci;
            lampy[4] = czas_cyfry_GPS.rok_tysiace;
            lampy[5] = czas_cyfry_GPS.rok_setki;
            lampy[6] = czas_cyfry_GPS.rok_dziesiatki;
            lampy[7] = czas_cyfry_GPS.rok_jednosci;
            lampy[8] = 10;
            lampy[9] = 10;
            lampy[10] = czas_cyfry_GPS.godzina_dziesiatki;
            lampy[11] = czas_cyfry_GPS.godzina_jednosci;
            lampy[12] = czas_cyfry_GPS.minuta_dziesiatki;
            lampy[13] = czas_cyfry_GPS.minuta_jednosci;
            lampy[14] = czas_cyfry_GPS.sekunda_dziesiatki;
            lampy[15] = czas_cyfry_GPS.sekunda_jednosci; 
          }
          else
          {
            lampy[0] = 0;
            lampy[1] = 0;
            lampy[2] = 0;
            lampy[3] = 0;
            lampy[4] = 0;
            lampy[5] = 0;
            lampy[6] = 0;
            lampy[7] = 0;
            lampy[8] = 10;
            lampy[9] = 10;
            lampy[10] = 0;
            lampy[11] = 0;
            lampy[12] = 0;
            lampy[13] = 0;
            lampy[14] = 0;
            lampy[15] = 0;
          }

          czysc_przecinki();
          prawy_przecinek[1] = HIGH;
          prawy_przecinek[3] = HIGH;
          prawy_przecinek[11] = HIGH;
          prawy_przecinek[13] = HIGH;
          
          break;
      }

       
       ///////////////////////////////////////////////////////////
      /////////////////////HH:MM:SS:CC///////GPS/////////////////
       case 2:
       {
          if (gps_on_czas)
          {
            czas_cyfry_GPS.setnasekunda_jednosci = (czas_GPS.milisekunda / 10) % 10;
            czas_cyfry_GPS.setnasekunda_dziesiatki = czas_GPS.milisekunda / 100;
  
            lampy[0] = 10;
            lampy[1] = 10;
            lampy[2] = 10;
            lampy[3] = 10;
            lampy[4] = czas_cyfry_GPS.godzina_dziesiatki;
            lampy[5] = czas_cyfry_GPS.godzina_jednosci;
            lampy[6] = czas_cyfry_GPS.minuta_dziesiatki;
            lampy[7] = czas_cyfry_GPS.minuta_jednosci;
            lampy[8] = czas_cyfry_GPS.sekunda_dziesiatki;
            lampy[9] = czas_cyfry_GPS.sekunda_jednosci;
            lampy[10] = czas_cyfry_GPS.setnasekunda_dziesiatki;
            lampy[11] = czas_cyfry_GPS.setnasekunda_jednosci;
            lampy[12] = 10;
            lampy[13] = 10;
            lampy[14] = 10;
            lampy[15] = 10;
          }
          else
          {
            lampy[0] = 10;
            lampy[1] = 10;
            lampy[2] = 10;
            lampy[3] = 10;
            lampy[4] = 0;
            lampy[5] = 0;
            lampy[6] = 0;
            lampy[7] = 0;
            lampy[8] = 0;
            lampy[9] = 0;
            lampy[10] = 0;
            lampy[11] = 0;
            lampy[12] = 10;
            lampy[13] = 10;
            lampy[14] = 10;
            lampy[15] = 10;
          }

          czysc_przecinki();
          prawy_przecinek[5] = HIGH;
          prawy_przecinek[7] = HIGH;
          prawy_przecinek[9] = HIGH;
          lewy_przecinek[10] = HIGH;         
          break;
      }

      
       ///////////////////////////////////////////////////////////
      /////////////////////BUDZIK////////////////////////////////    
      case 3:
      {
        czysc_przecinki();
        
        lampy[0] = budziki[wybrany_budzik].tryb;
        lampy[1] = budziki[wybrany_budzik].gps;
        lampy[2] = 10;
        lampy[3] = budziki[wybrany_budzik].godzina_dziesiatki;
        lampy[4] = budziki[wybrany_budzik].godzina_jednosci;
        lampy[5] = budziki[wybrany_budzik].minuta_dziesiatki;
        lampy[6] = budziki[wybrany_budzik].minuta_jednosci;
        lampy[7] = 10;

        if(budzik_stan == 0)
        {
          lampy[0] = budziki[wybrany_budzik].tryb;
          lampy[1] = budziki[wybrany_budzik].gps;
          lampy[2] = 10;
          lampy[3] = budziki[wybrany_budzik].godzina_dziesiatki;
          lampy[4] = budziki[wybrany_budzik].godzina_jednosci;
          lampy[5] = budziki[wybrany_budzik].minuta_dziesiatki;
          lampy[6] = budziki[wybrany_budzik].minuta_jednosci;
          lampy[7] = 10;
          
          if(budziki[wybrany_budzik].tryb == 0 || budziki[wybrany_budzik].tryb == 1)
          {
            
            lampy[8] = budziki[wybrany_budzik].dzien_dziesiatki;
            lampy[9] = budziki[wybrany_budzik].dzien_jednosci;
            lampy[10] = budziki[wybrany_budzik].miesiac_dziesiatki;
            lampy[11] = budziki[wybrany_budzik].miesiac_jednosci;
            lampy[12] = budziki[wybrany_budzik].rok_dziesiatki;
            lampy[13] = budziki[wybrany_budzik].rok_jednosci;
            
            prawy_przecinek[9] = HIGH;
            prawy_przecinek[11] = HIGH;
          }
          else if(budziki[wybrany_budzik].tryb == 2)
          {
            lampy[8] = 10;
            lampy[9] = 10;
            lampy[10] = 10;
            lampy[11] = 10;
            lampy[12] = 10;
            lampy[13] = 10; 
          }
          else
          {
            lampy[8] = budziki[wybrany_budzik].dzien_dziesiatki;
            lampy[9] = budziki[wybrany_budzik].dzien_jednosci;
            lampy[10] = budziki[wybrany_budzik].miesiac_dziesiatki;
            lampy[11] = budziki[wybrany_budzik].miesiac_jednosci;
            lampy[12] = 10;
            lampy[13] = 10;
            prawy_przecinek[9] = HIGH;
          }
        }
        else
        {
          lampy[0] = budziki_pom.tryb;
          lampy[1] = budziki_pom.gps;
          lampy[2] = 10;
          lampy[3] = budziki_pom.godzina_dziesiatki;
          lampy[4] = budziki_pom.godzina_jednosci;
          lampy[5] = budziki_pom.minuta_dziesiatki;
          lampy[6] = budziki_pom.minuta_jednosci;
          lampy[7] = 10;
          
          if(budziki_pom.tryb == 0 || budziki_pom.tryb == 1)
          {
          
            lampy[8] = budziki_pom.dzien_dziesiatki;
            lampy[9] = budziki_pom.dzien_jednosci;
            lampy[10] = budziki_pom.miesiac_dziesiatki;
            lampy[11] = budziki_pom.miesiac_jednosci;
            lampy[12] = budziki_pom.rok_dziesiatki;
            lampy[13] = budziki_pom.rok_jednosci;
            
            prawy_przecinek[9] = HIGH;
            prawy_przecinek[11] = HIGH;
          }
          else if(budziki_pom.tryb == 2)
          {
            lampy[8] = 10;
            lampy[9] = 10;
            lampy[10] = 10;
            lampy[11] = 10;
            lampy[12] = 10;
            lampy[13] = 10; 
          }
          else
          {
            lampy[8] = budziki_pom.dzien_dziesiatki;
            lampy[9] = budziki_pom.dzien_jednosci;
            lampy[10] = budziki_pom.miesiac_dziesiatki;
            lampy[11] = budziki_pom.miesiac_jednosci;
            lampy[12] = 10;
            lampy[13] = 10;
            prawy_przecinek[9] = HIGH;
          }
        }

        czysc_przecinki();
        prawy_przecinek[0] = HIGH;
        prawy_przecinek[1] = HIGH;
        prawy_przecinek[4] = HIGH;
        prawy_przecinek[15] = HIGH;

        if(mruganie == 0)
        {
          if(budzik_stan == 1) 
          {
            lampy[0] = 10;
            prawy_przecinek[0] = 0;
            lewy_przecinek[0] = 0;
          }
          else if(budzik_stan == 2) 
          {
            lampy[1] = 10;
            prawy_przecinek[1] = 0;
            lewy_przecinek[1] = 0;
          }
          else if(budzik_stan == 3) 
          {
            lampy[3] = 10;
            prawy_przecinek[3] = 0;
            lewy_przecinek[3] = 0;
          }
          else if(budzik_stan == 4) 
          {
            lampy[4] = 10;
            prawy_przecinek[4] = 0;
            lewy_przecinek[4] = 0;
          }
          else if(budzik_stan == 5) 
          {
            lampy[5] = 10;
            prawy_przecinek[5] = 0;
            lewy_przecinek[5] = 0;
          }
          else if(budzik_stan == 6) 
          {
            lampy[6] = 10;
            prawy_przecinek[6] = 0;
            lewy_przecinek[6] = 0;
          }
          else if(budzik_stan == 7) 
          {
            lampy[8] = 10;
            prawy_przecinek[8] = 0;
            lewy_przecinek[8] = 0;
          }
          else if(budzik_stan == 8) 
          {
            lampy[9] = 10;
            prawy_przecinek[9] = 0;
            lewy_przecinek[9] = 0;
          }
          else if(budzik_stan == 9) 
          {
            lampy[10] = 10;
            prawy_przecinek[10] = 0;
            lewy_przecinek[10] = 0;
          }
          else if(budzik_stan == 10) 
          {
            lampy[11] = 10;
            prawy_przecinek[11] = 0;
            lewy_przecinek[11] = 0;
          }
          else if(budzik_stan == 11) 
          {
            lampy[12] = 10;
            prawy_przecinek[12] = 0;
            lewy_przecinek[12] = 0;
          }
          else if(budzik_stan == 12) 
          {
            lampy[13] = 10;
            prawy_przecinek[13] = 0;
            lewy_przecinek[13] = 0;
          }
        }

        lampy[14] = 10;
        lampy[15] = wybrany_budzik;
        break;
      } 

       ///////////////////////////////////////////////////////////
      /////////////////////STOPER////////////////////////////////
      case 4:
      {
          int i=0;
          
          if(stoper_start == 0 && wybrany_pomiar > 0)
          {
            stoper_cyfry.setnasekunda_jednosci = (pomiary[wybrany_pomiar-1].milisekunda / 10) % 10;
            stoper_cyfry.setnasekunda_dziesiatki = pomiary[wybrany_pomiar-1].milisekunda / 100;
  
            stoper_cyfry.sekunda_jednosci = pomiary[wybrany_pomiar-1].sekunda % 10;
            stoper_cyfry.sekunda_dziesiatki = pomiary[wybrany_pomiar-1].sekunda / 10;
  
            stoper_cyfry.minuta_jednosci = pomiary[wybrany_pomiar-1].minuta % 10;
            stoper_cyfry.minuta_dziesiatki = pomiary[wybrany_pomiar-1].minuta / 10;

            godzina_stoper_pom = pomiary[wybrany_pomiar-1].godzina;
  
            for(i=0; godzina_stoper_pom>9; i++)
            {
              godzina_stoper_pom /=10;
            }
  
            godzina_stoper_pom = pomiary[wybrany_pomiar-1].godzina;


            if(i==0)
            {
              lampy[5] = godzina_stoper_pom;
              lampy[4] = 0;
            }
            else
            {
              for(int a=0; a<=i; a++)
              {
                lampy[5-a] = godzina_stoper_pom % 10;
                godzina_stoper_pom /= 10;
              }
            }
  
            for(int a=0; a<5-i; a++)
            {
              lampy[a]=10;
            }
          }
          else if(stoper_start == 1 || (stoper_start == 0 && wybrany_pomiar == 0))
          {
            stoper_cyfry.setnasekunda_jednosci = (stoper.milisekunda /10 ) % 10;
            stoper_cyfry.setnasekunda_dziesiatki = stoper.milisekunda / 100;
  
            stoper_cyfry.sekunda_jednosci = stoper.sekunda % 10;
            stoper_cyfry.sekunda_dziesiatki = stoper.sekunda / 10;
  
            stoper_cyfry.minuta_jednosci = stoper.minuta % 10;
            stoper_cyfry.minuta_dziesiatki = stoper.minuta / 10;

            godzina_stoper_pom = stoper.godzina;
  
            for(i=0; godzina_stoper_pom>9; i++)
            {
              godzina_stoper_pom /=10;
            }
  
            godzina_stoper_pom = stoper.godzina;
  
            if(i==0)
            {
              lampy[5] = godzina_stoper_pom;
              lampy[4] = 0;
            }
            else
            {
              for(int a=0; a<=i; a++)
              {
                lampy[5-a] = godzina_stoper_pom % 10;
                godzina_stoper_pom /= 10;
              }
            }
  
            for(int a=0; a<5-i; a++)
            {
              lampy[a]=10;
            }  
             
          }

          lampy[6] = stoper_cyfry.minuta_dziesiatki;
          lampy[7] = stoper_cyfry.minuta_jednosci;
          lampy[8] = stoper_cyfry.sekunda_dziesiatki;
          lampy[9] = stoper_cyfry.sekunda_jednosci;
          lampy[10] = stoper_cyfry.setnasekunda_dziesiatki;
          lampy[11] = stoper_cyfry.setnasekunda_jednosci;
          lampy[12] = 10;
          lampy[13] = 10;
          lampy[14] = 10;

          if(stoper_start == 1) lampy[15] = ilosc_pomiarow;
          else  lampy[15] = wybrany_pomiar;
          
          
          czysc_przecinki();
          prawy_przecinek[5] = HIGH;
          prawy_przecinek[7] = HIGH;
          prawy_przecinek[9] = HIGH;
          lewy_przecinek[10] = HIGH;
          break;
      }


      ///////////////////////////////////////////////////////////
      /////////////////////MINUTNIK/////////////////////////////
      case 5:
      {
        czysc_przecinki();
        minutnik_cyfry.sekunda_jednosci = minutnik.sekunda % 10;
        minutnik_cyfry.sekunda_dziesiatki = minutnik.sekunda / 10;
  
        minutnik_cyfry.minuta_jednosci = minutnik.minuta % 10;
        minutnik_cyfry.minuta_dziesiatki = minutnik.minuta / 10;

        minutnik_cyfry.godzina_jednosci = minutnik.godzina % 10;
        minutnik_cyfry.godzina_dziesiatki = minutnik.godzina / 10;

        lampy[0] = 10;
        lampy[1] = 10;
        lampy[2] = 10;
        lampy[3] = 10;
        lampy[4] = 10;
        lampy[5] = minutnik_cyfry.godzina_dziesiatki;
        lampy[6] = minutnik_cyfry.godzina_jednosci;
        lampy[7] = minutnik_cyfry.minuta_dziesiatki;
        lampy[8] = minutnik_cyfry.minuta_jednosci;
        lampy[9] = minutnik_cyfry.sekunda_dziesiatki;
        lampy[10] = minutnik_cyfry.sekunda_jednosci;
        lampy[11] = 10;
        lampy[12] = 10;

        if(minutnik_stan == 0)
        {
          lampy[13] = 10;
          lampy[14] = 10;
          lampy[15] = 10;
          prawy_przecinek[15] = LOW;
        }
        else
        {
          
          lampy[13] = minutnik.piosenka / 100;
          lampy[14] = (minutnik.piosenka / 10) % 10;
          lampy[15] = minutnik.piosenka % 10;
          prawy_przecinek[15] = HIGH;
        }

        prawy_przecinek[6] = HIGH;
        prawy_przecinek[8] = HIGH;
        

        if(minutnik_odtwarzanie && mruganie == 0)
        {
          for(int i=0; i<16; i++)
          {
            lampy[i] = 10;
          }
          czysc_przecinki();
        }
        else if(mruganie == 0)
        {
          if(minutnik_stan == 1) 
          {
            lampy[5] = 10;
            prawy_przecinek[5] = 0;
            lewy_przecinek[5] = 0;
          }
          else if(minutnik_stan == 2) 
          {
            lampy[6] = 10;
            prawy_przecinek[6] = 0;
            lewy_przecinek[6] = 0;
          }
          else if(minutnik_stan == 3) 
          {
            lampy[7] = 10;
            prawy_przecinek[7] = 0;
            lewy_przecinek[7] = 0;
          }
          else if(minutnik_stan == 4) 
          {
            lampy[8] = 10;
            prawy_przecinek[8] = 0;
            lewy_przecinek[8] = 0;
          }
          else if(minutnik_stan == 5) 
          {
            lampy[9] = 10;
            prawy_przecinek[9] = 0;
            lewy_przecinek[9] = 0;
          }
          else if(minutnik_stan == 6) 
          {
            lampy[10] = 10;
            prawy_przecinek[10] = 0;
            lewy_przecinek[10] = 0;
          }
          else if(minutnik_stan == 7) 
          {
            lampy[13] = 10;
            prawy_przecinek[13] = 0;
            lewy_przecinek[13] = 0;
          }
          else if(minutnik_stan == 8) 
          {
            lampy[14] = 10;
            prawy_przecinek[14] = 0;
            lewy_przecinek[14] = 0;
          }
          else if(minutnik_stan == 9) 
          {
            lampy[15] = 10;
            prawy_przecinek[15] = 0;
            lewy_przecinek[15] = 0;
          }
        }
        
        break;
      }

       ///////////////////////////////////////////////////////////
      /////////////////////BUDZIK - WYBÓR MP3////////////////////
      case 6:
      {
        czysc_przecinki();
        prawy_przecinek[13] = HIGH;
        prawy_przecinek[15] = HIGH;
        
        lampy[0] = 10;
        lampy[1] = 10;
        lampy[2] = 10;
        lampy[3] = 10;
        lampy[4] = 10;
        lampy[5] = 10;
        lampy[6] = 10;
        lampy[7] = 10;
        lampy[8] = 10;
        lampy[9] = 10;
        lampy[10] = 10;
        lampy[11] = budziki[budziki_wybor_piosenek_wybrany].piosenka / 100;
        lampy[12] = (budziki[budziki_wybor_piosenek_wybrany].piosenka / 10 ) % 10;
        lampy[13] = budziki[budziki_wybor_piosenek_wybrany].piosenka % 10;
        lampy[14] = 10;
        lampy[15] = budziki_wybor_piosenek_wybrany; 

        if(budziki_wybor_piosenek_stan && mruganie == 0)
        {
          lampy[11] = 10;
          lampy[12] = 10;
          lampy[13] = 10;

          czysc_przecinki();
          prawy_przecinek[15] = HIGH;
        }
        else if(!budziki_wybor_piosenek_stan && mruganie == 0)
        {
          lampy[15] = 10;
          
          czysc_przecinki();
          prawy_przecinek[13] = HIGH;
        }
        break;
      }
      
       //////////////////////////////////////////////////////////
      /////////////////////ODTWARZACZ MP3///////////////////////
      case 7:
      {
        lampy[0] = 10;
        lampy[1] = 10;
        lampy[2] = 10;
        lampy[3] = 10;
        lampy[4] = 10;
        lampy[5] = 10;
        lampy[6] = 10;
        lampy[7] = 10;
        lampy[8] = 10;
        lampy[9] = 10;
        lampy[10] = 10;
        lampy[11] = 10;
        lampy[12] = 10;
        lampy[13] = piosenka / 100;
        lampy[14] = (piosenka / 10) % 10;
        lampy[15] = piosenka % 10;

        czysc_przecinki();
        prawy_przecinek[15] = HIGH;
        break;
      }
      
       ///////////////////////////////////////////////////////////
      /////////////////////HH:MM:SS:CC///////////////////////////
      case 8:
      {
        lampy[0] = 10;
        lampy[1] = 10;
        lampy[2] = 10;
        lampy[3] = 10;
        lampy[4] = czas_cyfry_BRAK_GPS.godzina_dziesiatki;
        lampy[5] = czas_cyfry_BRAK_GPS.godzina_jednosci;
        lampy[6] = czas_cyfry_BRAK_GPS.minuta_dziesiatki;
        lampy[7] = czas_cyfry_BRAK_GPS.minuta_jednosci;
        lampy[8] = czas_cyfry_BRAK_GPS.sekunda_dziesiatki;
        lampy[9] = czas_cyfry_BRAK_GPS.sekunda_jednosci;
        lampy[10] = czas_cyfry_BRAK_GPS.setnasekunda_dziesiatki;
        lampy[11] = czas_cyfry_BRAK_GPS.setnasekunda_jednosci;
        lampy[12] = 10;
        lampy[13] = 10;
        lampy[14] = 10;
        lampy[15] = 10;

        czysc_przecinki();
        prawy_przecinek[5] = HIGH;
        prawy_przecinek[7] = HIGH;
        prawy_przecinek[9] = HIGH;
        lewy_przecinek[10] = HIGH;       
        
        break;
      } 


       ///////////////////////////////////////////////////////////
      ///////////////DD:MM:YYYY  HH:MM:SS////////////////////////
      case 9:
      {
        lampy[0] = czas_cyfry_BRAK_GPS.dzien_dziesiatki;
        lampy[1] = czas_cyfry_BRAK_GPS.dzien_jednosci;
        lampy[2] = czas_cyfry_BRAK_GPS.miesiac_dziesiatki;
        lampy[3] = czas_cyfry_BRAK_GPS.miesiac_jednosci;
        lampy[4] = czas_cyfry_BRAK_GPS.rok_tysiace;
        lampy[5] = czas_cyfry_BRAK_GPS.rok_setki;
        lampy[6] = czas_cyfry_BRAK_GPS.rok_dziesiatki;
        lampy[7] = czas_cyfry_BRAK_GPS.rok_jednosci;
        lampy[8] = 10;
        lampy[9] = 10;
        lampy[10] = czas_cyfry_BRAK_GPS.godzina_dziesiatki;
        lampy[11] = czas_cyfry_BRAK_GPS.godzina_jednosci;
        lampy[12] = czas_cyfry_BRAK_GPS.minuta_dziesiatki;
        lampy[13] = czas_cyfry_BRAK_GPS.minuta_jednosci;
        lampy[14] = czas_cyfry_BRAK_GPS.sekunda_dziesiatki;
        lampy[15] = czas_cyfry_BRAK_GPS.sekunda_jednosci; 

        czysc_przecinki();
        prawy_przecinek[1] = HIGH;
        prawy_przecinek[3] = HIGH;
        prawy_przecinek[11] = HIGH;
        prawy_przecinek[13] = HIGH;
        
        break;
      }

      
       ////////////////////////////////////////////////////////////
      //////////////HH:MM:SS  DD:MM:YYYY//////////////////////////
      case 10:
      {
        if(czas_BRAK_GPS_stan == 0)
        {
          lampy[0] = czas_cyfry_BRAK_GPS.godzina_dziesiatki;
          lampy[1] = czas_cyfry_BRAK_GPS.godzina_jednosci;
          lampy[2] = czas_cyfry_BRAK_GPS.minuta_dziesiatki;
          lampy[3] = czas_cyfry_BRAK_GPS.minuta_jednosci;
          lampy[4] = czas_cyfry_BRAK_GPS.sekunda_dziesiatki;
          lampy[5] = czas_cyfry_BRAK_GPS.sekunda_jednosci;
          lampy[6] = 10;
          lampy[7] = 10;
          lampy[8] = czas_cyfry_BRAK_GPS.dzien_dziesiatki;
          lampy[9] = czas_cyfry_BRAK_GPS.dzien_jednosci;
          lampy[10] = czas_cyfry_BRAK_GPS.miesiac_dziesiatki;
          lampy[11] = czas_cyfry_BRAK_GPS.miesiac_jednosci;
          lampy[12] = czas_cyfry_BRAK_GPS.rok_tysiace;
          lampy[13] = czas_cyfry_BRAK_GPS.rok_setki;
          lampy[14] = czas_cyfry_BRAK_GPS.rok_dziesiatki;
          lampy[15] = czas_cyfry_BRAK_GPS.rok_jednosci;
        }
        else
        {
          lampy[0] = czas_cyfry_BRAK_GPS_pom.godzina_dziesiatki;
          lampy[1] = czas_cyfry_BRAK_GPS_pom.godzina_jednosci;
          lampy[2] = czas_cyfry_BRAK_GPS_pom.minuta_dziesiatki;
          lampy[3] = czas_cyfry_BRAK_GPS_pom.minuta_jednosci;
          lampy[4] = czas_cyfry_BRAK_GPS_pom.sekunda_dziesiatki;
          lampy[5] = czas_cyfry_BRAK_GPS_pom.sekunda_jednosci;
          lampy[6] = 10;
          lampy[7] = 10;
          lampy[8] = czas_cyfry_BRAK_GPS_pom.dzien_dziesiatki;
          lampy[9] = czas_cyfry_BRAK_GPS_pom.dzien_jednosci;
          lampy[10] = czas_cyfry_BRAK_GPS_pom.miesiac_dziesiatki;
          lampy[11] = czas_cyfry_BRAK_GPS_pom.miesiac_jednosci;
          lampy[12] = czas_cyfry_BRAK_GPS_pom.rok_tysiace;
          lampy[13] = czas_cyfry_BRAK_GPS_pom.rok_setki;
          lampy[14] = czas_cyfry_BRAK_GPS_pom.rok_dziesiatki;
          lampy[15] = czas_cyfry_BRAK_GPS_pom.rok_jednosci;
        }

        czysc_przecinki();
        prawy_przecinek[1] = HIGH;
        prawy_przecinek[3] = HIGH;
        prawy_przecinek[9] = HIGH;
        prawy_przecinek[11] = HIGH;
        
        if(budzik_odtwarzanie && mruganie == 0)
        {
          for(int i=0; i<16; i++)
          {
            lampy[i] = 10;
          }
          czysc_przecinki();
        }
        
        if(mruganie == 0 && czas_BRAK_GPS_stan != 0)
        {
          if(czas_BRAK_GPS_stan == 1)
          {
            lampy[0] = 10;
            lewy_przecinek[0] = 0;
            prawy_przecinek[0] = 0;
          }
          if(czas_BRAK_GPS_stan == 2)
          {
            lampy[1] = 10;
            lewy_przecinek[1] = 0;
            prawy_przecinek[1] = 0;
          }
          if(czas_BRAK_GPS_stan == 3)
          {
            lampy[2] = 10;
            lewy_przecinek[2] = 0;
            prawy_przecinek[2] = 0;
          }
          if(czas_BRAK_GPS_stan == 4)
          {
            lampy[3] = 10;
            lewy_przecinek[3] = 0;
            prawy_przecinek[3] = 0;
          }
          if(czas_BRAK_GPS_stan == 5)
          {
            lampy[4] = 10;
            lewy_przecinek[4] = 0;
            prawy_przecinek[4] = 0;
          }
          if(czas_BRAK_GPS_stan == 6)
          {
            lampy[5] = 10;
            lewy_przecinek[5] = 0;
            prawy_przecinek[5] = 0;
          }
          if(czas_BRAK_GPS_stan == 7)
          {
            lampy[8] = 10;
            lewy_przecinek[8] = 0;
            prawy_przecinek[8] = 0;
          }
          if(czas_BRAK_GPS_stan == 8)
          {
            lampy[9] = 10;
            lewy_przecinek[9] = 0;
            prawy_przecinek[9] = 0;
          }
          if(czas_BRAK_GPS_stan == 9)
          {
            lampy[10] = 10;
            lewy_przecinek[10] = 0;
            prawy_przecinek[10] = 0;
          }
          if(czas_BRAK_GPS_stan == 10)
          {
            lampy[11] = 10;
            lewy_przecinek[11] = 0;
            prawy_przecinek[11] = 0;
          }
          if(czas_BRAK_GPS_stan == 11)
          {
            lampy[12] = 10;
            lewy_przecinek[12] = 0;
            prawy_przecinek[12] = 0;
          }
          if(czas_BRAK_GPS_stan == 12)
          {
            lampy[13] = 10;
            lewy_przecinek[13] = 0;
            prawy_przecinek[13] = 0;
          }
          if(czas_BRAK_GPS_stan == 13)
          {
            lampy[14] = 10;
            lewy_przecinek[14] = 0;
            prawy_przecinek[14] = 0;
          }
          if(czas_BRAK_GPS_stan == 14)
          {
            lampy[15] = 10;
            lewy_przecinek[15] = 0;
            prawy_przecinek[15] = 0;
          }
        }
        break;
      }
   }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////// SPRAWDZENIE CZY BUDZIK POWIENIEN URUCHOMIC ALARM ////////////////////////////////////////////////////////////////////////////////////////////////////////////
void budzik_fun()
{
  for(int i=0; i<10; i++)
  {
    switch(budziki[i].tryb)
    {
      case 1:
      {
        if(budziki[i].gps && gps_on_czas)
        {
          if(budziki[i].gotowy && czas_cyfry_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_GPS.sekunda_dziesiatki == 0 && czas_cyfry_GPS.sekunda_jednosci == 0 && czas_cyfry_GPS.dzien_dziesiatki == budziki[i].dzien_dziesiatki && czas_cyfry_GPS.dzien_jednosci == budziki[i].dzien_jednosci
          && czas_cyfry_GPS.miesiac_dziesiatki == budziki[i].miesiac_dziesiatki && czas_cyfry_GPS.miesiac_jednosci == budziki[i].miesiac_jednosci && czas_cyfry_GPS.rok_dziesiatki == budziki[i].rok_dziesiatki
          && czas_cyfry_GPS.rok_jednosci == budziki[i].rok_jednosci)
          {
            budziki[i].tryb = 0;
            ktory_odtwarzany = i;
            budzik_odtworz(i, 0);
          }
        }
        else
        {
          if(budziki[i].gotowy && czas_cyfry_BRAK_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_BRAK_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_BRAK_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_BRAK_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_GPS.sekunda_dziesiatki == 0 && czas_cyfry_GPS.sekunda_jednosci == 0 && czas_cyfry_BRAK_GPS.dzien_dziesiatki == budziki[i].dzien_dziesiatki && czas_cyfry_BRAK_GPS.dzien_jednosci == budziki[i].dzien_jednosci
          && czas_cyfry_BRAK_GPS.miesiac_dziesiatki == budziki[i].miesiac_dziesiatki && czas_cyfry_BRAK_GPS.miesiac_jednosci == budziki[i].miesiac_jednosci && czas_cyfry_BRAK_GPS.rok_dziesiatki == budziki[i].rok_dziesiatki
          && czas_cyfry_BRAK_GPS.rok_jednosci == budziki[i].rok_jednosci)
          {
            budziki[i].tryb = 0;
            ktory_odtwarzany = i;
            budzik_odtworz(i, 0);
          }
        }
        break;
      }

      case 2:
      {
        if(budziki[i].gps && gps_on_czas)
        {
          if(czas_cyfry_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_GPS.sekunda_jednosci == 0 && czas_cyfry_GPS.sekunda_dziesiatki == 0)
          {
            if(budziki[i].gotowy)
            {
              budzik_odtworz(i, 0);
              ktory_odtwarzany = i;
              budziki[i].gotowy = 0;
            }
          }
          else
          {
            budziki[i].gotowy = 1;
          }
        }
        else
        {
          if(czas_cyfry_BRAK_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_BRAK_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_BRAK_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_BRAK_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_BRAK_GPS.sekunda_jednosci == 0 && czas_cyfry_BRAK_GPS.sekunda_dziesiatki == 0)
          {
            if(budziki[i].gotowy)
            {
              budzik_odtworz(i, 0);
              ktory_odtwarzany = i;
              budziki[i].gotowy = 0;
            }
          }
          else
          {
            budziki[i].gotowy = 1;
          }
        }
        break;
      }
      
      case 3:
      {
        if(budziki[i].gps && gps_on_czas)
        {
          if(czas_cyfry_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_GPS.dzien_dziesiatki == budziki[i].dzien_dziesiatki && czas_cyfry_GPS.dzien_jednosci == budziki[i].dzien_jednosci
          && czas_cyfry_GPS.miesiac_dziesiatki == budziki[i].miesiac_dziesiatki && czas_cyfry_GPS.miesiac_jednosci == budziki[i].miesiac_jednosci && czas_cyfry_GPS.sekunda_jednosci == 0 && czas_cyfry_GPS.sekunda_dziesiatki == 0)
          {
            if(budziki[i].gotowy)
            {
              budzik_odtworz(i, 0);
              ktory_odtwarzany = i;
              budziki[i].gotowy = 0;
            }
          }
          else
          {
            budziki[i].gotowy = 1;
          }
        }
        else
        {
          if(czas_cyfry_BRAK_GPS.godzina_dziesiatki == budziki[i].godzina_dziesiatki && czas_cyfry_BRAK_GPS.godzina_jednosci == budziki[i].godzina_jednosci && czas_cyfry_BRAK_GPS.minuta_dziesiatki == budziki[i].minuta_dziesiatki 
          && czas_cyfry_BRAK_GPS.minuta_jednosci == budziki[i].minuta_jednosci && czas_cyfry_BRAK_GPS.dzien_dziesiatki == budziki[i].dzien_dziesiatki && czas_cyfry_BRAK_GPS.dzien_jednosci == budziki[i].dzien_jednosci
          && czas_cyfry_BRAK_GPS.miesiac_dziesiatki == budziki[i].miesiac_dziesiatki && czas_cyfry_BRAK_GPS.miesiac_jednosci == budziki[i].miesiac_jednosci && czas_cyfry_BRAK_GPS.sekunda_jednosci == 0 && czas_cyfry_BRAK_GPS.sekunda_dziesiatki == 0)
          {
            if(budziki[i].gotowy)
            {
              budzik_odtworz(i, 0);
              ktory_odtwarzany = i;
              budziki[i].gotowy = 0;
            }
          }
          else
          {
            budziki[i].gotowy = 1;
          }
        }
        break;
      }
    }
  }

  if(budzik_odtwarzanie)
  {
    if(millis() - budzik_millis > 300000)   ///////// ALARM SIE WYLACZA SAM PO 5 MINUTACH
    {
      zatrzymaj_odtwarzanie();
      budzik_odtwarzanie = 0;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// SPRAWDZENIE CZY MINUTNIK POWIENIEN URUCHOMIC ALARM ///////////////////////////////////////////////////////////////////////////////////////////////////////
void minutnik_fun()
{
  if(minutnik_start && minutnik.godzina == 0 && minutnik.minuta == 0 && minutnik.sekunda == 0)
  {
    minutnik.milisekunda = 0;
    minutnik_start = 0;
    minutnik_odtwarzanie = 1;
    budzik_odtworz(0, 1);
  }

  if(minutnik_odtwarzanie)
  {
    if(millis() - minutnik_millis > 300000) ///////// ALARM SIE WYLACZA SAM PO 5 MINUTACH
    {
      zatrzymaj_odtwarzanie();
      minutnik_odtwarzanie = 0;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



///////////// WYKONANIE SEKWENCJI ZAPALENIA MULTIPLEKSOWEGO CALEGO WYSWIETLACZA /////////////////////////////////////////////////////////////////////////////////////////////
void nixie()
{
  switch(nixie_stan)
  {
    case 0:
      digitalWrite(SRCLK, LOW);
      digitalWrite(SER, wybrana_nixie ? LOW : HIGH);
      digitalWrite(SRCLK, HIGH);
      
      nixie_stan = 1;
      nixie_micros = micros();
      break;

    case 1:
      if(micros() - nixie_micros > CZAS_WYGASZENIA)
      {
        nixie_stan = 2;
        ustawkatody(lampy[wybrana_nixie], lewy_przecinek[wybrana_nixie], prawy_przecinek[wybrana_nixie]);
        if(lampy[wybrana_nixie] != 10)
        {
          digitalWrite(RCLK, HIGH);
        }
       
        nixie_micros = micros(); 
      }
      break;

    case 2:
      if(micros() - nixie_micros > CZAS_ZAPALENIA)
      { 
        nixie_stan = 0;
        digitalWrite(RCLK, LOW);
        wybrana_nixie++;
        if(wybrana_nixie>15)
        {
          wybrana_nixie=0;
        }
      }
      break;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////// USTAWIENIE KATOD ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void ustawkatody(uint8_t katoda, uint8_t lewy_przecinek, uint8_t prawy_przecinek)
{
  for(int i=0; i<10; i++)
  {
    katody[i]=LOW;
  }
  if(katoda!=10) katody[katoda]=HIGH;
  
    digitalWrite(K0, katody[0]);
    digitalWrite(K1, katody[1]);
    digitalWrite(K2, katody[2]);
    digitalWrite(K3, katody[3]);
    digitalWrite(K4, katody[4]);
    digitalWrite(K5, katody[5]);
    digitalWrite(K6, katody[6]);
    digitalWrite(K7, katody[7]);
    digitalWrite(K8, katody[8]);
    digitalWrite(K9, katody[9]);
    
    digitalWrite(PRP, prawy_przecinek);
    digitalWrite(PRL, lewy_przecinek);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



////////////////// WYLACZENIE WSZYSTKICH PRZECINKOW /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void czysc_przecinki()
{
  for(int i=0; i<16; i++)
  {
    lewy_przecinek[i] = LOW;
    prawy_przecinek[i] = LOW;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////// USTAWIENIE TRYBU PRACY MP3 - /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void mp3_tryb_pracy(uint8_t tryb_pracy)
{
  uint8_t polecenie[4] = { 0xaa, 0x18, 0x01, 0x00 };
   polecenie[3] = tryb_pracy;

  uint8_t suma = 0;
  for (uint8_t i=0; i < 4; i++) 
  {
    suma = suma + polecenie[i];
  }

  for (uint8_t i=0; i < 4; i++) 
  {
    mp3.write(polecenie[i]);
  }  
  mp3.write(suma);
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// WYSLANIE POLECENIA ODTWORZENIA MELODII DO MODUŁU MP3 ///////////////////////////////////////////////////////////////////////////////////////////////////////
void budzik_odtworz(uint16_t numer, uint8_t tryb_sm)
{
  uint8_t polecenie[5] = { 0xaa, 0x07, 0x02, 0x00, 0x00 };
  
  if(tryb_sm == 0)
  {
    mp3_tryb_pracy(1);
    budzik_odtwarzanie = 1;
    budzik_millis = millis();
    polecenie[3] = budziki[numer].piosenka >> 8;
    polecenie[4] = budziki[numer].piosenka & 0xff;
  }
  else if(tryb_sm == 1)
  {
    mp3_tryb_pracy(1);
    minutnik_odtwarzanie = 1;
    minutnik_millis = millis();
    polecenie[3] = minutnik.piosenka >> 8;
    polecenie[4] = minutnik.piosenka & 0xff;
  }
  else if(tryb_sm == 2)
  {
    mp3_tryb_pracy(1);
    wybor_piosenek_odtwarzanie = 1;
    polecenie[3] = piosenka >> 8;
    polecenie[4] = piosenka & 0xff;
  }
  else if(tryb_sm == 3)
  {
    mp3_tryb_pracy(2);
    polecenie[3] = numer >> 8;
    polecenie[4] = numer & 0xff;
  }
  

  uint8_t suma = 0;
  for (uint8_t i=0; i < 5; i++) 
  {
    suma = suma + polecenie[i];
  }

  for (uint8_t i=0; i < 5; i++) 
  {
    mp3.write(polecenie[i]);
  }  
  mp3.write(suma);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// WYSLANIE POLECENIA ZATRZYMANIA MELODII DO MODUŁU MP3 ///////////////////////////////////////////////////////////////////////////////////////////////////////
void zatrzymaj_odtwarzanie()
{
  uint8_t polecenie[4] = { 0xaa, 0x04, 0x00, 0xae };
  for(int i=0; i<4; i++)
  {
    mp3.write(polecenie[i]);
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// SPRAWDZANIE POPRAWNOSCI WPROWADZEGO CZASU I DATY ///////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t poprawnosc_czasu(uint8_t poprawnosc_tryb, uint8_t godzina, uint8_t minuta, uint8_t sekunda, uint8_t dzien, uint8_t miesiac, uint16_t rok)
{
  switch(poprawnosc_tryb) 
  {
    case 0:   //////// SAMA GODZINA /////////
      if(godzina > 23 || minuta > 59 || sekunda > 59)
      {
        return false;
      }
      else return true;
      break;

    case 1:   //////// GODZINA I DATA BEZ ROKU ////////
      if(godzina > 23 || minuta > 59 || sekunda > 59 ||  miesiac == 0 || miesiac > 12 || dzien == 0 || (miesiac == 2 && dzien > 29) 
      || ((miesiac == 1 || miesiac == 3 || miesiac == 5 || miesiac == 7 || miesiac == 8 || miesiac == 10 || miesiac == 12) && dzien > 31) || ((miesiac == 4 || miesiac == 6 || miesiac == 9 || miesiac == 11)  && dzien > 30))
      {
        return false;
      }
      else return true;
      break;

    case 2:   //////// GODZINA I DATA  ////////
      if(godzina > 23 || minuta > 59 || sekunda > 59 || miesiac == 0 || miesiac > 12 || dzien == 0 || (miesiac == 2 && przestepny(rok) && dzien > 29) || (miesiac == 2 && !przestepny(rok) && dzien > 28) 
      || ((miesiac == 1 || miesiac == 3 || miesiac == 5 || miesiac == 7 || miesiac == 8 || miesiac == 10 || miesiac == 12) && dzien > 31) || ((miesiac == 4 || miesiac == 6 || miesiac == 9 || miesiac == 11)  && dzien > 30))
      {
        return false;
      }
      else return true;
      break;
  }

  return false;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////// SPRAWDZENIE CZY DANY ROK JEST PRZESTEPNY ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t przestepny(uint16_t rok)
{
  if((rok % 4 == 0 && rok % 100 != 0) || rok % 400 == 0)
  {
    return true;
  }
  else
  {
    return false;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



/////////////////// JESLI USTAWIANIE BUDZIKA BYLO PRAWIDLOWE, NASTEPUJE ZMIANA ZAPISANIE TYMCZASOWYCH DANYCH JAKO STALE /////////////////////////////////////////////////////
void zamiana_budzikow()
{
  budziki[wybrany_budzik].tryb = budziki_pom.tryb;
  budziki[wybrany_budzik].gps = budziki_pom.gps;
  budziki[wybrany_budzik].godzina_dziesiatki = budziki_pom.godzina_dziesiatki;
  budziki[wybrany_budzik].godzina_jednosci = budziki_pom.godzina_jednosci;
  budziki[wybrany_budzik].minuta_dziesiatki = budziki_pom.minuta_dziesiatki;
  budziki[wybrany_budzik].minuta_jednosci = budziki_pom.minuta_jednosci;
  budziki[wybrany_budzik].dzien_dziesiatki = budziki_pom.dzien_dziesiatki;
  budziki[wybrany_budzik].dzien_jednosci = budziki_pom.dzien_jednosci;
  budziki[wybrany_budzik].miesiac_dziesiatki = budziki_pom.miesiac_dziesiatki;
  budziki[wybrany_budzik].miesiac_jednosci = budziki_pom.miesiac_jednosci;
  budziki[wybrany_budzik].rok_dziesiatki = budziki_pom.rok_dziesiatki;
  budziki[wybrany_budzik].rok_jednosci = budziki_pom.rok_jednosci;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////// SEKWENCJA SWIECENIE LAMP PRZY WLACZENIU ZASILANIA //////////////////////////////////////////////////////////////////////////////////////////////////////////
void sekwencja_startowa()
{
      digitalWrite(K0, HIGH);

      digitalWrite(RCLK, LOW);
      for(int i=0; i<16; i++)
      {
        digitalWrite(SRCLK, LOW);
        digitalWrite(SER, HIGH);
        digitalWrite(SRCLK, HIGH);
      }
      
      digitalWrite(RCLK, HIGH);
      delay(300);
      digitalWrite(RCLK, LOW);
      delay(300);

      for(int i=0; i<16; i++)
      {
        digitalWrite(SRCLK, LOW);
        digitalWrite(SER, LOW);
        digitalWrite(SRCLK, HIGH);
      }
      
      
      for(int k=0; k<2; k++)
      {
        digitalWrite(RCLK, LOW);
        digitalWrite(SRCLK, LOW);
        digitalWrite(SER, HIGH);
        digitalWrite(SRCLK, HIGH);
        digitalWrite(RCLK, HIGH);
        delay(70);
    
        for(int i=1; i<16; i++)
        {
          digitalWrite(RCLK, LOW);
          digitalWrite(SRCLK, LOW);
          digitalWrite(SER, LOW);
          digitalWrite(SRCLK, HIGH);
          digitalWrite(RCLK, HIGH);
  
          delay(70);
        }
      }

      delay(400);

      digitalWrite(RCLK, LOW);
      for(int i=0; i<16; i++)
      {
        digitalWrite(SRCLK, LOW);
        digitalWrite(SER, HIGH);
        digitalWrite(SRCLK, HIGH);
      }
      digitalWrite(RCLK, HIGH);
      delay(500);


      digitalWrite(RCLK, LOW);
      for(int i=0; i<16; i++)
      {
        digitalWrite(SRCLK, LOW);
        digitalWrite(SER, LOW);
        digitalWrite(SRCLK, HIGH);
      }
      digitalWrite(RCLK, HIGH);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
