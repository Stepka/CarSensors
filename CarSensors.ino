

/*------------------------------------------------*/
// подключение библиотек для работы с OLED
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
// подключение библиотек для работы с DS18B20
#include <OneWire.h>
#include <Wire.h>
#include <DallasTemperature.h>
//#include "DHT.h"

/*---------------------- Display --------------------------*/
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 7
//Adafruit_SSD1306 display(OLED_RESET);
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*---------------------- Temperature sensors --------------------------*/
#define ONE_WIRE_BUS 2 // вывод, к которому подключён DS18B20
#define TEMPERATURE_PRECISION 12 // точность измерений (9 ... 12)

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensor_hub(&oneWire);
//DeviceAddress Thermometer;
DeviceAddress sensors[1];

/*--------------------- Humidity ---------------------------*/
//#define DHTPIN 4     // Digital pin connected to the DHT sensor
//
//// Uncomment whatever type you're using!
//#define DHTTYPE DHT11   // DHT 11
//
//// Connect pin 1 (on the left) of the sensor to +5V
//// NOTE: If using a board with 3.3V logic like an Arduino Due connect pin 1
//// to 3.3V instead of 5V!
//// Connect pin 2 of the sensor to whatever your DHTPIN is
//// Connect pin 4 (on the right) of the sensor to GROUND
//// Connect a 10K resistor from pin 2 (data) to pin 1 (power) of the sensor
//
//// Initialize DHT sensor.
//// Note that older versions of this library took an optional third parameter to
//// tweak the timings for faster processors.  This parameter is no longer needed
//// as the current DHT reading algorithm adjusts itself to work on faster procs.
//DHT dht(DHTPIN, DHTTYPE);

/*---------------------- Pressure & Attitude --------------------------*/
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // I2C

/*------------------------------------------------*/
int num_devices = 0;
#define OUT_LINE 0
#define IN_LINE 1
/*------------------------------------------------*/

void setup() {
  Serial.begin(9600);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // инициализация дисплея по интерфейсу I2C, адрес 0x3C (для OLED 128x32)
  display.clearDisplay(); // очистка дисплея
  display.setTextColor(WHITE); // установка цвета текста

  sensor_hub.begin(); // инициализация DS18B20
  num_devices = sensor_hub.getDeviceCount();
  Serial.print(F("Found "));
  Serial.print(num_devices);
  Serial.print(F(" DS18B20 sensors"));

  for (int i = 0; i < num_devices; i++)
  {
    if (sensor_hub.getAddress(sensors[i], i))
    {
      sensor_hub.setResolution(sensors[i], TEMPERATURE_PRECISION);
    }
    else
    {
      Serial.println(F("Could not find a valid DS18B20 sensor, check wiring!"));
    }
  }

  //  dht.begin();

  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

/*------------------------------------------------*/
void loop() {

  sensor_hub.requestTemperatures(); // считывание значение температуры

  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  //  float h = dht.readHumidity();
  //  // Read temperature as Celsius (the default)
  //  float t = dht.readTemperature();

  display.clearDisplay(); // очистить дисплей
  display.setTextSize(1); // установка размера шрифта
  display.setCursor(7, 2); // установка курсора в позицию X = 0; Y = 0
  display.print("Cyber Culture Motors"); // знак цельсия
  for (int i = 0; i < num_devices; i++)
  {
    display_temperature(i, sensor_hub.getTempC(sensors[i]));
  }
  display_temperature(IN_LINE, bmp.readTemperature());

  display.setCursor(6, 18 + 2 * 16 + 6); // установка курсора в позицию X = 0; Y = 0
  display.print((char)232);
  display.print(" ");
  display.print(pressureToMmHg(bmp.readPressure()));
  display.print("mmHg    ");

  display.print((char)94);
  display.print(" ");
  display.print(pressureToAltitude(bmp.readPressure()));
  display.print("m");

  /////////////////////////////////////////////////////////////

  display.display(); // всё это отображаем на экране

  delay(500);
}

void display_temperature(byte line, float value)
{
  display.setCursor(15, 23 + line * 16); // установка курсора в позицию X = 0; Y = 0
  display.setTextSize(1); // установка размера шрифта
  switch (line)
  {
    case OUT_LINE:
      display.print("Out ");
      break;
    case IN_LINE:
      display.print(" In ");
      break;
  }
  display.setCursor(36, 16 + line * 16); // установка курсора в позицию X = 0; Y = 0
  display.setTextSize(2); // установка размера шрифта
  if (value >= 0)
  {
    display.print("+"); // выводим значение температуры на экран
  }
  else
  {
    display.print("-"); // выводим значение температуры на экран
  }
  if (value > -10 && value < 10)
  {
    display.print("0"); // выводим значение температуры на экран
  }
  display.print(String(abs(value), 1)); // выводим значение температуры на экран
  display.setCursor(96, 23 + line * 16); // установка курсора в позицию X = 0; Y = 0
  display.setTextSize(1); // установка размера шрифта
  display.print((char)247); // и рисуем
  display.print("C"); // знак цельсия
}

int pressureToAltitude(float pressure)
{
  if (!pressure) return 0;                  // If the pressure module has been disabled return '0'
  pressure /= 100.0F;                     // Convert [Pa] to [hPa]
  return int(44330.0 * (1.0 - pow(pressure / 1013.25, 0.1903))); // Сalculate altitude
}

int pressureToMmHg(float pressure)
{
  return (int)(pressure * 0.00750061683f);          // Convert [Pa] to [mm Hg]
}
