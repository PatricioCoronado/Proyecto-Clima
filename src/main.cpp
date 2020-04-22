/*
 * proyecto: clima ESP32
 *  Escribe en una base de datos variables de humedad y temperatura
 * aplicación: climaESP32-DHT22
 * plataforma: ESP32
 * sensor SHT22
 * Patricio Coronado
 * diciembre de 2018
 * modificado julio 2019
 * Sensado de temperatura y humedad con DHT22
 * Los datos se envian a una página con GET para
 * que los ponga en una base de datos.
 */
/*************************************************************
					Ubicación del sensor
**************************************************************/
//#define __Pueblo //Para usar la wifi del pueblo
#define __Madrid //Para usar la wifi de Madrid
//char lugar[]="Madrid-Habitacion1";//en la base de datos será "origen"
char lugar[]="Madrid-Estudio";//en la base de datos será "origen"
/**************************************************************
      Host de la base de datos y datos de la WIFI
 **************************************************************/
#define HOST sprintf(web,"https://patriciocoronadocollado.000webhostapp.com/clima.php/?valor1=%f&valor2=%f&valor3=%f&ubicacion=%s",temperatura,humedad,pila,lugar);
/**************************************************************
                    WIFI a utilizar
 **************************************************************/
#ifdef __Pueblo
  #define WIFI WiFiMulti.addAP("FX-991SP","Doniga_93");
#endif
#ifdef __Madrid
  #define WIFI wifiMulti.addAP("router_no_encontrado", "RoloTomasi8086");
#endif
/**************************************************************
            Tiempo en deepsleep
 **************************************************************/
#define SEGUNDOS_DEEPSLEEP 1800 //Media Hora
/************************************************************
    Sensor a utilizar. Comentar el que no sea
 ************************************************************/
#define __DHT22
//#define __SHT11
/*************************************************************
 * 					librerias
**************************************************************/
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiMulti.h>
#include "HTTPClientESP32.h"//Librería local
#include "soc/soc.h"//for disable brownout detector
#include "soc/rtc_cntl_reg.h"//for disable brownout detector
/*************************************************************
//declaracines para el sensor
**************************************************************/
#ifdef __DHT22
	#include "DHTesp.h"
	DHTesp Sensor; //Instancia el sensor
	#define DHTTYPE DHT22  // DHT 22 (AM2302)
	#define DHTPIN   15    // Pin which is connected to the DHT sensor.
	TempAndHumidity Datos; // estructura donde se leen los datos del sensor
#endif
#ifdef __SHT11
	#include "SHT1x.h"//Sensor
  	#define dataPin  22 //Pin datos
  	#define clockPin 23 //Pin de clk solo para el SHT11
  	SHT1x sht1x(dataPin, clockPin);
#endif
/************************************************************
			ADC
*************************************************************/
#define pinDiodo 32 //Salida para poner corriente en el diodo de referencia
#define vDiodo 34 //Pin para leer la tensión del diodo
float pila; //Voltaje de la pila
int ADC; //Lectura del ADC
/************************************************************
			declaración de funciones
*************************************************************/
void entra_en_modo_deepsleep_ESP32(int);
void desabilita_brownout(void);
void imprime_variables(void);
boolean lee_sensor_dht22(void);
/*************************************************************
					declaracines para el wifi
**************************************************************/
WiFiMulti wifiMulti;
char web[256]; //URL para enviar el GET
/*************************************************************
 *  		variables y constantes globales
**************************************************************/
float temperatura,humedad;
boolean depuracion=true;
boolean lecturaSensor=false;
/*************************************************************
						Pines
**************************************************************/
#define LED 12
/*************************************************************
						SETUP
**************************************************************/
void setup() 
{
/*
    Configuración de la atenuación ADC
	ADC_0db, Vin=3V->3V
    ADC_2_5db, Vin=3V->2.238V
    ADC_6db, Vin=3V->1.5V
    ADC_11db, 3.9V Vin=3V->0.833V limitado por VDD_Max 
*/
 	//analogSetAttenuation(ADC_6db);
  	//analogSetPinAttenuation(pinVref,ADC_6db);
	desabilita_brownout();
	pinMode(LED,OUTPUT);
	pinMode(32,OUTPUT);
	digitalWrite(LED,LOW);
	Serial.begin(115200);
	Serial.flush();
	#ifdef __DHT22
		Sensor.setup(DHTPIN,DHTesp::DHT22);//Inicializa el sensor
	#endif
	WIFI //Determina las variables de la wifi
}
/*************************************************************
						LOOP
**************************************************************/
void loop() 
{ 	
	//Calcula la tensión de alimentación
	
	digitalWrite(pinDiodo,HIGH);
	delay(50);//Espera a que se estableca la sallida
	ADC=0;
	for(int i=0;i<10;i++) ADC = ADC + analogRead(vDiodo);//Toma 10 conversiones
	ADC=ADC/10;//Calcula la media
	digitalWrite(pinDiodo,LOW);
	if(ADC!=0) pila=0.61*4095/ADC;//Calcula la tensión de la pila
	else pila=-1;
	//Lee el sensor
	#ifdef __SHT11//..............................................................
    humedad = sht1x.readHumidity();
    temperatura = sht1x.readTemperatureC();
   #endif//.......................................................................
   #ifdef __DHT22 //..............................................................
   lecturaSensor=lee_sensor_dht22();
   int contadorDHT22=1;
   while(!lecturaSensor)
   {
     delay(2000);
     lecturaSensor=lee_sensor_dht22();
     if(depuracion) Serial.printf("intentando leer el DHT22 %d veces...",contadorDHT22);
    if(++contadorDHT22>=10) 
    {
      Serial.printf("no pude leer el DHT22...by");
      entra_en_modo_deepsleep_ESP32(SEGUNDOS_DEEPSLEEP);
    }
   }
   humedad = Datos.humidity;
   temperatura = Datos.temperature;
   #endif //........................................................................
	if(depuracion) imprime_variables();
	// conecta con la wifi envia datos
		if((wifiMulti.run() == WL_CONNECTED)) 
		{
			digitalWrite(LED,HIGH);
			HTTPClient http; //
			//URL de entrada a la base de datos
			HOST //Rellena el array "web" con la dirección donde se envían los datos
			if(depuracion) Serial.println("...enviando la solicitud al servidor.....");
			http.begin(web); //inicia la conexión
			int httpCode = http.GET();//Envía dato. HttpCode es el código recibido desde el servidos
			//Aqui se gestina la recepción de la página solicitada
				if(httpCode > 0 ) 
				{
					if(depuracion) Serial.println("exito en el intento de conexion con la URL!");
					if(httpCode == HTTP_CODE_OK) 
					{//Si recibe una página...
						String payload = http.getString(); //payload contendrá #OK
						//Serial.println(payload);
						if (payload.indexOf("#OK")!=-1)
							Serial.println("#0K");
						else 
							Serial.println("#ERROR");
					}
				}
				else {Serial.println("fallo el intento de conexion con la URL!");}
			
			http.end();
			digitalWrite(LED,LOW);
		}
	
	
	entra_en_modo_deepsleep_ESP32(SEGUNDOS_DEEPSLEEP);
}
/************************************************************			
						FUNCIONES
*************************************************************/
/*************************************************************
		funcion que lee el sensor DHT22
**************************************************************/
#ifdef __DHT22
boolean lee_sensor_dht22(void)
{
	Datos = Sensor.getTempAndHumidity();
	delay(500);
	// Check if any reads failed and exit early (to try again).
	if (Sensor.getStatus() != 0) 
	{
		if(depuracion)
		{Serial.println("¡error de status en el sensor!: " + String(Sensor.getStatusString()));}
		return false;
	}
	else return true;
}
#endif
/*************************************************************
		funcion entra en deepsleep
	Habilitamos el temporizador con la función 
	esp_sleep_enable_timer_wakeup,	ingresamos el tiempo 
	en deep sleep en segundos y luego llamamos a la función
	 esp_deep_sleep_start. 
**************************************************************/
void entra_en_modo_deepsleep_ESP32(int segundosDeepSleep)
{
	#define uSEGUNDOS_A_SEGUNDOS 1000000  // factor de conversion de micro seconds a seconds 
	esp_sleep_enable_timer_wakeup(uSEGUNDOS_A_SEGUNDOS * segundosDeepSleep);
	esp_deep_sleep_start();
}
/*************************************************************
		funcion que desabilita el brownout del chip
		para que no se resetee cuando suba el consumo 
		y caiga la tensión de la pila al conectar la wifi
**************************************************************/
void desabilita_brownout(void)
{
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
}
/****************************************************************
 *  Envía por el puerto serie temperatura y humedad
 ***************************************************************/
void imprime_variables(void)
{
      Serial.flush();
      Serial.print("Temperatura: ");
      Serial.print(temperatura);
      Serial.println(" *C");
      Serial.flush();
      Serial.print("Humedad relativa: ");
      Serial.print(humedad);
      Serial.println("%");
      Serial.flush();
	  Serial.print("voltaje de la pila: ");
	  Serial.print(pila);
      Serial.println("V");
      Serial.flush();
} 
/******************  FIN  **************************************/	