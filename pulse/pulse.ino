/*  PulseSensor Starter Project and Signal Tester
 *  The Best Way to Get Started  With, or See the Raw Signal of, your PulseSensor.com™ & Arduino.
 *
 *  Here is a link to the tutorial
 *  https://pulsesensor.com/pages/code-and-guide
 *
 *  WATCH ME (Tutorial Video):
 *  https://www.youtube.com/watch?v=RbB8NSRa5X4
 *
 *
-------------------------------------------------------------
1) This shows a live human Heartbeat Pulse.
2) Live visualization in Arduino's Cool "Serial Plotter".
3) Blink an LED on each Heartbeat.
4) This is the direct Pulse Sensor's Signal.
5) A great first-step in troubleshooting your circuit and connections.
6) "Human-readable" code that is newbie friendly."
*/
#include <ESP8266WiFi.h> // Importa a Biblioteca ESP8266WiFi
#include <PubSubClient.h> // Importa a Biblioteca PubSubClient

//  Variables
int PulseSensorPurplePin = 0;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
int LED13 = 14;   //  The on-board Arduion LED


int Signal;                // holds the incoming raw data. Signal value can range from 0-1024
int Threshold = 810;            // Determine which Signal to "count as a beat", and which to ingore.

//defines de id mqtt e tópicos para publicação e subscribe
#define TOPICO_SUBSCRIBE "feau/pulso"     //tópico MQTT de escuta. 
#define topicLive "feau/sensor/live"
                                   //ATENÇÃO: deve ser o mesmo tópico de publish utilizado na Raspberry PI!!!
                                                 
#define ID_MQTT "sensor"     //id mqtt (para identificação de sessão)
                                        //IMPORTANTE: este deve ser único no broker (ou seja, 
                                        //            se um client MQTT tentar entrar com o mesmo 
                                        //            id de outro já conectado ao broker, o broker 
                                        //            irá fechar a conexão de um deles).
                               

// WIFI

//const char* SSID = "IOT"; // SSID / nome da rede WI-FI que deseja se conectar
//const char* PASSWORD = "iotalpha"; // Senha da rede WI-FI que deseja se conectar


const char* SSID = "UnivapWifi"; // SSID / nome da rede WI-FI que deseja se conectar
const char* PASSWORD = "universidade"; // Senha da rede WI-FI que deseja se conectar

char live_status = 0;

long lastMsg = 0;

char msg[20];

int looping = 0;
 
// MQTT
const char* BROKER_MQTT = "broker.hivemq.com"; //URL do broker MQTT que se deseja utilizar
//const char* BROKER_MQTT = "192.168.1.1"; //URL do broker MQTT que se deseja utilizar
int BROKER_PORT = 1883;//1881; // Porta do Broker MQTT

//Variáveis e objetos globais
WiFiClient espClient; // Cria o objeto espClient
PubSubClient MQTT(espClient); // Instancia o Cliente MQTT passando o objeto espClient
char EstadoSaida = '0';  //variável que armazena o estado atual da saída
char CmdNodeMCU[3] = {'B','T', '1'};   //ATENÇÃO: troque o "BBB" por um dos códigos enviados pela Raspberry PI ("BT1", "BT2", "BT3", "BT4" ou "BT5"),
                                       //         de modo que cada NodeMCU "espere" apenas por um deles
 
//Prototypes
void initSerial();
void initWiFi();
void initMQTT();
void reconectWiFi(); 
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void VerificaConexoesWiFIEMQTT(void);
void InitOutput(void);

//Função: inicializa e conecta-se na rede WI-FI desejada
//Parâmetros: nenhum
//Retorno: nenhum
void initWiFi() 
{
    delay(10);
    Serial.println("------Conexao WI-FI------");
    Serial.print("Conectando-se na rede: ");
    Serial.println(SSID);
    Serial.println("Aguarde");
    
    reconectWiFi();
}
 
//Função: inicializa parâmetros de conexão MQTT(endereço do 
//        broker, porta e seta função de callback)
//Parâmetros: nenhum
//Retorno: nenhum
void initMQTT() 
{
    MQTT.setServer(BROKER_MQTT, BROKER_PORT);   //informa qual broker e porta deve ser conectado
    MQTT.setCallback(mqtt_callback);            //atribui função de callback (função chamada quando qualquer informação de um dos tópicos subescritos chega)
}
 
//Função: função de callback 
//esta função é chamada toda vez que uma informação de 
//um dos tópicos subescritos chega)
//Parâmetros: nenhum
//Retorno: nenhum
void mqtt_callback(char* topic, byte* payload, unsigned int length) 
{
    char MsgRecebida[3];
    
    //se a mensagem não tem três caracteres (ou seja, não é "BT1", "BT2", "BT3", "BT4" ou "BT5"), 
    //automaticamente ela é inválida. Nesse caso, nada mais é feito com a mensagem recebida.
    if (length != 3)
      return;

    //obtem a string do payload recebido
    for(int i = 0; i < length; i++) 
    {
      MsgRecebida[i] = (char)payload[i];
      Serial.print(MsgRecebida[i]);
    }
       
   
    //avalia se a mensagem é para este NodeMCU
    if (memcmp(MsgRecebida,CmdNodeMCU,3) == 0)
    {
      digitalWrite(D0,HIGH);
      EstadoSaida = '1';
      Serial.println("LIGADO!");   
    }else
    {
      digitalWrite(D0,LOW);
      EstadoSaida = '0';
      Serial.println("DESLIGADO!");    
    }    
}
 
//Função: reconecta-se ao broker MQTT (caso ainda não esteja conectado ou em caso de a conexão cair)
//        em caso de sucesso na conexão ou reconexão, o subscribe dos tópicos é refeito.
//Parâmetros: nenhum
//Retorno: nenhum
void reconnectMQTT() 
{
    while (!MQTT.connected()) 
    {
        Serial.print("* Tentando se conectar ao Broker MQTT: ");
        Serial.println(BROKER_MQTT);
        if (MQTT.connect(ID_MQTT)) 
        {
            Serial.println("Conectado com sucesso ao broker MQTT!");
            MQTT.subscribe(TOPICO_SUBSCRIBE);
            MQTT.subscribe(topicLive);
            digitalWrite(D1, HIGH); 
        } 
        else 
        {
            Serial.println("Falha ao reconectar no broker.");
            Serial.println("Havera nova tentatica de conexao em 2s");
            digitalWrite(D1, LOW);
            delay(2000);
        }
    }
}
 
//Função: reconecta-se ao WiFi
//Parâmetros: nenhum
//Retorno: nenhum
void reconectWiFi() 
{
    //se já está conectado a rede WI-FI, nada é feito. 
    //Caso contrário, são efetuadas tentativas de conexão
    if (WiFi.status() == WL_CONNECTED)
        return;
        
    WiFi.begin(SSID, PASSWORD); // Conecta na rede WI-FI
    
    while (WiFi.status() != WL_CONNECTED) 
    {
        delay(100);
        Serial.print(".");
    }
  
    Serial.println();
    Serial.print("Conectado com sucesso na rede ");
    Serial.print(SSID);
    Serial.println("IP obtido: ");
    Serial.println(WiFi.localIP());
}

//Função: verifica o estado das conexões WiFI e ao broker MQTT. 
//        Em caso de desconexão (qualquer uma das duas), a conexão
//        é refeita.
//Parâmetros: nenhum
//Retorno: nenhum
void VerificaConexoesWiFIEMQTT(void)
{
    if (!MQTT.connected()) 
        reconnectMQTT(); //se não há conexão com o Broker, a conexão é refeita
    
     reconectWiFi(); //se não há conexão com o WiFI, a conexão é refeita
}

//Função: inicializa o output em nível lógico baixo
//Parâmetros: nenhum
//Retorno: nenhum
void InitOutput(void)
{
    pinMode(D0, OUTPUT);
    pinMode(D1, OUTPUT);
    digitalWrite(D0, LOW);
    digitalWrite(D1, LOW);         
    EstadoSaida = '0';
}

// The SetUp Function:
void setup() {
  pinMode(LED13,OUTPUT);         // pin that will blink to your heartbeat!
  Serial.begin(115200);         // Set's up Serial Communication at certain speed.
  Serial.println("Iniciando...");
  initWiFi();
  initMQTT();
   
}

// The Main Loop Function
void loop() {
  int pulse;
  
  VerificaConexoesWiFIEMQTT();
   
  Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
                                              // Assign this value to the "Signal" variable.

   Serial.println(Signal);                    // Send the Signal value to Serial Plotter.


   if(Signal > Threshold){                          // If the signal is above "550", then "turn-on" Arduino's on-Board LED.
     digitalWrite(LED13,HIGH);
   } else {
     digitalWrite(LED13,LOW);                //  Else, the sigal must be below "550", so "turn-off" this LED.
   }

   if(
    looping > 30)

    {
      looping = 0;
      pulse = 99;
      dtostrf(pulse, 4, 2, msg); //4 is mininum width, 2 is precision
      MQTT.publish(TOPICO_SUBSCRIBE, msg);

      Serial.println("Sended!!");
    }

delay(100);

  looping++;

}
