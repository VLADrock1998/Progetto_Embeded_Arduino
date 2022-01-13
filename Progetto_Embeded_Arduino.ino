#include <FastLED.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
/*PROTOTIPI FUNZIONI*/
void setup();
void loop();
static void t_ventole_frontali(void *pvParameters);
static void t_ventole_corsair(void *pvParameters);
static void t_ssd(void *pvParameters);
static void t_pompa(void *pvParameters);
static void t_ventola_posteriore(void *pvParameters);
static void t_verifica_accensione(void *pvParameters);
static void t_verifica_pulsante_joy(void *pvParameters);
static void t_verifica_luminosita(void *pvParameters);
static void t_verifica_temperatura(void *pvParameters);
static void t_verifica_temperatura(void *pvParameters);
static void t_communicator(void *pvParameters);
static CRGB get_colore_BASE(int colore);
static CRGB get_colore(int colore);
static void set_colore(int colore, int red, int green, int blue);
static void show_leds(void);
static void show_leds_spegni_tutto(void);
static void spegni_tutto(void);
/*DEFINIZIONE PIN DISPOSITIVI E COSTANTI*/
#define LED_PIN_FAN1 7          //Ventola frontale alta
#define LED_PIN_FAN2 10         //Ventola frontale bassa
#define LED_PIN_FAN3 8          //Ventola interna posteriore
#define LED_PIN_CORSAIR 4       //3 Ventole dissipatore(3[bassa] 4[alta] 5[centrale]) + 3 Ventole top(1[frontale] 2[posteriore] 6[centrale])
#define LED_PIN_PUMP 9          //Pompa dissipatore
#define LED_PIN_SSD1 13         //SSD basso
#define LED_PIN_SSD2 12         //SSD medio basso
#define LED_PIN_SSD3 11         //SSD medio alto
#define LED_PIN_SSD4 3          //SSD alto
#define THERM_PIN 6             //Pin termometro
#define DHTTYPE DHT11           //Tipo di sensore temperatura utilizzato
#define JOYSTICK_X_PIN A1       //Coordinata X joystick
#define JOYSTICK_Y_PIN A2       //Coordinata Y joystick
#define JOYSTICK_BUTTON 2       //Pulsante joystick
#define MAIN_BUTTON 5           //Pulsante principale
#define LUM_PIN A10             //Sensore luminosità
#define NUM_LEDS_FAN_SSD  8     //Numero led ventole frontali e ssd
#define NUM_LEDS_R_FAN  24      //Numero led ventola posteriore
#define NUM_LEDS_PUMP 12        //Numero led pompa
#define NUM_LEDS_CORSAIR 96     //Numero led corsair -> intervalli: Top_frontale[0,15] Top_posteriore[16,31] Dissi_bassa[32,47] Dissi_alta[48,63] Dissi_centro[64,79] Top_centrale[80,95]
#define NUM_LEDS_CORSAIR_FAN 16 //Numero led singola ventola corsair
/*DEFINIZIONE DISPOSITIVI CON LED*/
static CRGB fan1[NUM_LEDS_FAN_SSD];            //Ventola frontale alta
static CRGB fan2[NUM_LEDS_FAN_SSD];            //Ventola frontale bassa
static CRGB fan3[NUM_LEDS_R_FAN];              //Ventola interna posteriore
static CRGB corsair[NUM_LEDS_CORSAIR];         //3 Ventole dissipatore(3[bassa] 4[alta] 5[centrale]) + 3 Ventole top(posizione 1[frontale] 2[posteriore])
static CRGB pump[NUM_LEDS_PUMP];               //Pompa dissipatore
static CRGB ssd1[NUM_LEDS_FAN_SSD];            //SSD basso
static CRGB ssd2[NUM_LEDS_FAN_SSD];            //SSD medio basso
static CRGB ssd3[NUM_LEDS_FAN_SSD];            //SSD medio alto
static CRGB ssd4[NUM_LEDS_FAN_SSD];            //SSD alto
/*DEFINIZIONE COLORI*/
//                     R   G   B
static CRGB colore0_BASE=CRGB(255,0  ,0  );    //Ventole Frontali
static CRGB colore1_BASE=CRGB(0  ,0  ,255);    //
static CRGB colore2_BASE=CRGB(255,0  ,0  );    //Ventole Corsair
static CRGB colore3_BASE=CRGB(0  ,255,0  );    //
static CRGB colore4_BASE=CRGB(255,0  ,0  );    //SSD
static CRGB colore5_BASE=CRGB(255,0  ,255);    //
static CRGB colore6_BASE=CRGB(255,0  ,0  );    //Pompa     NON SONO UTILIZZATI.
static CRGB colore7_BASE=CRGB(255,255,0  );    //
static CRGB colore8_BASE=CRGB(0  ,0  ,255);    //Ventola Posteriore
static CRGB colore9_BASE=CRGB(0  ,255,255);    //          NON È UTILIZZATO.
static CRGB colore0=colore0_BASE;              //Ventole Frontali
static CRGB colore1=colore1_BASE;              //
static CRGB colore2=colore2_BASE;              //Ventole Corsair
static CRGB colore3=colore3_BASE;              //
static CRGB colore4=colore4_BASE;              //SSD
static CRGB colore5=colore5_BASE;              //
static CRGB colore6=colore6_BASE;              //Pompa     NON SONO UTILIZZATI.
static CRGB colore7=colore7_BASE;              //
static CRGB colore8=colore8_BASE;              //Ventola Posteriore
static CRGB colore9=colore9_BASE;              //          NON È UTILIZZATO.
static CRGB spento =CRGB(  0,  0,  0);         //Spento
/*DEFINIZIONE SEMAFORI*/
static SemaphoreHandle_t sem_show;             //Mutex che regola la funzione FASTLED.SHOW -> se non si controlla l'accesso a questa funzione possono succedere cose spiacevoli.
static SemaphoreHandle_t sem_read_color;       //Semaforo che regola l'accesso ai colori CRGB. Queste possono essere lette contemporaneamente da molti task ma scritte solo da uno alla volta.
static SemaphoreHandle_t sem_is_on;            //Semaforo che regola l'accesso alla variabile IS_ON. Questa può essere letta contemporaneamente da molti task ma scritta solo da uno alla volta.
static SemaphoreHandle_t sem_temperatura;      //Semaforo che regola l'accesso alla variabile temperatura. Questa può essere letta contemporaneamente da molti task ma scritta solo da uno alla volta.
static SemaphoreHandle_t sem_colori_invertiti; //Semaforo che regola l'accesso alla variabile JOY_COLORI_INVERTITI. Può essere letta contemporaneamente da molti task ma scritta solo da uno alla volta.
/*DEFINIZIONE VARIABILI E OGGETTI UTILI*/
static const unsigned int message_len = 150;   //Lunghezza massima dei messaggi mandati al communicator
static bool is_on;                             //Variabile che indica se il sistema è acceso o spento
static bool joy_colori_invertiti;              //Variabile che indica se invertire o no i colori del task joystick
static bool supress_communicator;              //Variabile che serve ad evitare che il communicator spammi le comunicazioni. Utile in fase di testing
static int ritardo;                            //Variabile che indica la velocità di riproduzione dell'effetto RGB
static int max_report_counter;                 //Numero di cicli prima che un task invii il report
static float temperatura;                      //Temperatura rilevata dal sensore
static float max_temp;                         //Temperatura massima rilevabile
static float min_temp;                         //Temperatura minima rilevabile
static int max_lum;                            //Luminosità massima rilevabile
static int min_lum;                            //Luminosità minima rilevabile
static QueueHandle_t coda_messaggi;            //Gestore della coda della comunicazione MANY-TO-ONE
static DHT therm(THERM_PIN, DHTTYPE);          //Termometro sensore DHT11

void setup(void) 
{
    /*INIZIALIZZO DEVICE RGB ETC.*/
    FastLED.addLeds<WS2812,LED_PIN_FAN1,GRB>(fan1,NUM_LEDS_FAN_SSD);
    FastLED.addLeds<WS2812,LED_PIN_FAN2,GRB>(fan2,NUM_LEDS_FAN_SSD);
    FastLED.addLeds<WS2812,LED_PIN_FAN3,GRB>(fan3,NUM_LEDS_R_FAN);
    FastLED.addLeds<WS2812,LED_PIN_CORSAIR,GRB>(corsair,NUM_LEDS_CORSAIR);
    FastLED.addLeds<WS2812,LED_PIN_PUMP,GRB>(pump,NUM_LEDS_PUMP);
    FastLED.addLeds<WS2812,LED_PIN_SSD1,GRB>(ssd1,NUM_LEDS_FAN_SSD);
    FastLED.addLeds<WS2812,LED_PIN_SSD2,GRB>(ssd2,NUM_LEDS_FAN_SSD);
    FastLED.addLeds<WS2812,LED_PIN_SSD3,GRB>(ssd3,NUM_LEDS_FAN_SSD);
    FastLED.addLeds<WS2812,LED_PIN_SSD4,GRB>(ssd4,NUM_LEDS_FAN_SSD);
    pinMode(JOYSTICK_BUTTON, INPUT_PULLUP);
    pinMode(MAIN_BUTTON, INPUT);
    coda_messaggi=xQueueCreate(20, sizeof(char)*message_len);
    therm.begin();
    Serial.begin(9600);
    Serial.println("Inizializzazione oggetti terminata.");
    /*INIZIALIZZAZIONE VARIABILI*/
    is_on=true;
    joy_colori_invertiti=false;
    supress_communicator=false;
    ritardo=25;
    max_report_counter=50;
    min_lum=150;
    max_lum=850;
    min_temp=10.0;
    max_temp=50.0;
    temperatura=min_temp;
    Serial.println("Inizializzazione variabili terminata.");
    /*INIZIALIZZAZIONE SEMAFORI*/
    sem_show = xSemaphoreCreateMutex();
    sem_read_color = xSemaphoreCreateMutex();
    sem_is_on = xSemaphoreCreateMutex(); 
    sem_temperatura = xSemaphoreCreateMutex();
    sem_colori_invertiti = xSemaphoreCreateMutex();
    Serial.println("Inizializzazione semafori terminata.");
    /*CREAZIONE TASKS*/
    //          NOME FUNZIONE                                  DESCRIZIONE                      MEM         PRIORITA -> 5=MAX
    xTaskCreate(t_ventole_frontali,         (const portCHAR *)"Ventole frontali",                980, NULL, 4, NULL);//Stack ipotizzato: 944
    xTaskCreate(t_ventole_corsair,          (const portCHAR *)"Ventole corsair",                 980, NULL, 1, NULL);//Stack ipotizzato: 946
    xTaskCreate(t_ssd,                      (const portCHAR *)"SSD",                             980, NULL, 1, NULL);//Stack ipotizzato: 942
    xTaskCreate(t_pompa,                    (const portCHAR *)"Pompa dissipatore",               980, NULL, 1, NULL);//Stack ipotizzato: 912
    xTaskCreate(t_ventola_posteriore,       (const portCHAR *)"Ventola posteriore",              980, NULL, 1, NULL);//Stack ipotizzato: 948
    xTaskCreate(t_verifica_accensione,      (const portCHAR *)"Controllo pulsante accensione",  1000, NULL, 5, NULL);//Stack ipotizzato: 985
    xTaskCreate(t_verifica_pulsante_joy,    (const portCHAR *)"Controllo pulsante joystick",    1000, NULL, 5, NULL);//Stack ipotizzato: 987
    xTaskCreate(t_verifica_luminosita,      (const portCHAR *)"Controllo luminosità",           2000, NULL, 2, NULL);//Stack ipotizzato: 1951
    xTaskCreate(t_verifica_temperatura,     (const portCHAR *)"Controllo temperatura",          1900, NULL, 2, NULL);//Stack ipotizzato: 1882
    xTaskCreate(t_communicator,             (const portCHAR *)"Comunicazione logs",             2000, NULL, 3, NULL);//Stack ipotizzato: 1975
    Serial.println("Creazione tasks terminata.");
    /*AVVIO SCHEDULER*/
    Serial.println("Avvio scheduling. PREGA.");
    vTaskStartScheduler();
    Serial.println("Questo non dovresti leggerlo mai.");
}
void loop(void) 
{
/*IL LOOP NON DOVREBBE MAI ESSERE ESEGUITO*/
}

void t_ventole_frontali (void *pvParameters)
{
    /*  COSA FA:        Utilizza il joystick per colorare a piacimento le due ventole frontali del case.
        COSA COMUNICA:  Che ha eseguito.
    */  
    int report_counter=0;
    int x;
    int y;
    int col_primario;
    int col_secondario;
    while(1)
    {
        xSemaphoreTake(sem_colori_invertiti, portMAX_DELAY);
        if (joy_colori_invertiti)
        {
            col_primario=1;
            col_secondario=0;
        }
        else
        {
            col_primario=0;
            col_secondario=1;
        }
        xSemaphoreGive(sem_colori_invertiti);
        
        x = analogRead(JOYSTICK_X_PIN);
        y = analogRead(JOYSTICK_Y_PIN);
        if (x>=874)
        {
            if (y<362)
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[6]=fan1[7]=get_colore(col_secondario);
                fan2[6]=fan2[7]=get_colore(col_secondario);
                show_leds();
            }
            else if ((362<=y) && (y<700))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[7]=get_colore(col_secondario);
                fan2[7]=get_colore(col_secondario);
                show_leds();
            }
            else if ((700<=y) && (y<874))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[0]=get_colore(col_secondario);
                fan2[0]=get_colore(col_secondario);
                show_leds();
            }
            else
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[0]=fan1[1]=get_colore(col_secondario);
                fan2[0]=fan1[1]=get_colore(col_secondario);
                show_leds();
            }
        }
        else if ((700<=x) && (x<874))
        {
            if (y<362)
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[6]=get_colore(col_secondario);
                fan2[6]=get_colore(col_secondario);
                show_leds();
            }
            else if ((362<=y) && (y<700))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                show_leds();
            }
            else if ((700<=y) && (y<874))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                show_leds();
            }
            else
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[1]=get_colore(col_secondario);
                fan2[1]=get_colore(col_secondario);
                show_leds();
            }
        }
        else if ((362<=x) && (x<700))
        {
            if (y<362)
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[5]=get_colore(col_secondario);
                fan2[5]=get_colore(col_secondario);
                show_leds();
            }
            else if ((362<=y) && (y<700))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                show_leds();
            }
            else if ((700<=y) && (y<874))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                show_leds();
            }
            else
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[2]=get_colore(col_secondario);
                fan2[2]=get_colore(col_secondario);
                show_leds();
            }
        }
        else
        {
            if (y<362)
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[5]=fan1[4]=get_colore(col_secondario);
                fan2[5]=fan1[4]=get_colore(col_secondario);
                show_leds();
            }
            else if ((362<=y) && (y<700))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[4]=get_colore(col_secondario);
                fan2[4]=get_colore(col_secondario);
                show_leds();
            }
            else if ((700<=y) && (y<874))
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[3]=get_colore(col_secondario);
                fan2[3]=get_colore(col_secondario);
                show_leds();
            }
            else
            {
                for (int i=0; i<NUM_LEDS_FAN_SSD ; i++)
                {
                    fan1[i]=get_colore(col_primario);
                    fan2[i]=get_colore(col_primario);
                }
                fan1[3]=fan1[2]=get_colore(col_secondario);
                fan2[3]=fan1[2]=get_colore(col_secondario);
                show_leds();
            }
        }

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            xQueueSend(coda_messaggi, "IL TASK (t_ventole_frontali)      HA ESEGUITO.", 0);
        }
        else
        {
            report_counter++;
        }
        vTaskDelay(50);
    }
}

void t_ventole_corsair (void *pvParameters)
{
    /*  COSA FA:        Gestisce l'effetto di luci delle ventole corsair
        COSA COMUNICA:  Che ha eseguito.
    */  
    int report_counter=0;
    while(1)
    {
        /*  COSA FA:    Colora le ventole corsair per formare un gioco di luci.
        COSA COMUNICA:  Comunica che ha eseguito.
        CHECK:          MANCA CONTROLLO STAMPA
        */  
        for (int i=0 ; i<NUM_LEDS_CORSAIR ; i++)
        {
            corsair[i]=get_colore(2);
        }
        show_leds();
        vTaskDelay(1000);

        for (int i=0 ; i<NUM_LEDS_CORSAIR ; i++)
        {
            corsair[i]=get_colore(3);
        }
        show_leds();
        vTaskDelay(1000);

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            xQueueSend(coda_messaggi, "IL TASK (t_ventole_corsair)       HA ESEGUITO.", 0);
        }
        else
        {
            report_counter++;
        }
    }
}

void t_ssd (void *pvParameters)
{
    int report_counter=0;
    while(1)
    {
        /*  COSA FA:    Colora gli SSD per formare un gioco di luci.
        COSA COMUNICA:  Comunica che ha eseguito.
        */  
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd1[i]=get_colore(4);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd2[i]=get_colore(4);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd3[i]=get_colore(4);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd4[i]=get_colore(4);
            show_leds();
        }

        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd1[i]=get_colore(5);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd2[i]=get_colore(5);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd3[i]=get_colore(5);
            show_leds();
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd4[i]=get_colore(5);
            show_leds();
        }

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            xQueueSend(coda_messaggi, "IL TASK (t_ssd)                   HA ESEGUITO.", 0);
        }
        else
        {
            report_counter++;
        }
    }
}

void t_pompa (void *pvParameters)
{
    int report_counter=0;
    float percentuale;
    String stringa="";
    char messaggio[message_len];
    CRGB colore;
    while(1)
    {   
        /*  COSA FA:    Setta la luce della pompa in base alla temperatura rillevata dal sensore di temperatura.
        COSA COMUNICA:  Percentuale temperatura.
        CHECK:          NON FINITO. MANCA SEMAFORO + STAMPA DEL VALORE LETTO. sem_temperatura
        */ 
        stringa = "IL TASK (t_pompa)                 HA ESEGUITO. Percentuale temperatura: ";
        colore = CRGB(255,255,0);
        xSemaphoreTake(sem_temperatura, portMAX_DELAY);
        percentuale = float((float(temperatura*100.0))/(float(max_temp-min_temp)));
        xSemaphoreGive(sem_temperatura);
        colore.g = int(255 - ((colore.g*percentuale)/100));

        for (int i=0 ; i<NUM_LEDS_PUMP ; i++)
        {
            pump[i]=colore;
        }
        show_leds();

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            stringa = stringa + String(percentuale) + "%";
            stringa.toCharArray(messaggio, message_len);
            xQueueSend(coda_messaggi, (void*)messaggio, 0);
        }
        else
        {
            report_counter++;
        }
        vTaskDelay(500);
    }
}

void t_ventola_posteriore (void *pvParameters)
{
    /*  COSA FA:    Setta la luce della ventola posteriore su un colore fisso.
    COSA COMUNICA:  Che ha eseguito.
    CHECK:          NON FINITO.
    */ 
    int report_counter=0;
    while(1)
    {
        for (int i=0 ; i<NUM_LEDS_R_FAN ; i++)
        {
            fan3[i]=get_colore(8);
        }
        show_leds();
        vTaskDelay(200);

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            xQueueSend(coda_messaggi, "IL TASK (t_ventola_posteriore)    HA ESEGUITO.", 0);
        }
        else
        {
            report_counter++;
        }
    }
}

void t_verifica_accensione (void *pvParameters)
{
    /*  COSA FA:        Controlla se il pulsante principale viene premuto oppure no.
        COSA COMUNICA:  Comunica solo quando il pulsante è premuto.
        CHECK:          NON FINITO.
    */  
    bool on;
    int lettura;
    while(1)
    {
        xSemaphoreTake(sem_is_on, portMAX_DELAY);
        on=is_on;
        xSemaphoreGive(sem_is_on);
        lettura = digitalRead(MAIN_BUTTON);
        
        if (lettura==1)//Se è stato premuto il pulsante
        {
            if(on)
            {
                xSemaphoreTake(sem_is_on, portMAX_DELAY);
                is_on=false;
                xSemaphoreGive(sem_is_on);
                spegni_tutto();
                xQueueSend(coda_messaggi, "IL TASK (t_verifica_accensione)   HA ESEGUITO. Il pulsante è stato premuto: SISTEMA SPENTO.", 0);
            }
            else
            {
                xSemaphoreTake(sem_is_on, portMAX_DELAY);
                is_on=true;
                xSemaphoreGive(sem_is_on);
                xQueueSend(coda_messaggi, "IL TASK (t_verifica_accensione)   HA ESEGUITO. Il pulsante è stato premuto: SISTEMA ACCESO.", 0);
            }
            vTaskDelay(500);//Ritardo per evitare doppie letture
        }
        vTaskDelay(100);
    }
}

void t_verifica_luminosita (void *pvParameters)
{
    /*  COSA FA:        Monitora la luminosità ed aggiorna i colori in tempo reale diminuendone ed aumentandone l'intensità.
        COSA COMUNICA:  Comunica luminosità letta.
        CHECK:          NON FINITO. 
    */  
    int report_counter=0;
    int lettura=0;
    float percentuale=0.0;
    char messaggio[message_len];
    String stringa= "";
    while(1)
    {
        lettura = analogRead(LUM_PIN);
        stringa = "IL TASK (t_verifica_luminosita)   HA ESEGUITO. Luminosità letta: ";

        if (lettura>max_lum)
        {
            lettura=max_lum;
        }
        if (lettura<min_lum)
        {
            lettura=min_lum;
        }
        
        lettura=lettura-min_lum;
        percentuale=float((lettura*100)/(max_lum-min_lum));

        xSemaphoreTake(sem_read_color, portMAX_DELAY);
        for(int i=0 ; i<10 ; i++)//Setta tutti i colori secondo la nuova percentuale.
        {
            CRGB nuovo_colore = get_colore_BASE(i);
            nuovo_colore.r = (nuovo_colore.r*percentuale)/100;
            nuovo_colore.g = (nuovo_colore.g*percentuale)/100;
            nuovo_colore.b = (nuovo_colore.b*percentuale)/100;
            set_colore(i, nuovo_colore.r, nuovo_colore.g, nuovo_colore.b);
        }
        xSemaphoreGive(sem_read_color);

        if (report_counter>=max_report_counter)
        {
            report_counter=0;
            stringa = stringa + String(lettura) + " ; Percentuale letta: " + String(percentuale);
            stringa.toCharArray(messaggio, message_len);
            xQueueSend(coda_messaggi, (void*)messaggio, 0);
        }
        else
        {
            report_counter++;
        }
        vTaskDelay(50);
    }
}

void t_verifica_temperatura (void *pvParameters)
{
    /*  COSA FA:        Monitora la temperatura ed aggiorna la variabile globale "temperatura".
        COSA COMUNICA:  Comunica temperatura letta. Se non ci riesce comunica errore.
        CHECK:          NON FINITO.
    */  
    int report_counter= 0; 
    float umid= 0.0;
    float temp= 0.0;
    float heat_idx= 0.0;
    char messaggio[message_len];
    String stringa= "";
    while(1)
    {
        umid = therm.readHumidity();
        temp = therm.readTemperature();
        stringa="IL TASK (t_verifica_temperatura)  HA ESEGUITO. Temperatura letta: ";

        if (!(isnan(umid)||isnan(temp)))//Se riesce a leggere sovrascrive la variabile globale.
        {
            heat_idx = therm.computeHeatIndex(temp, umid, false);
            if (heat_idx>max_temp)
            {
                heat_idx=max_temp;
            }
            if (heat_idx<min_temp)
            {
                heat_idx=min_temp;
            }
            xSemaphoreTake(sem_temperatura,portMAX_DELAY);
            temperatura = heat_idx;
            xSemaphoreGive(sem_temperatura);
        }

        if (report_counter>=max_report_counter)
        {
            if (isnan(umid)||isnan(temp))
            {
                stringa = stringa + "ERRORE LETTURA.";
                stringa.toCharArray(messaggio, message_len);
                report_counter=0;
                xQueueSend(coda_messaggi, (void*)messaggio, 0);
            }
            else
            {
                stringa = stringa + String(heat_idx);
                stringa.toCharArray(messaggio, message_len);
                report_counter=0;
                xQueueSend(coda_messaggi, (void*)messaggio, 0);
            }
        }
        else
        {
            report_counter++;
        }
        vTaskDelay(1000);
    }
}

void t_verifica_pulsante_joy (void *pvParameters)
{
    /*  COSA FA:        Controlla se il pulsante del joystick viene premuto oppure no.
        COSA COMUNICA:  Comunica solo quando il pulsante è premuto.
        CHECK:          NON FINITO.
    */  
    //MANCA SEMAFOROOOOOOOOOOOOOOOOOOOOOOOOO sem_colori_invertiti
    bool invertito= false;
    int lettura= 0;
    char messaggio[message_len];
    while(1)
    {
        xSemaphoreTake(sem_colori_invertiti, portMAX_DELAY);
        invertito= joy_colori_invertiti;
        xSemaphoreGive(sem_colori_invertiti);

        lettura= bool(!digitalRead(JOYSTICK_BUTTON));
        if (lettura==1)//Se è stato premuto il pulsante
        {
            if(invertito)
            {
                xSemaphoreTake(sem_colori_invertiti, portMAX_DELAY);
                joy_colori_invertiti=false;
                xSemaphoreGive(sem_colori_invertiti);
                xQueueSend(coda_messaggi, "IL TASK (t_verifica_pulsante_joy) HA ESEGUITO. Il pulsante è stato premuto: COLORI NORMALI.", 0);
            }
            else
            {
                xSemaphoreTake(sem_colori_invertiti, portMAX_DELAY);
                joy_colori_invertiti=true;
                xSemaphoreGive(sem_colori_invertiti);
                xQueueSend(coda_messaggi, "IL TASK (t_verifica_pulsante_joy) HA ESEGUITO. Il pulsante è stato premuto: COLORI INVERTITI.", 0);
            }
            vTaskDelay(500);//Ritardo per evitare doppie letture
        }
        vTaskDelay(50);
    }
}

void t_communicator (void *pvParameters)
{
    /*  COSA FA:        Stampa su seriale tutti i messaggi che riceve. È l'unica task MANY-TO-ONE del progetto.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */
    while(1)
    {
        char messaggio[message_len];
        int res = xQueueReceive(coda_messaggi, &messaggio, 0);
        if ((res==1) && (!supress_communicator))
        {
            Serial.println(messaggio);
        }
        vTaskDelay(300);
    }
}

CRGB get_colore_BASE(int colore)
{
    /*  COSA FA:        Ritorna il colore base richiesto. Questa funzione è utilizzata solo dal task che gestisce la luminosità.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */
    switch (colore)
    {
        case 0:
        {
            return colore0_BASE;
            break;
        }
        case 1:
        {
            return colore1_BASE;
            break;
        }
        case 2:
        {
            return colore2_BASE;
            break;
        }
        case 3:
        {
            return colore3_BASE;
            break;
        }
        case 4:
        {
            return colore4_BASE;
            break;
        }
        case 5:
        {
            return colore5_BASE;
            break;
        }
        case 6:
        {
            return colore6_BASE;
            break;
        }
        case 7:
        {
            return colore7_BASE;
            break;
        }
        case 8:
        {
            return colore8_BASE;
            break;
        }
        case 9:
        {
            return colore9_BASE;
            break;
        }
        default:
        {
            return spento;
            break;
        }
    }
}

CRGB get_colore(int colore)
{
    /*  COSA FA:        Ritorna il colore richiesto.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */
    switch (colore)
    {
        case 0:
        {
            return colore0;
            break;
        }
        case 1:
        {
            return colore1;
            break;
        }
        case 2:
        {
            return colore2;
            break;
        }
        case 3:
        {
            return colore3;
            break;
        }
        case 4:
        {
            return colore4;
            break;
        }
        case 5:
        {
            return colore5;
            break;
        }
        case 6:
        {
            return colore6;
            break;
        }
        case 7:
        {
            return colore7;
            break;
        }
        case 8:
        {
            return colore8;
            break;
        }
        case 9:
        {
            return colore9;
            break;
        }
        case 99:
        {
            return spento;
            break;
        }
        default:
        {
            return spento;
            break;
        }
    }
}

void set_colore(int colore, int red, int green, int blue)
{
    /*  COSA FA:        Serve al task che regola la luminosità per aggiornare i colori in base, appunto, alla luminosità.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */  
    if(colore==0)
    {
        colore0.r=red;
        colore0.g=green;
        colore0.b=blue;
    }
    else if(colore==1)
    {
        colore1.r=red;
        colore1.g=green;
        colore1.b=blue;
    }
    else if(colore==2)
    {
        colore2.r=red;
        colore2.g=green;
        colore2.b=blue;
    }
    else if(colore==3)
    {
        colore3.r=red;
        colore3.g=green;
        colore3.b=blue;
    }
    else if(colore==4)
    {
        colore4.r=red;
        colore4.g=green;
        colore4.b=blue;
    }
    else if(colore==5)
    {
        colore5.r=red;
        colore5.g=green;
        colore5.b=blue;
    }
    else if(colore==6)
    {
        colore6.r=red;
        colore6.g=green;
        colore6.b=blue;
    }
    else if(colore==7)
    {
        colore7.r=red;
        colore7.g=green;
        colore7.b=blue;
    }
    else if(colore==8)
    {
        colore8.r=red;
        colore8.g=green;
        colore8.b=blue;
    }
    else
    {
        colore9.r=red;
        colore9.g=green;
        colore9.b=blue;
    }
}

void show_leds(void)
{
    /*  COSA FA:        Applica la funzione FASTLED che mostra i colori dei led dell'intero sistema.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */  
    xSemaphoreTake(sem_is_on, portMAX_DELAY);
    bool on=is_on;
    xSemaphoreGive(sem_is_on);

    if (on)
    {
        xSemaphoreTake(sem_show, portMAX_DELAY);
        FastLED.show();
        xSemaphoreGive(sem_show);
        vTaskDelay(ritardo);
    }
    else
    {
        vTaskDelay(ritardo);
    }
}

void show_leds_spegni_tutto(void)
{
    /*  COSA FA:        Identico a show_leds ma non controlla la variabile globale prima di effettuare la modifica. Questa funzione è usata solo da SPEGNI_TUTTO.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */  
    xSemaphoreTake(sem_show, portMAX_DELAY);
    FastLED.show();
    xSemaphoreGive(sem_show);
    vTaskDelay(ritardo);
}

void spegni_tutto(void)
{
    /*  COSA FA:        Spegne i led dell'intero sistema.
        COSA COMUNICA:  Nulla.
        CHECK:          NON FINITO.
    */  
    vTaskDelay(50);
    for (int i=0 ; i<3 ; i++)
    {
        for (int i=0 ; i<NUM_LEDS_CORSAIR ; i++)
        {
            corsair[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            fan1[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            fan2[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_R_FAN ; i++)
        {
            fan3[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd1[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd2[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd3[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_FAN_SSD ; i++)
        {
            ssd4[i]=spento;
        }
        for (int i=0 ; i<NUM_LEDS_PUMP ; i++)
        {
            pump[i]=spento;
        }
        show_leds_spegni_tutto();
        vTaskDelay(25);
    }
}
