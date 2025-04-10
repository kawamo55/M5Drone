
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <SPIFFS.h>

//
#define CHANNEL 1
#define ANGLECONTROL         0
#define RATECONTROL          1
#define ANGLECONTROL_W_LOG   2
#define RATECONTROL_W_LOG    3
#define ALT_CONTROL_MODE     4
#define NOT_ALT_CONTROL_MODE 5
#define RESO10BIT            (4096)

#define INIT_MODE         0
#define AVERAGE_MODE      1
#define FLIGHT_MODE       2
#define PARKING_MODE      3
#define LOG_MODE          4
#define AUTO_LANDING_MODE 5

// StampFly MAC ADDRESS
// 1 F4:12:FA:66:80:54 (Yellow)
// 2 F4:12:FA:66:77:A4
uint8_t Addr1[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t Addr2[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//
uint8_t senddata[25];  // 19->22->23->24->25

esp_now_peer_info_t peerInfo;

uint8_t Ch_counter;

volatile uint8_t is_peering                = 0;
volatile uint8_t Received_flag             = 0;
volatile uint8_t Channel                   = CHANNEL;

volatile float fly_bat_voltage             = 0.0f;
volatile float roll_angle                  = 0.0f;
volatile float pitch_angle                 = 0.0f;
volatile float yaw_angle                   = 0.0f;
volatile float altitude                    = 0.0f;

//
volatile uint8_t auto_up_down_status          = 0;
volatile uint32_t auto_up_down_status_counter = 0;
volatile uint8_t proactive_flag          = 0;
volatile uint32_t proactive_flag_counter = 0;
volatile uint32_t recive_cnt = 0;

//
volatile int16_t tof_front                 = 0.0;
volatile uint8_t is_fly_flag               = 0;
volatile unsigned long is_fly_flag_counter = 0;

volatile uint8_t fly_status                = 0;
volatile uint8_t fly_status_manual         = 0;
volatile uint8_t alt_flag                  = 0;
volatile uint8_t fly_mode                  = 0;
volatile uint8_t last_fly_mode             = 0;

#define LKEYBUFMAX 512
int keyBuf_Pointer=0;
char keyBuffer[LKEYBUFMAX+2];
int takeoff=0;


void Tcb(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len)
{
  Serial.print("Call back function");
}


void OnDataRecv(const uint8_t *mac_addr, const uint8_t *recv_data, int data_len) {
    if (is_peering) {
        if (recv_data[7] == 0xaa && recv_data[8] == 0x55 && recv_data[9] == 0x16 && recv_data[10] == 0x88) {
            Received_flag = 1;
            Channel       = recv_data[0];
            Addr2[0]      = recv_data[1];
            Addr2[1]      = recv_data[2];
            Addr2[2]      = recv_data[3];
            Addr2[3]      = recv_data[4];
            Addr2[4]      = recv_data[5];
            Addr2[5]      = recv_data[6];
            Serial.printf("Receive !\n");
        }
    } else {
        if (recv_data[0] == 88 && recv_data[1] == 88) {
            memcpy((uint8_t *)&roll_angle, &recv_data[2 + 4 * (3 - 1)], 4);
            memcpy((uint8_t *)&pitch_angle, &recv_data[2 + 4 * (4 - 1)], 4);
            memcpy((uint8_t *)&yaw_angle, &recv_data[2 + 4 * (5 - 1)], 4);
            memcpy((uint8_t *)&fly_bat_voltage, &recv_data[2 + 4 * (15 - 1)], 4);
            memcpy((uint8_t *)&altitude, &recv_data[2 + 4 * (25 - 1)], 4);
            alt_flag = recv_data[2 + 4 * (28 - 1)];
            fly_mode = recv_data[2 + 4 * (28 - 1) + 1];
            memcpy((uint8_t *)&tof_front, &recv_data[2 + 4 * (28 - 1) + 2], 2);
            is_fly_flag = 1;
            if (fly_mode == PARKING_MODE) {
                fly_status_manual = 0;
            }
            recive_cnt++;
            /*
            if (recive_cnt > 20) {
              Serial.printf("Receive Status! --voltage[%f]]\n",fly_bat_voltage);
              recive_cnt = 0;
            }
            */
            // USBSerial.printf("roll:%.2f, pitch:%.2f, yaw:%.2f, voltage:%.2f, alt_flag:%d, fly_mode:%d,
            // tof_front:%d\r\n", roll_angle, pitch_angle, yaw_angle, fly_bat_voltage, alt_flag, fly_mode, tof_front);
        }
    }
}

void rc_init(uint8_t ch, uint8_t *addr) {
    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    if (esp_now_init() == ESP_OK) {
        esp_now_unregister_recv_cb();
        esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
//        esp_now_register_recv_cb((esp_now_recv_cb_t)Tcb);
//        esp_now_register_recv_cb(Tcb);
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }

    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, addr, 6);
    Serial.printf("Channel=%d\n",ch);
    peerInfo.channel = ch;
    peerInfo.encrypt = false;
    uint8_t peer_mac_addre;
    while (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
    }
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

void beep() {
  Serial.println("---Beep()");
}

void buzzer_sound(uint32_t frequency, uint32_t duration_ms) {
  Serial.println("---Buzzer_sound()");
}

#define BUF_SIZE 128
// EEPROMにデータを保存する
void save_data(void) {
    if (!SPIFFS.begin()) {
        // 初始化失败时处理
        Serial.println("SPIFFS-An error occurred while mounting SPIFFS");

        // 格式化SPIFFS分区
        if (SPIFFS.format()) {
            // 格式化成功
            Serial.println("SPIFFS partition formatted successfully");
            // 重启
            esp_restart();
        } else {
            Serial.println("SPIFFS partition format failed");
        }
        esp_restart();
    } else {
        /* CREATE FILE */
        File fp = SPIFFS.open("/peer_info.txt", FILE_WRITE);  // 書き込み、存在すれば上書き
        char buf[BUF_SIZE + 1];
        sprintf(buf, "%d,%02X,%02X,%02X,%02X,%02X,%02X", Channel, Addr2[0], Addr2[1], Addr2[2], Addr2[3], Addr2[4], Addr2[5]);
        fp.write((uint8_t *)buf, BUF_SIZE);
        fp.close();
        SPIFFS.end();

        Serial.printf("Saved Data:%d,[%02X:%02X:%02X:%02X:%02X:%02X]", Channel, Addr2[0], Addr2[1], Addr2[2], Addr2[3], Addr2[4], Addr2[5]);
    }
}

// EEPROMからデータを読み出す
void load_data(void) {
    SPIFFS.begin(true);
    File fp = SPIFFS.open("/peer_info.txt", FILE_READ);
    char buf[BUF_SIZE + 1];
    while (fp.read((uint8_t *)buf, BUF_SIZE) == BUF_SIZE) {
        // USBSerial.print(buf);
        sscanf(buf, "%hhd,%hhX,%hhX,%hhX,%hhX,%hhX,%hhX", &Channel, &Addr2[0], &Addr2[1], &Addr2[2], &Addr2[3], &Addr2[4], &Addr2[5]);
        Serial.printf("%d,%02X,%02X,%02X,%02X,%02X,%02X\n\r", Channel, Addr2[0], Addr2[1], Addr2[2], Addr2[3], Addr2[4], Addr2[5]);
    }
    fp.close();
    SPIFFS.end();
}


void send_data_now(void) {
  esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, sizeof(senddata));
}

void peering(void) {
    uint8_t break_flag;
    uint32_t beep_delay = 0;
    char dis_buff[100]  = {0};
    // StampFlyはMACアドレスをFF:FF:FF:FF:FF:FFとして
    // StampFlyのMACアドレスをでブロードキャストする
    // その際にChannelが機体と送信機で同一でない場合は受け取れない
    //  ESP-NOWコールバック登録
    esp_now_unregister_recv_cb();
//    esp_now_register_recv_cb((esp_now_recv_cb_t)OnDataRecv);
    esp_now_register_recv_cb(OnDataRecv);
//    esp_now_register_recv_cb(Tcb);

    // ペアリング
    Ch_counter = 1;
    while (1) {
        delay(100);
        Serial.printf("Try channel %02d.\n\r", Ch_counter);
        peerInfo.channel = Ch_counter;
        peerInfo.encrypt = false;
        while (esp_now_mod_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to mod peer");
        }
        esp_wifi_set_channel(Ch_counter, WIFI_SECOND_CHAN_NONE);

        // Wait receive StampFly MAC Address
        uint32_t counter=1;
        // Channelをひとつづつ試していく
        for (uint8_t i = 0; i < 100; i++) {
            break_flag = 0;
            if (Received_flag == 1) {
                break_flag = 1;
                break;
            }
            usleep(100);
        }
        if (millis() - beep_delay >= 500) {
            beep();
            beep_delay = millis();
        }
        if (break_flag) break;
        Ch_counter++;
        if (Ch_counter == 15) Ch_counter = 1;
    }
    // Channel = Ch_counter;

    save_data();
    is_peering = 0;
    buzzer_sound(4000, 600);
    delay(1000);

    Serial.printf("Channel:%02d\n\r", Channel);
    Serial.printf("MAC2:%02X:%02X:%02X:%02X:%02X:%02X:\n\r", Addr2[0], Addr2[1], Addr2[2], Addr2[3], Addr2[4], Addr2[5]);
    Serial.printf("MAC1:%02X:%02X:%02X:%02X:%02X:%02X:\n\r", Addr1[0], Addr1[1], Addr1[2], Addr1[3], Addr1[4], Addr1[5]);

    // Peering
    while (esp_now_del_peer(Addr1) != ESP_OK) {
        Serial.println("Failed to delete peer1");
    }
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, Addr2, 6);  // Addr1->Addr2 ////////////////////////////
    peerInfo.channel = Channel;
    peerInfo.encrypt = false;
    while (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer2");
    }
    esp_wifi_set_channel(Channel, WIFI_SECOND_CHAN_NONE);
    esp_restart();
}

void change_channel(uint8_t ch) {
    peerInfo.channel = ch;
    if (esp_now_mod_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to modify peer");
        return;
    }
    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
}

void setup() {

  Serial.begin(9600);
  is_peering=0;
  // put your setup code here, to run once:
  // SPIFFS.format(); 
  Serial.println("Start");
  load_data();
  // Serial.println("Loaded");
  if   (Addr2[0] == 0xFF && Addr2[1] == 0xFF && Addr2[2] == 0xFF && Addr2[3] == 0xFF && Addr2[4] == 0xFF && Addr2[5] == 0xFF) {
    // USBSerial.println("-->");
    Serial.println("--Setup");
    rc_init(1,Addr1);
    delay(10);
    is_peering=1;
    peering();
  } else {
    rc_init(Channel,Addr2);
  }
}

int getChar( ){
  int ch,lc;
  char cbuf[4];

  keyBuf_Pointer=0;
  ch=Serial.peek();
  lc=ch;
  while(ch > -1) {
    Serial.readBytes(cbuf,1);
    cbuf[1]='\0';
    if (keyBuf_Pointer < LKEYBUFMAX) {
      keyBuffer[keyBuf_Pointer]=cbuf[0];
      keyBuf_Pointer++;
    }
    lc=ch;
    ch=Serial.peek();
  }
  keyBuffer[keyBuf_Pointer]='\0';
  if (keyBuf_Pointer > 0)
    Serial.println(keyBuffer);
  return lc;
}

void set_send_data(float Psi, float Throttle,float Phi,float Theta,uint8_t auto_up_down_status,uint8_t flip_button,uint8_t Mode,uint8_t AltMode,uint8_t proactive_flag)
{
      uint8_t *d_int;

    // ブロードキャストの混信を防止するためこの機体のMACアドレスに送られてきたものか判断する
    senddata[0] = peerInfo.peer_addr[3];  ////////////////////////////
    senddata[1] = peerInfo.peer_addr[4];  ////////////////////////////
    senddata[2] = peerInfo.peer_addr[5];  ////////////////////////////

    d_int       = (uint8_t *)&Psi;
    senddata[3] = d_int[0];
    senddata[4] = d_int[1];
    senddata[5] = d_int[2];
    senddata[6] = d_int[3];

    d_int        = (uint8_t *)&Throttle;
    senddata[7]  = d_int[0];
    senddata[8]  = d_int[1];
    senddata[9]  = d_int[2];
    senddata[10] = d_int[3];

    d_int        = (uint8_t *)&Phi;
    senddata[11] = d_int[0];
    senddata[12] = d_int[1];
    senddata[13] = d_int[2];
    senddata[14] = d_int[3];

    d_int        = (uint8_t *)&Theta;
    senddata[15] = d_int[0];
    senddata[16] = d_int[1];
    senddata[17] = d_int[2];
    senddata[18] = d_int[3];

    senddata[19] = auto_up_down_status;
    senddata[20] = flip_button;
    senddata[21] = Mode;
    senddata[22] = AltMode;

    senddata[23] = proactive_flag;

    // checksum
    senddata[24] = 0;
    for (uint8_t i = 0; i < 24; i++) senddata[24] = senddata[24] + senddata[i];
    //
    esp_err_t result = esp_now_send(peerInfo.peer_addr, senddata, sizeof(senddata));
    // Serial.printf("--Send Status[%d]\n",result);
    //
    if (auto_up_down_status) {
        auto_up_down_status_counter++;
        if (auto_up_down_status_counter > 20) {
            auto_up_down_status_counter = 0;
            auto_up_down_status         = 0;
        }
    }
    if (proactive_flag) {
        proactive_flag_counter++;
        if (proactive_flag_counter > 20) {
            proactive_flag_counter = 0;
            proactive_flag         = 0;
        }
    }
    if (is_fly_flag) {
        is_fly_flag_counter++;
        if (is_fly_flag_counter > 2000) {
            is_fly_flag_counter = 0;
            is_fly_flag         = 0;
        }
    }

}

void loop() {
  int cc=0;
  // put your main code here, to run repeatedly:
  if (getChar()>0) {
    if (!strcmp(keyBuffer,"prst\n")) {
      Serial.println("Reset Mac addr!!");
      memset(Addr2, 0xFF, 6);
      save_data();
      esp_restart();
    }
    //
    //       set_send_data(float Psi, float Throttle,float Phi,float Theta,uint8_t auto_up_down_status,uint8_t flip_button,uint8_t Mode,uint8_t AltMode,uint8_t proactive_flag)
    //       set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //
    if (!strcmp(keyBuffer,"takeoff\n")) {
      // Serial.println("--Send command");
      // Serial.printf("RA:%8.4f PA:%8.4f YA:%8.4f AL:%8.4f\n",roll_angle ,pitch_angle, yaw_angle, altitude);
      set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
      delay(45);
      // set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
      for(cc=0; cc<20; cc++) {
        delay(40);
        // Serial.printf("RA:%8.4f PA:%8.4f YA:%8.4f AL:%8.4f\n",roll_angle ,pitch_angle, yaw_angle, altitude);
        set_send_data(0.0f ,0.5f ,0.0f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
      }
      //
      Serial.printf("RA:%8.4f PA:%8.4f YA:%8.4f AL:%8.4f\n",roll_angle ,pitch_angle, yaw_angle, altitude);
      takeoff=1;
    }
    if (!strcmp(keyBuffer,"up10\n") and takeoff) {
      set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
      delay(45);
      for(cc=0; cc<10; cc++) {
        delay(40);
        // Serial.printf("RA:%8.4f PA:%8.4f YA:%8.4f AL:%8.4f\n",roll_angle ,pitch_angle, yaw_angle, altitude);
        set_send_data(0.0f ,0.5f ,0.0f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
      }
    }

    if (!strcmp(keyBuffer,"fend\n")) {
      set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0);
      takeoff=0;
      Serial.println("--Send End");
    }
    if (!strcmp(keyBuffer,"rsta\n")) {
      Serial.printf("RA:%8.4f PA:%8.4f YA:%8.4f AL:%8.4f\n",roll_angle ,pitch_angle, yaw_angle, altitude);
    }
    // template keyCommand
    /*
    if (!strcmp(keyBuffer,"\n")) {

    }
    */
    Serial.println("--Loop");
    Serial.printf("V:%8f AL%8.4f\n",fly_bat_voltage, altitude);
    Serial.printf("No1,%02X,%02X,%02X,%02X,%02X,%02X\n\r", Addr1[0], Addr1[1], Addr1[2], Addr1[3], Addr1[4], Addr1[5]);
    Serial.printf("%d,%02X,%02X,%02X,%02X,%02X,%02X\n\r", Channel, Addr2[0], Addr2[1], Addr2[2], Addr2[3], Addr2[4], Addr2[5]);
  } else {
      if (takeoff > 0) {
        set_send_data(0.0f ,0.0f ,0.0f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0, (uint8_t)0);
        delay(50);
        for(cc=0; cc<4; cc++) {
    //    set_send_data(float Psi, float Throttle,float Phi,float Theta,uint8_t auto_up_down_status,uint8_t flip_button,uint8_t Mode,uint8_t AltMode,uint8_t proactive_flag)
    //      set_send_data(0.05f ,0.5f ,0.01f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   left
    //      set_send_data(0.00f ,0.5f ,0.0f ,0.5f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   right
    //      set_send_data(0.00f ,0.5f ,0.5f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   back
    //      set_send_data(0.05f ,0.5f ,-0.2f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   fwd
    //      set_send_data(0.3f ,0.5f ,0.0f ,0.0f ,(uint8_t)0, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   fwd
          set_send_data(0.2f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
          delay(110);
          set_send_data(0.0f ,0.2f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
    //   bak
    //      set_send_data(-0.1f ,0.0f ,0.0f ,0.0f ,(uint8_t)1, (uint8_t)0, (uint8_t)2, (uint8_t)5, (uint8_t)0);
          delay(110);
        }
        Serial.println("--hob");
      }
    }
}

