#include <M5Cardputer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <PCMFlow.h>
#include <PCMFlowG722.h>

String inputLine = "";
String history[20];
int historyCount = 0;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

String username = "";
int currentChannel = 1;

volatile bool beepPending = false;
volatile bool redrawPending = false;

bool pttActive = false;

G722Encoder enc;
G722Decoder dec;

constexpr int FRAME_SAMPLES = 320;
constexpr int FRAME_BYTES = 160;
int16_t pcmBuf[FRAME_SAMPLES];
uint8_t encodedBuf[FRAME_BYTES];

#define AUDIO_QUEUE_SIZE 16
int16_t audioQueue[AUDIO_QUEUE_SIZE][FRAME_SAMPLES];
volatile int audioQueueHead = 0;
volatile int audioQueueTail = 0;

constexpr uint8_t PKT_TEXT = 0x01;
constexpr uint8_t PKT_AUDIO = 0x02;

volatile int txAudioCount = 0;
volatile int rxAudioCount = 0;
volatile int txFailCount = 0;
volatile int txTextCount = 0;
volatile int rxTextCount = 0;

volatile int decodePeak = 0;
volatile int playPeak = 0;
volatile int lastEncodedLen = 0;

unsigned long lastAudioSendTime = 0;

// --- 播放节奏控制 ---
unsigned long lastPlayTime = 0;
constexpr int PLAY_INTERVAL_MS = 20;

unsigned long lastCounterUpdateTime = 0;
int lastShownTxAudio = -1;
int lastShownRxAudio = -1;
int lastShownDecodePeak = -1;
int lastShownPlayPeak = -1;

void addMessage(String msg)
{
  if (historyCount < 20)
  {
    history[historyCount++] = msg;
  }
  else
  {
    for (int i = 0; i < 19; i++)
    {
      history[i] = history[i + 1];
    }
    history[19] = msg;
  }
}

bool handleCommand(String cmd)
{
  cmd.trim();

  if (cmd == "/user")
  {
    addMessage("Username: " + username);
    return true;
  }

  if (cmd.startsWith("/user "))
  {
    String newName = cmd.substring(6);
    newName.trim();
    if (newName.length() > 0)
    {
      username = newName;
      addMessage("Username -> " + username);
    }
    return true;
  }

  if (cmd == "/channel")
  {
    addMessage("Channel: " + String(currentChannel));
    return true;
  }

  if (cmd.startsWith("/channel "))
  {
    String arg = cmd.substring(9);
    arg.trim();
    int ch = arg.toInt();
    if (ch >= 1 && ch <= 13)
    {
      currentChannel = ch;
      esp_wifi_set_promiscuous(true);
      esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
      esp_wifi_set_promiscuous(false);
      addMessage("Channel -> " + String(currentChannel));
    }
    return true;
  }

  addMessage("Unknown cmd: " + cmd);
  return true;
}

void updateCounters()
{
  int baseY = M5Cardputer.Display.height() - 40;

  M5Cardputer.Display.fillRect(
    0, baseY, M5Cardputer.Display.width(), 26, BLACK);

  M5Cardputer.Display.drawString(
    "TXa:" + String(txAudioCount) + " RXa:" + String(rxAudioCount),
    2, baseY + 12);

  M5Cardputer.Display.drawString(
    "decPk:" + String(decodePeak) + " playPk:" + String(playPeak),
    2, baseY);
}

void redraw()
{
  M5Cardputer.Display.fillScreen(BLACK);

  const int lineHeight = 10;
  int maxVisible = (M5Cardputer.Display.height() - 42) / lineHeight;
  int startIndex = 0;

  if (historyCount > maxVisible)
  {
    startIndex = historyCount - maxVisible;
  }

  int y = 2;
  for (int i = startIndex; i < historyCount; i++)
  {
    M5Cardputer.Display.drawString(history[i], 2, y);
    y += lineHeight;
  }

  M5Cardputer.Display.fillRect(
    0, M5Cardputer.Display.height() - 14,
    M5Cardputer.Display.width(), 14, DARKGREY);

  M5Cardputer.Display.drawString(
    ">" + inputLine, 2, M5Cardputer.Display.height() - 12);

  if (pttActive)
  {
    M5Cardputer.Display.fillRect(
      M5Cardputer.Display.width() - 40, 2, 38, 12, RED);
    M5Cardputer.Display.drawString(
      "TX", M5Cardputer.Display.width() - 34, 3);
  }
  else
  {
    M5Cardputer.Display.fillRect(
      M5Cardputer.Display.width() - 40, 2, 38, 12, DARKGREEN);
    M5Cardputer.Display.drawString(
      "RX", M5Cardputer.Display.width() - 34, 3);
  }

  updateCounters();
}

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len)
{
  if (len < 1) return;

  uint8_t pktType = incomingData[0];

  if (pktType == PKT_TEXT)
  {
    rxTextCount++;
    String msg = "";
    for (int i = 1; i < len; i++)
    {
      msg += (char)incomingData[i];
    }
    addMessage(msg);
    beepPending = true;
    redrawPending = true;
  }
  else if (pktType == PKT_AUDIO)
  {
    rxAudioCount++;

    if (!pttActive && len > 1)
    {
      int audioLen = len - 1;
      if (audioLen <= FRAME_BYTES)
      {
        int nextHead = (audioQueueHead + 1) % AUDIO_QUEUE_SIZE;
        if (nextHead != audioQueueTail)
        {
          dec.decode((uint8_t*)&incomingData[1], audioLen,
                     pcmBuf, FRAME_SAMPLES);

          int16_t peak = 0;
          for (int i = 0; i < FRAME_SAMPLES; i++)
          {
            int16_t v = pcmBuf[i];
            if (v < 0) v = -v;
            if (v > peak) peak = v;
          }
          decodePeak = peak;

          memcpy(audioQueue[audioQueueHead], pcmBuf, sizeof(pcmBuf));
          audioQueueHead = nextHead;
        }
      }
    }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

void setup()
{
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);

  M5Cardputer.Speaker.begin();

  enc.begin({16000, 1, 16});
  dec.begin({16000, 1, 16});

  WiFi.mode(WIFI_STA);
  delay(100);

  String mac = WiFi.macAddress();
  int retry = 0;
  while (mac == "00:00:00:00:00:00" && retry < 10)
  {
    delay(100);
    mac = WiFi.macAddress();
    retry++;
  }
  mac.replace(":", "");
  username = mac.substring(mac.length() - 4);

  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);

  if (esp_now_init() != ESP_OK)
  {
    M5Cardputer.Display.println("ESP-NOW FAIL");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  addMessage("ESP-NOW CHAT READY");
  addMessage("CH:" + String(currentChannel));
  addMessage("MAC:" + WiFi.macAddress());

  redraw();
}

void loop()
{
  M5Cardputer.update();

  if (beepPending)
  {
    beepPending = false;
    M5Cardputer.Speaker.tone(2000, 100);
  }

  if (redrawPending)
  {
    redrawPending = false;
    redraw();
  }

  // --- 20ms 定时播放，与 loop() 循环速度解耦 ---
  if (!pttActive && audioQueueTail != audioQueueHead)
  {
    if (millis() - lastPlayTime >= PLAY_INTERVAL_MS)
    {
      lastPlayTime = millis();

      int16_t pp = 0;
      for (int i = 0; i < FRAME_SAMPLES; i++)
      {
        int16_t v = audioQueue[audioQueueTail][i];
        if (v < 0) v = -v;
        if (v > pp) pp = v;
      }
      playPeak = pp;

      M5Cardputer.Speaker.playRaw(
        audioQueue[audioQueueTail], FRAME_SAMPLES, 16000, false, 1, 0);

      audioQueueTail = (audioQueueTail + 1) % AUDIO_QUEUE_SIZE;
    }
  }

  if (millis() - lastCounterUpdateTime > 200)
  {
    lastCounterUpdateTime = millis();
    if (txAudioCount != lastShownTxAudio ||
        rxAudioCount != lastShownRxAudio ||
        decodePeak != lastShownDecodePeak ||
        playPeak != lastShownPlayPeak)
    {
      updateCounters();
      lastShownTxAudio = txAudioCount;
      lastShownRxAudio = rxAudioCount;
      lastShownDecodePeak = decodePeak;
      lastShownPlayPeak = playPeak;
    }
  }

  Keyboard_Class::KeysState status =
    M5Cardputer.Keyboard.keysState();

  bool tabNow = status.tab;

  if (tabNow && !pttActive)
  {
    pttActive = true;
    M5Cardputer.Speaker.end();
    delay(10);
    M5Cardputer.Mic.begin();
    lastAudioSendTime = millis();
    redraw();
  }

  if (!tabNow && pttActive)
  {
    pttActive = false;
    M5Cardputer.Mic.end();
    delay(10);
    M5Cardputer.Speaker.begin();
    redraw();
  }

  // PTT 发送中：采集 → 放大 → VAD → 编码 → 发送
  if (pttActive)
  {
    if (millis() - lastAudioSendTime >= 20)
    {
      if (M5Cardputer.Mic.record(pcmBuf, FRAME_SAMPLES, 16000))
      {
        constexpr int GAIN_MULTIPLIER = 10;
        for (int i = 0; i < FRAME_SAMPLES; i++)
        {
          int32_t v = (int32_t)pcmBuf[i] * GAIN_MULTIPLIER;
          if (v > 32767) v = 32767;
          if (v < -32768) v = -32768;
          pcmBuf[i] = (int16_t)v;
        }

        int16_t peak = 0;
        for (int i = 0; i < FRAME_SAMPLES; i++)
        {
          int16_t v = pcmBuf[i];
          if (v < 0) v = -v;
          if (v > peak) peak = v;
        }

        constexpr int VAD_THRESHOLD = 500;

        if (peak < VAD_THRESHOLD)
        {
          lastAudioSendTime = millis();
        }
        else
        {
          int encodedLen = enc.encode(pcmBuf, FRAME_SAMPLES, encodedBuf, FRAME_BYTES);

          if (encodedLen > 0)
          {
            lastEncodedLen = encodedLen;

            uint8_t packet[FRAME_BYTES + 1];
            packet[0] = PKT_AUDIO;
            memcpy(&packet[1], encodedBuf, encodedLen);

            esp_err_t result = esp_now_send(broadcastAddress, packet, encodedLen + 1);

            if (result == ESP_OK)
            {
              txAudioCount++;
            }
            else
            {
              txFailCount++;
            }

            lastAudioSendTime = millis();
          }
        }
      }
    }
  }

  if (M5Cardputer.Keyboard.isChange())
  {
    for (auto c : status.word)
    {
      inputLine += c;
    }

    if (status.del)
    {
      if (inputLine.length() > 0)
      {
        inputLine.remove(inputLine.length() - 1);
      }
    }

    if (status.enter)
    {
      if (inputLine.length() > 0)
      {
        if (inputLine.startsWith("/"))
        {
          handleCommand(inputLine);
        }
        else
        {
          String packet = username + ": " + inputLine;
          addMessage(packet);

          String payload = String((char)PKT_TEXT) + packet;
          esp_now_send(
            broadcastAddress,
            (uint8_t*)payload.c_str(),
            payload.length()
          );
          txTextCount++;
        }
        inputLine = "";
      }
    }

    redraw();
  }
}