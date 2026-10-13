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

// 编码帧队列：回调只入队原始 G722 帧，解码挪到主循环
#define ENC_QUEUE_SIZE 16
struct EncFrame
{
  uint8_t len;
  uint8_t seq;
  uint8_t data[FRAME_BYTES];
};
EncFrame encQueue[ENC_QUEUE_SIZE];
volatile int encQueueHead = 0;
volatile int encQueueTail = 0;

// 文本队列：回调只做定长拷贝，避免在回调中堆分配 String
#define TEXT_QUEUE_SIZE 4
#define TEXT_MAX_LEN 250
struct TextFrame
{
  char data[TEXT_MAX_LEN + 1];
};
TextFrame textQueue[TEXT_QUEUE_SIZE];
volatile int textQueueHead = 0;
volatile int textQueueTail = 0;

constexpr uint8_t PKT_TEXT = 0x01;
constexpr uint8_t PKT_AUDIO = 0x02;

volatile int txAudioCount = 0;
volatile int rxAudioCount = 0;
volatile int txFailCount = 0;
volatile int rxDropCount = 0;
volatile int txTextCount = 0;
volatile int rxTextCount = 0;

volatile int decodePeak = 0;
volatile int playPeak = 0;
volatile int lastEncodedLen = 0;

// --- 播放节奏控制（累积节拍：loop 卡顿后自动追赶，不累积永久延迟） ---
unsigned long nextPlayTime = 0;
constexpr int PLAY_INTERVAL_MS = 20;

// --- VAD 挂起：语音结束后继续发 N 帧（20ms/帧），避免句尾被切 ---
constexpr int VAD_HANGOVER_FRAMES = 15; // 300ms
int hangoverFrames = 0;
uint8_t txSeq = 0;

// --- 接收侧丢帧补偿（PLC）：丢包时重复上一帧 ---
constexpr int MAX_PLC_FRAMES = 2;
uint8_t lastRxSeq = 0;
bool haveLastRxSeq = false;
int16_t lastDecoded[FRAME_SAMPLES];

// --- 显示双缓冲画布，消除整屏重绘闪烁 ---
M5Canvas canvas(&M5Cardputer.Display);

// 播放队列入队（满则丢弃并计数）
bool pushPcmFrame(const int16_t *src)
{
  int nextHead = (audioQueueHead + 1) % AUDIO_QUEUE_SIZE;
  if (nextHead == audioQueueTail)
  {
    rxDropCount++;
    return false;
  }
  memcpy(audioQueue[audioQueueHead], src, FRAME_SAMPLES * sizeof(int16_t));
  audioQueueHead = nextHead;
  return true;
}

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

void redraw()
{
  // 所有绘制先在 canvas 完成，最后一次 pushSprite 上屏，消除闪烁
  canvas.fillScreen(BLACK);

  const int lineHeight = 10;
  int maxVisible = (canvas.height() - 16) / lineHeight;
  int startIndex = 0;

  if (historyCount > maxVisible)
  {
    startIndex = historyCount - maxVisible;
  }

  int y = 2;
  for (int i = startIndex; i < historyCount; i++)
  {
    canvas.drawString(history[i], 2, y);
    y += lineHeight;
  }

  canvas.fillRect(
    0, canvas.height() - 14,
    canvas.width(), 14, DARKGREY);

  canvas.drawString(
    ">" + inputLine, 2, canvas.height() - 12);

  if (pttActive)
  {
    canvas.fillRect(
      canvas.width() - 40, 2, 38, 12, RED);
    canvas.setTextColor(WHITE);
    canvas.drawString("TX", canvas.width() - 34, 3);
  }
  else
  {
    canvas.fillRect(
      canvas.width() - 40, 2, 38, 12, DARKGREEN);
    canvas.setTextColor(WHITE);
    canvas.drawString("RX", canvas.width() - 34, 3);
  }
  canvas.setTextColor(GREEN);

  canvas.pushSprite(0, 0);
}

// 麦克风双缓冲：一块填满后在回调里立即提交另一块，采集无间隙
// （record() 只是向持续运行的采集任务提交缓冲，不排队就会丢采样）
int16_t micBuf[2][FRAME_SAMPLES];
volatile int micReady = -1; // -1=无就绪帧；0/1=micBuf 下标待处理

void micBufferRelease(void *args, void *data, size_t length)
{
  int idx = (data == (void *)micBuf[0]) ? 0 : 1;
  // 回调内提交另一块缓冲（官方支持的用法），消除采集间隙
  M5Cardputer.Mic.record(micBuf[idx ^ 1], FRAME_SAMPLES, 16000);
  micReady = idx;
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

    // 只做定长拷贝入队，String/显示挪到主循环
    int msgLen = len - 1;
    if (msgLen > 0)
    {
      if (msgLen > TEXT_MAX_LEN) msgLen = TEXT_MAX_LEN;

      int nextHead = (textQueueHead + 1) % TEXT_QUEUE_SIZE;
      if (nextHead != textQueueTail)
      {
        memcpy(textQueue[textQueueHead].data, &incomingData[1], msgLen);
        textQueue[textQueueHead].data[msgLen] = '\0';
        textQueueHead = nextHead;
      }
    }
  }
  else if (pktType == PKT_AUDIO)
  {
    rxAudioCount++;

    // 包格式：[类型][序号][G722帧]
    if (!pttActive && len > 2)
    {
      int audioLen = len - 2;
      if (audioLen <= FRAME_BYTES)
      {
        // 只入队原始编码帧，解码挪到主循环
        int nextHead = (encQueueHead + 1) % ENC_QUEUE_SIZE;
        if (nextHead != encQueueTail)
        {
          memcpy(encQueue[encQueueHead].data, &incomingData[2], audioLen);
          encQueue[encQueueHead].len = audioLen;
          encQueue[encQueueHead].seq = incomingData[1];
          encQueueHead = nextHead;
        }
        else
        {
          rxDropCount++;
        }
      }
    }
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  // 以发送回调的最终状态为准（esp_now_send 返回 OK 只代表入队成功）
  if (status != ESP_NOW_SEND_SUCCESS)
  {
    txFailCount++;
  }
}

void setup()
{
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);

  // 显示双缓冲画布（全屏 16bit），默认绿色字体
  canvas.setColorDepth(16);
  canvas.createSprite(
    M5Cardputer.Display.width(), M5Cardputer.Display.height());
  canvas.setTextColor(GREEN);

  M5Cardputer.Speaker.begin();
  // 默认音量仅 64/255，拉满改善声音小
  M5Cardputer.Speaker.setVolume(255);

  // 注册麦克风缓冲释放回调（须在首次 record 之前设置）
  M5Cardputer.Mic.setBufferReleaseCallback(nullptr, micBufferRelease);

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

  // 处理收到的文本（主循环中入历史，避免在回调里堆分配）
  if (textQueueTail != textQueueHead)
  {
    addMessage(String(textQueue[textQueueTail].data));
    textQueueTail = (textQueueTail + 1) % TEXT_QUEUE_SIZE;
    beepPending = true;
    redrawPending = true;
  }

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

  // --- 主循环解码收到的编码帧，含丢帧补偿（PLC） ---
  if (!pttActive && encQueueTail != encQueueHead)
  {
    EncFrame &f = encQueue[encQueueTail];

    // 序号出现间隙说明中间丢包，用上一帧重复补偿（限制帧数防机械感）
    if (haveLastRxSeq)
    {
      uint8_t gap = (uint8_t)(f.seq - lastRxSeq - 1);
      if (gap > 0 && gap <= 100)
      {
        int plc = (gap < MAX_PLC_FRAMES) ? gap : MAX_PLC_FRAMES;
        for (int k = 0; k < plc; k++)
        {
          pushPcmFrame(lastDecoded);
        }
      }
    }

    dec.decode(f.data, f.len, lastDecoded, FRAME_SAMPLES);

    int16_t peak = 0;
    for (int i = 0; i < FRAME_SAMPLES; i++)
    {
      int16_t v = lastDecoded[i];
      if (v < 0) v = -v;
      if (v > peak) peak = v;
    }
    decodePeak = peak;

    // 轻量向下扩张：峰值低于阈值时按比例衰减，压低句间底噪（语音帧不受影响）
    constexpr int EXPAND_THRESH = 1000;
    if (peak > 0 && peak < EXPAND_THRESH)
    {
      int32_t factor = ((int32_t)peak << 8) / EXPAND_THRESH;
      for (int i = 0; i < FRAME_SAMPLES; i++)
      {
        lastDecoded[i] = (int16_t)(((int32_t)lastDecoded[i] * factor) >> 8);
      }
    }

    pushPcmFrame(lastDecoded);

    haveLastRxSeq = true;
    lastRxSeq = f.seq;
    encQueueTail = (encQueueTail + 1) % ENC_QUEUE_SIZE;
  }

  // --- 播放节拍：playRaw 为非阻塞入队，用累积计时控制节奏 ---
  // loop 卡顿落后时自动加速追赶，不累积永久延迟；落后过多则对齐防突发
  if (!pttActive && audioQueueTail != audioQueueHead)
  {
    unsigned long now = millis();
    if ((long)(now - nextPlayTime) >= 0)
    {
      if (now - nextPlayTime > 200) nextPlayTime = now;
      nextPlayTime += PLAY_INTERVAL_MS;

      int16_t pp = 0;
      const int16_t *p = audioQueue[audioQueueTail];
      for (int i = 0; i < FRAME_SAMPLES; i++)
      {
        int16_t v = p[i];
        if (v < 0) v = -v;
        if (v > pp) pp = v;
      }
      playPeak = pp;

      // playRaw 返回 false 表示内部缓冲满，保留该帧下轮重试
      if (M5Cardputer.Speaker.playRaw(
            audioQueue[audioQueueTail], FRAME_SAMPLES, 16000, false, 1, 0))
      {
        audioQueueTail = (audioQueueTail + 1) % AUDIO_QUEUE_SIZE;
      }
    }
  }

  Keyboard_Class::KeysState status =
    M5Cardputer.Keyboard.keysState();

  bool tabNow = status.tab;

  if (tabNow && !pttActive)
  {
    pttActive = true;
    // 清空接收队列，避免松开 PTT 后播放过期的旧音频
    encQueueTail = encQueueHead;
    audioQueueTail = audioQueueHead;
    haveLastRxSeq = false;
    hangoverFrames = 0;
    nextPlayTime = millis();
    M5Cardputer.Speaker.end();
    delay(10);
    M5Cardputer.Mic.begin();
    // 提交首块缓冲，此后由释放回调自动交替提交，采集无缝
    micReady = -1;
    M5Cardputer.Mic.record(micBuf[0], FRAME_SAMPLES, 16000);
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

  // PTT 发送中：采集由回调双缓冲驱动，这里只处理就绪帧
  // 流程：取帧 → 放大 → VAD 挂起 → 编码 → 发送
  if (pttActive)
  {
    int ready = micReady;
    if (ready >= 0)
    {
      micReady = -1;
      // 拷出处理，采集任务同时填充另一块
      memcpy(pcmBuf, micBuf[ready], sizeof(pcmBuf));

      // 增益 ×10 + 软限幅：超过限幅点后压缩增益，避免硬削波发劈
      constexpr int GAIN_MULTIPLIER = 10;
      constexpr int SOFT_LIMIT = 24000;
      for (int i = 0; i < FRAME_SAMPLES; i++)
      {
        int32_t v = (int32_t)pcmBuf[i] * GAIN_MULTIPLIER;
        if (v > SOFT_LIMIT)
        {
          v = SOFT_LIMIT + (v - SOFT_LIMIT) / 8;
          if (v > 32767) v = 32767;
        }
        else if (v < -SOFT_LIMIT)
        {
          v = -SOFT_LIMIT + (v + SOFT_LIMIT) / 8;
          if (v < -32768) v = -32768;
        }
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

      // VAD 挂起：检测到语音则重置挂起计数，静音期继续发送挂起帧
      if (peak >= VAD_THRESHOLD)
      {
        hangoverFrames = VAD_HANGOVER_FRAMES;
      }
      else if (hangoverFrames > 0)
      {
        hangoverFrames--;
      }

      if (peak >= VAD_THRESHOLD || hangoverFrames > 0)
      {
        int encodedLen = enc.encode(pcmBuf, FRAME_SAMPLES, encodedBuf, FRAME_BYTES);

        if (encodedLen > 0)
        {
          lastEncodedLen = encodedLen;

          // 包格式：[类型][序号][G722帧]，序号供接收端丢帧补偿
          uint8_t packet[FRAME_BYTES + 2];
          packet[0] = PKT_AUDIO;
          packet[1] = txSeq++;
          memcpy(&packet[2], encodedBuf, encodedLen);

          if (esp_now_send(broadcastAddress, packet, encodedLen + 2) == ESP_OK)
          {
            txAudioCount++;
          }
          else
          {
            txFailCount++;
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

          // ESP-NOW 单包上限 250 字节（含 1 字节类型前缀）
          if (packet.length() + 1 > 250)
          {
            addMessage("Too long: " + String(packet.length()) + "B (max 249)");
          }
          else
          {
            addMessage(packet);

            String payload = String((char)PKT_TEXT) + packet;
            if (esp_now_send(
                  broadcastAddress,
                  (uint8_t *)payload.c_str(),
                  payload.length()) == ESP_OK)
            {
              txTextCount++;
            }
            else
            {
              txFailCount++;
            }
          }
        }
        inputLine = "";
      }
    }

    redraw();
  }
}