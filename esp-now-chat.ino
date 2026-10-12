#include <M5Cardputer.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

String inputLine = "";
String history[20];
int historyCount = 0;

uint8_t broadcastAddress[] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

String username = "Nobody";
int currentChannel = 1;

volatile bool beepPending = false;
volatile bool redrawPending = false;

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

  if (cmd.startsWith("/user "))
  {
    String newName = cmd.substring(6);
    newName.trim();

    if (newName.length() > 0)
    {
      username = newName;
      addMessage("Username -> " + username);
    }
    else
    {
      addMessage("Usage: /user xxx");
    }

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
      esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);
      addMessage("Channel -> " + String(currentChannel));
    }
    else
    {
      addMessage("Channel must be 1-13");
    }

    return true;
  }

  addMessage("Unknown cmd: " + cmd);
  return true;
}

void redraw()
{
  M5Cardputer.Display.fillScreen(BLACK);

  const int lineHeight = 10;

  int maxVisible =
    (M5Cardputer.Display.height() - 16) /
    lineHeight;

  int startIndex = 0;

  if (historyCount > maxVisible)
  {
    startIndex =
      historyCount - maxVisible;
  }

  int y = 2;

  for (int i = startIndex;
       i < historyCount;
       i++)
  {
    M5Cardputer.Display.drawString(
      history[i],
      2,
      y
    );

    y += lineHeight;
  }

  M5Cardputer.Display.fillRect(
    0,
    M5Cardputer.Display.height() - 14,
    M5Cardputer.Display.width(),
    14,
    DARKGREY
  );

  M5Cardputer.Display.drawString(
    ">" + inputLine,
    2,
    M5Cardputer.Display.height() - 12
  );
}

void onDataRecv(
  const esp_now_recv_info_t *info,
  const uint8_t *incomingData,
  int len)
{
  String msg = "";

  for (int i = 0; i < len; i++)
  {
    msg += (char)incomingData[i];
  }

  addMessage(msg);
  beepPending = true;
  redrawPending = true;
}

void onDataSent(
  const wifi_tx_info_t *info,
  esp_now_send_status_t status)
{
  // Optional send status handling
}

void setup()
{
  auto cfg = M5.config();

  M5Cardputer.begin(cfg, true);

  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.setTextSize(1);

  M5Cardputer.Speaker.begin();

  WiFi.mode(WIFI_STA);

  esp_wifi_set_channel(currentChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    M5Cardputer.Display.println("ESP-NOW FAIL");

    while (true)
    {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;      // 0 = 跟随当前 Wi-Fi 信道，切信道自动生效
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);

  addMessage("ESP-NOW CHAT READY");
  addMessage("CH:" + String(currentChannel) + "  MAC:");
  addMessage(WiFi.macAddress());

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

  if (M5Cardputer.Keyboard.isChange())
  {
    Keyboard_Class::KeysState status =
      M5Cardputer.Keyboard.keysState();

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
          String packet =
            username +
            ": " +
            inputLine;

          addMessage(packet);

          esp_now_send(
            broadcastAddress,
            (uint8_t*)packet.c_str(),
            packet.length()
          );
        }

        inputLine = "";
      }
    }

    redraw();
  }
}