//***************************************************************************************
#define SAMPLES 128
uint16_t value;
byte peak[] = {0, 0, 0, 0, 0, 0, 0};
double vReal[SAMPLES];
double vImag[SAMPLES];
//***************************************************************************************
uint16_t COLORIX;
uint16_t max_bar_height = 140;
uint16_t bar_height = 196; //32
uint16_t bar_width = 24;
uint16_t bar_spacing = 10;
uint16_t bars_xoffset = 8;
uint16_t amp[7];
//***************************************************************************************
void displayBand(int band, int dsize) {
  int dmax = max_bar_height; //max bar height
  if (dsize > dmax) dsize = dmax;
  for (int s = dsize - 1; s <= dmax; s = s + 1) {
    for (uint16_t bar_tmp = 0; bar_tmp < bar_width; bar_tmp++)
      screenmemory_drawpixel((bar_width + bar_spacing)*band + bar_tmp + 1 + bars_xoffset, bar_height - s, 0x3F);
  }
  for (int s = dsize; s > 0 ; s = s - 1) {
    if (s > 120 ) COLORIX = 6; //RED
    else if (s > 100 && s <= 120) COLORIX = 24; //YELLOW
    else COLORIX = 26; //GREEN

    for (uint16_t bar_tmp = 0; bar_tmp < bar_width; bar_tmp++) {
      screenmemory_drawpixel((bar_width + bar_spacing)*band + bar_tmp + 1 + bars_xoffset, bar_height - s, COLORIX);
    }
  }
  if (dsize > peak[band]) {
    peak[band] = dsize;
  }
}
//**************************************************************************************

void visualyze() {
  for (int i = 0; i < 0+SAMPLES; i++) {

  if (audio.m_curSample<1152-SAMPLES)
    value = ((int16_t*)audio.m_outBuff)[2 * i + audio.m_curSample * 2];
  else 
    value = ((int16_t*)audio.m_outBuff)[2 * i + (1152-audio.m_curSample) *2  ];
   
    vReal[i] = value / 512 * audio.m_vol/100;
    vImag[i] = 0;
  }

  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  for (int i = 0; i < 7; i++) {
    amp[i] = 0;  //reset amp bar values
  }

  for (int i = 1; i < (SAMPLES/2); i++) { // Don't use sample 0 and only first SAMPLES/2 are usable. Each array eleement represents a frequency and its value the amplitude.
    if (i > 2 && i <= 3 & (uint16_t)vReal[i] > amp[0] )  amp[0] = (uint16_t)vReal[i]; //125Hz
    if (i > 3 && i <= 7 & (uint16_t)vReal[i] > amp[1] )  amp[1] = (uint16_t)vReal[i]; //250Hz
    if (i > 7 && i <= 12 & (uint16_t)vReal[i] > amp[2] )  amp[2] = (uint16_t)vReal[i]; //500Hz
    if (i > 12 && i <= 24 & (uint16_t)vReal[i] > amp[3] )  amp[3] = (uint16_t)vReal[i]; //1kHz
    if (i > 24 && i <= 30 & (uint16_t)vReal[i] > amp[4] )  amp[4] = (uint16_t)vReal[i]; //2kHz
    if (i > 30 && i <= 45 & (uint16_t)vReal[i] > amp[5] )  amp[5] = (uint16_t)vReal[i]; //4kHz
    if (i > 45 && i <= 64 & (uint16_t)vReal[i] > amp[6] )  amp[6] = (uint16_t)vReal[i]; //8kHz
    
  }

   //correct bar ratio
   amp[6] *= 2;
   amp[5] *= 2;
   amp[4] *= 1.5;
   amp[3]*=1.0;
   amp[2]*=0.50;
   amp[1] *= 0.25;
   amp[0] *= 0.25;
   
  ///DRAW BARS
  for (uint16_t tmp = 0; tmp < 7; tmp++) displayBand(tmp, amp[tmp]);

  //DRAW PEAKS
  for (byte band = 0; band <= 6; band++) {
    for (uint16_t bar_tmp = 0; bar_tmp < bar_width; bar_tmp++)
      screenmemory_drawpixel((bar_width + bar_spacing)*band + bar_tmp + 1 + bars_xoffset, bar_height - peak[band], 0x3d);
  }

  for (byte band = 0; band <= 6; band++) {
    if (peak[band] > 0) peak[band] -= 1; // Decay the peak
  }

  xQueueSend(vidQueue, &SCREENMEMORY, 0); //refresh LCD
}
//********************************************************************************
uint16_t PLAYINGFILE = 0;
uint16_t TOTALFILES = 0;

char TRACKNAME[64];
uint8_t VOLUME = 7;

//--------------------------------------------------------------------------------
char* EXPLORE(char* PATH) {
  uint8_t num = 0;
  uint8_t loadedFileNames = 0;

  //clear memory variables
  for (uint16_t tmp = 0; tmp < MAXFILES; tmp++) memset (filename[tmp], 0, sizeof(filename[tmp]));
  fileext[0] = 0;
  fileext[1] = 0;
  fileext[2] = 0;
  fileext[3] = 0;

  num = 0;
  loadedFileNames = 0;

  //Load List files in root directory.
  ///if (!dirFile.open("/", O_READ)) {
  if (!dirFile.open(PATH, O_READ)) {
    while (1) {};
  }
  while (num < MAXFILES && file.openNext(&dirFile, O_READ)) {

    // Skip hidden files.
    if (!file.isHidden()) {
      for (uint8_t i = sizeof(filename[num]); i > 3; i--) filename[num][i] = 0;
      file.getName(filename[num], MAXFILENAME_LENGTH);
      if (file.isSubDir()) {
        sprintf(filename[num], "%s/", filename[num]);
        num++;
      } else {
        for (uint8_t i = strlen(filename[num]); i > 3; i--) {
          if (filename[num][i] != 0) {
            fileext[3] = '\0';
            fileext[2] = filename[num][i];
            fileext[1] = filename[num][i - 1];
            fileext[0] = filename[num][i - 2];
            break;
          }
        }
      }

      if (DEBUG) {
        ///Serial.println(fileext[num]);
        ///Serial.println(strlen(filename[num]));
      }

      //check MP3 File extension, then increase index
      if ((fileext[0] == 'M' || fileext[0] == 'm')
          && (fileext[1] == 'P' || fileext[1] == 'p')
          && (fileext[2] == '3' || fileext[2] == '3')) {
        num++;
      }
      if ((fileext[0] == 'W' || fileext[0] == 'w')
          && (fileext[1] == 'A' || fileext[1] == 'a')
          && (fileext[2] == 'V' || fileext[2] == 'v')) {
        num++;
      }
      //OGG not supported
      /*if ((fileext[num][0] == 'O' || fileext[num][0] == 'o')
         && (fileext[num][1] == 'G' || fileext[num][1] == 'g')
         && (fileext[num][2] == 'G' || fileext[num][2] == 'g')) {
            num++;
        }*/
    }
    loadedFileNames = num;
    file.close();
  }

  dirFile.close();

  if (DEBUG) {
    Serial.println("--------------------------------------");
    Serial.print("Count of loaded File Names:");
    Serial.println(loadedFileNames);
  }

  sortStrings(filename, loadedFileNames);

  //DRAW FILENAMES INTO BUFFER
  uint8_t CURSOR = 0;
  uint8_t PAGE = 0;
  bool NamesDisplayed = false;

  while (1) {
#if BLUETOOTH_ENABLED
    ///      hid_update();
    ///      PS4_JOY();
#endif

    PAGE = CURSOR / FILESPERPAGE;
    if (!NamesDisplayed) {
      screenmemory_fillscreen(63); //black color
      set_font_XY(16, 24 );
      draw_string(PATH, 48);


      for (num = PAGE * FILESPERPAGE; num < ((PAGE + 1)*FILESPERPAGE) && num < loadedFileNames; num++) {
        set_font_XY(40, 48 + 20 * (num % FILESPERPAGE));
        ///draw_string(filename[num],48);

        if (filename[num][strlen(filename[num]) - 1] == '/') draw_string(filename[num], 23);
        else draw_string(filename[num], 48);

        delay(1);
      }
      NamesDisplayed = true;
    }

    //Draw Cursor
    set_font_XY(16, 48 + (20 * (CURSOR % FILESPERPAGE)));
    draw_string("->", 48);
    delay(200);

    //PROCESS CURSOR SELECTION
    while (JOY_CROSS == 0 && JOY_SQUARE == 0 && JOY_OPTIONS == 0 && JOY_SHARE == 0 && JOY_UP == 0 && JOY_DOWN == 0 && JOY_LEFT == 0 && JOY_RIGHT == 0) {
      if (digitalRead(PIN_A) == 1) {
        JOY_CROSS = 1;  //A
        delay(25);
      }
      if (digitalRead(PIN_B) == 1) {
        JOY_SQUARE = 1;   //B
        delay(25);
      }
      if (digitalRead(PIN_SELECT) == 1) {
        JOY_OPTIONS = 1;   //SELECT
        delay(25);
      }
      if (digitalRead(PIN_START) == 1) {
        JOY_SHARE = 1;   //START
        delay(25);
      }
      if (digitalRead(PIN_UP) == 1) {
        JOY_UP = 1;
        delay(25);
      }
      if (digitalRead(PIN_DOWN) == 1) {
        JOY_DOWN = 1;   //DOWN
        delay(25);
      }
      if (digitalRead(PIN_LEFT) == 1) {
        JOY_LEFT = 1;   //LEFT
        delay(25);
      }
      if (digitalRead(PIN_RIGHT) == 1) {
        JOY_RIGHT = 1;   //RIGHT
        delay(25);
      }
    }

    //Empty Cursor
    set_font_XY(16, 48 + (20 * (CURSOR % FILESPERPAGE)));
    draw_string("  ", 48);

    if (JOY_SHARE == 1 && JOY_OPTIONS == 1) {
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      EXIT = true;
      PLAYING = false;
      return MAINPATH;
    }

    if (JOY_UP == 1 ) {
      if (CURSOR % FILESPERPAGE == 0) NamesDisplayed = false; //changed page
      if (CURSOR == 0 && loadedFileNames > 0) CURSOR = loadedFileNames - 1;
      else if (CURSOR > 0 && loadedFileNames > 0) CURSOR--;
      JOY_UP = 0;
    }
    if (JOY_DOWN == 1 ) {
      if (CURSOR % FILESPERPAGE == FILESPERPAGE - 1 || CURSOR == loadedFileNames - 1) NamesDisplayed = false; //changed page
      if (CURSOR == loadedFileNames - 1 && loadedFileNames > 0) CURSOR = 0;
      else if (CURSOR < loadedFileNames - 1 && loadedFileNames > 0) CURSOR++;
      JOY_DOWN = 0;
    }
    if (JOY_LEFT == 1) {
      if (CURSOR > FILESPERPAGE - 1) CURSOR -= FILESPERPAGE;
      NamesDisplayed = false;
      JOY_LEFT = 0;
    }
    if (JOY_RIGHT == 1) {
      if (CURSOR / FILESPERPAGE < loadedFileNames / FILESPERPAGE) CURSOR += FILESPERPAGE;
      if (CURSOR > loadedFileNames - 1) CURSOR = loadedFileNames - 1;
      NamesDisplayed = false;
      JOY_RIGHT = 0;
    }
    if (JOY_OPTIONS == 1) {
      //do nothing  = unused
      JOY_OPTIONS = 0;
    }
    if ((JOY_CROSS == 1 || JOY_SHARE == 1) && JOY_OPTIONS == 0) {
      dirFile.close();
      JOY_CROSS = 0;
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      delay(25);

      PLAYINGFILE = CURSOR;
      TOTALFILES = loadedFileNames;

      sprintf(MAINPATH, "%s%s", PATH, filename[CURSOR]);
      if (DEBUG) Serial.println(MAINPATH);

      sprintf(TRACKNAME, "%s", filename[CURSOR]);

      return MAINPATH ; //START //A
    }
    if ((JOY_SQUARE == 1 ) && JOY_OPTIONS == 0) {

      dirFile.close();
      JOY_SQUARE = 0;
      JOY_SHARE = 0;
      JOY_OPTIONS = 0;
      delay(25);

      if (DEBUG) Serial.println(PATH);
      if (DEBUG) Serial.println(strlen(PATH));

      sprintf(MAINPATH, "%s", PATH);

      if (strlen(MAINPATH) > 1) {
        MAINPATH[strlen(MAINPATH) - 1] = '\0';
        for (uint8_t strpos = strlen(MAINPATH) - 1; strpos > 0; strpos--) {
          if (MAINPATH[strpos] == '/') break;
          MAINPATH[strpos] = '\0';
        }
      }

      if (DEBUG) Serial.println(MAINPATH);
      if (DEBUG) Serial.println(strlen(MAINPATH));
      return MAINPATH ;
    }
  };
}
//################################################################################
//********************************************************************************
char* Browse(char* PATH) {
  if (PATH[strlen(PATH) - 1] != '/')
    if (strlen(PATH) > 1) {
      PATH[strlen(PATH) - 1] = '\0';
      for (uint8_t strpos = strlen(PATH) - 1; strpos > 0; strpos--) {
        if (PATH[strpos] == '/') break;
        PATH[strpos] = '\0';
      }
    }

  Serial.print("EXPLORE: ");
  Serial.println(PATH);
  //................................................................................
  while (PATH[strlen(PATH) - 1] == '/')  {
    PATH =  EXPLORE(PATH);
    if (EXIT) break;
  }
  //................................................................................

  return PATH;
}
//________________________________________________________________________________

char* PREVNEXT(char* PATH, uint8_t POSITION) {

  if (PATH[strlen(PATH) - 1] != '/')
    if (strlen(PATH) > 1) {
      PATH[strlen(PATH) - 1] = '\0';
      for (uint8_t strpos = strlen(PATH) - 1; strpos > 0; strpos--) {
        if (PATH[strpos] == '/') break;
        PATH[strpos] = '\0';
      }
    }

  uint8_t num = 0;
  uint8_t loadedFileNames = 0;

  //LOAD FILENAMES INTO MEMORY...

  num = 0;
  if (!dirFile.open(PATH, O_READ)) {
    while (1) {};
  }
  while (num < MAXFILES && file.openNext(&dirFile, O_READ)) {

    // Skip directories and hidden files.
    if (!file.isSubDir() && !file.isHidden()) {

      for (uint8_t i = sizeof(filename[num]); i > 3; i--) filename[num][i] = 0;

      file.getName(filename[num], MAXFILENAME_LENGTH);

      if (file.isSubDir()) {
        ///sprintf(filename[num], "%s/", filename[num]);
        ///num++;
      } else {
        for (uint8_t i = strlen(filename[num]); i > 3; i--) {
          if (filename[num][i] != 0) {
            fileext[3] = '\0';
            fileext[2] = filename[num][i];
            fileext[1] = filename[num][i - 1];
            fileext[0] = filename[num][i - 2];
            break;
          }
        }
      }

      //check MP3 File extension, then increase index
      if ((fileext[0] == 'M' || fileext[0] == 'm')
          && (fileext[1] == 'P' || fileext[1] == 'p')
          && (fileext[2] == '3' || fileext[2] == '3')) {
        num++;
      }
      if ((fileext[0] == 'W' || fileext[0] == 'w')
          && (fileext[1] == 'A' || fileext[1] == 'a')
          && (fileext[2] == 'V' || fileext[2] == 'v')) {
        num++;
      }
    }
    loadedFileNames = num;
    file.close();
  }
  dirFile.close();
  if (DEBUG) {
    Serial.println("--------------------------------------");
    Serial.print("Count of loaded File Names:");
    Serial.println(loadedFileNames);
  }

  sortStrings(filename, loadedFileNames);

  sprintf(MAINPATH, "%s%s", PATH, filename[POSITION]);
  if (DEBUG) Serial.println(MAINPATH);

  sprintf(TRACKNAME, "%s", filename[POSITION]);

  return MAINPATH ; //START //A
}
//________________________________________________________________________________
