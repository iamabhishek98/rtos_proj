#ifndef SONGS_H_
#define SONGS_H_

/*
	Header file that stores all the song related constants 
	such as notes and duration between notes for the song 
	requirments of the 3 different tunes in CG2271 project.
*/

// Remap existing notes to higher frequencies that can be played by our buzzer
#define del_song 5 

float songspeed = 2; // the bigger the number the slower the song


/*
Credit for the song info: 
https://github.com/robsoncouto/arduino-songs/tree/master/supermariobros
*/

//*****************************************
#define NOTE_B0  31 * del_song
#define NOTE_C1  33 * del_song
#define NOTE_CS1 35 * del_song
#define NOTE_D1  37 * del_song
#define NOTE_DS1 39 * del_song
#define NOTE_E1  41 * del_song
#define NOTE_F1  44 * del_song
#define NOTE_FS1 46 * del_song
#define NOTE_G1  49 * del_song
#define NOTE_GS1 52 * del_song
#define NOTE_A1  55 * del_song
#define NOTE_AS1 58 * del_song
#define NOTE_B1  62 * del_song
#define NOTE_C2  65 * del_song
#define NOTE_CS2 69 * del_song
#define NOTE_D2  73 * del_song
#define NOTE_DS2 78 * del_song
#define NOTE_E2  82 * del_song
#define NOTE_F2  87 * del_song
#define NOTE_FS2 93 * del_song
#define NOTE_G2  98 * del_song
#define NOTE_GS2 104 * del_song
#define NOTE_A2  110 * del_song
#define NOTE_AS2 117 * del_song
#define NOTE_B2  123 * del_song
#define NOTE_C3  131 * del_song
#define NOTE_CS3 139 * del_song
#define NOTE_D3  147 * del_song
#define NOTE_DS3 156 * del_song
#define NOTE_E3  165 * del_song
#define NOTE_F3  175 * del_song
#define NOTE_FS3 185 * del_song
#define NOTE_G3  196 * del_song
#define NOTE_GS3 208 * del_song
#define NOTE_A3  220 * del_song
#define NOTE_AS3 233 * del_song
#define NOTE_B3  247 * del_song
#define NOTE_C4  262 * del_song
#define NOTE_CS4 277 * del_song
#define NOTE_D4  294 * del_song
#define NOTE_DS4 311 * del_song
#define NOTE_E4  330 * del_song
#define NOTE_F4  349 * del_song
#define NOTE_FS4 370 * del_song
#define NOTE_G4  392 * del_song
#define NOTE_GS4 415 * del_song
#define NOTE_A4  440 * del_song
#define NOTE_AS4 466 * del_song
#define NOTE_B4  494 * del_song
#define NOTE_C5  523 * del_song
#define NOTE_CS5 554 * del_song
#define NOTE_D5  587 * del_song
#define NOTE_DS5 622 * del_song
#define NOTE_E5  659 * del_song
#define NOTE_F5  698 * del_song
#define NOTE_FS5 740 * del_song
#define NOTE_G5  784 * del_song
#define NOTE_GS5 831 * del_song
#define NOTE_A5  880 * del_song
#define NOTE_AS5 932 * del_song
#define NOTE_B5  988 * del_song
#define NOTE_C6  1047 * del_song
#define NOTE_CS6 1109 * del_song
#define NOTE_D6  1175 * del_song
#define NOTE_DS6 1245 * del_song
#define NOTE_E6  1319 * del_song
#define NOTE_F6  1397 * del_song
#define NOTE_FS6 1480 * del_song
#define NOTE_G6  1568 * del_song
#define NOTE_GS6 1661 * del_song
#define NOTE_A6  1760 * del_song
#define NOTE_AS6 1865 * del_song
#define NOTE_B6  1976 * del_song
#define NOTE_C7  2093 * del_song
#define NOTE_CS7 2217 * del_song
#define NOTE_D7  2349 * del_song
#define NOTE_DS7 2489 * del_song
#define NOTE_E7  2637 * del_song
#define NOTE_F7  2794 * del_song
#define NOTE_FS7 2960 * del_song
#define NOTE_G7  3136 * del_song
#define NOTE_GS7 3322 * del_song
#define NOTE_A7  3520 * del_song
#define NOTE_AS7 3729 * del_song
#define NOTE_B7  3951 * del_song
#define NOTE_C8  4186 * del_song
#define NOTE_CS8 4435 * del_song
#define NOTE_D8  4699 * del_song
#define NOTE_DS8 4978 * del_song

// Song played while vehicle is moving. Mario Underworld Tune.
int running_melody[] = {
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_C4, NOTE_C5, NOTE_A3, NOTE_A4,
  NOTE_AS3, NOTE_AS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0,
  NOTE_F3, NOTE_F4, NOTE_D3, NOTE_D4,
  NOTE_DS3, NOTE_DS4, 0,
  0, NOTE_DS4, NOTE_CS4, NOTE_D4,
  NOTE_CS4, NOTE_DS4,
  NOTE_DS4, NOTE_GS3,
  NOTE_G3, NOTE_CS4,
  NOTE_C4, NOTE_FS4, NOTE_F4, NOTE_E3, NOTE_AS4, NOTE_A4,
  NOTE_GS4, NOTE_DS4, NOTE_B3,
  NOTE_AS3, NOTE_A3, NOTE_GS3,
  0, 0, 0
};


// Song played at end of run. Pirates of the carribean theme song.
int end_melody[] = {       //Note of the song, 0 is a rest/pulse
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,

   NOTE_A4, NOTE_A4, 
   //Repeat of first part
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,

   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_C5, NOTE_D5, NOTE_B4, NOTE_B4, 0,
   NOTE_A4, NOTE_G4, NOTE_A4, 0,
   
   NOTE_E4, NOTE_G4, NOTE_A4, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 0, 
   NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 0,
   NOTE_E5, NOTE_D5, NOTE_E5, NOTE_A4, 0,
   
   NOTE_A4, NOTE_B4, NOTE_C5, NOTE_C5, 0, 
   NOTE_D5, NOTE_E5, NOTE_A4, 0, 
   NOTE_A4, NOTE_C5, NOTE_B4, NOTE_B4, 0,
   NOTE_C5, NOTE_A4, NOTE_B4, 0,
   //End of Repeat

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4,

   NOTE_E5, 0, 0, NOTE_F5, 0, 0,
   NOTE_E5, NOTE_E5, 0, NOTE_G5, 0, NOTE_E5, NOTE_D5, 0, 0,
   NOTE_D5, 0, 0, NOTE_C5, 0, 0,
   NOTE_B4, NOTE_C5, 0, NOTE_B4, 0, NOTE_A4
};

int running_duration[] = {
	83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  333,
  83, 83, 83, 83,
  83, 83, 166,
  166, 55, 55, 55,
  166, 166,
  166, 166,
  166, 166,
  55, 55, 55, 55, 55, 55,
  100, 100, 100,
  100, 100, 100,
  333, 333, 333
};

int end_duration[] = {         //duration of each note (in ms) Quarter Note is set to 250 ms
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,

  250, 125,
  //Rpeat of First Part
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 375, 125, 
  
  125, 125, 250, 125, 125, 
  125, 125, 250, 125, 125,
  125, 125, 250, 125, 125,
  125, 125, 125, 250, 125,

  125, 125, 250, 125, 125, 
  250, 125, 250, 125, 
  125, 125, 250, 125, 125,
  125, 125, 375, 375,
  //End of Repeat
  
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500,

  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 125, 125, 125, 375,
  250, 125, 375, 250, 125, 375,
  125, 125, 125, 125, 125, 500
};

#endif /* SONGS_H_ */
