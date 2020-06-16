#include <AltSoftSerial.h>

AltSoftSerial  OpenLCD; // Using the Alt Software Serial library to comm. with LCD. Most efficient alternative to hardware serial.

volatile byte UI_Index = 0; // Index that indicates what view to display on LCD











  //Start communication with LCD
  OpenLCD.begin(9600); // Initialize LCD
  OpenLCD.write('|'); // Put LCD into setting mode
  OpenLCD.write(0x0F); // Set Baud to 19200( 0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,0x0B=2400,
  OpenLCD.flush();
  
  OpenLCD.begin(19200); // Initialize LCD
  OpenLCD.write('|'); // Put LCD into setting mode
  OpenLCD.write(24); // Send contrast command
  OpenLCD.write(2); // Send contrast command








/*================================================================================
  Function: printLCD
  Description: This function will print out characters to the display. Row 1 and
  Row 2 strings are set in the "rows" array, respectively. If any row is more than
  16 characters, the string will be trimmed. If a row is less than 16 char, the row
  will be padded with spaces. The rows are then concat. into one string, and printed
  out to the display, resulting in a perfect fit.
  //  ================================================================================*/
bool printLCD (String rows[]) {

  // flag to return, for each row, to indicate if the string
  // length is too long to be printed in one row.
  // If so, it will be trimmed to 16 characters
  bool rows_overflow[2] = { false, false };

  // Store the passed string character length of both rows
  int rows_length[2] = { rows[0].length(), rows[1].length() } ;


  // Set the overflow flags for each row
  // trim the string if greater than 16 characters
  // pad the string with spaces if less than 16 characters
  for (int i = 0; i < 2; i++) {
    if (rows_length[i] > 16) {
      rows_overflow[i] = true;
      rows[i] = rows[i].substring(0, 16);
    }
    else {
      rows_overflow[i] = false;
      for (int j = rows_length[i]; j < 16; j++) {
        rows[i] += " ";
      } // end nested for loop
    } //end else
  } //end top level for loop


  OpenLCD.write('|'); //Setting character
  OpenLCD.write('-'); //Clear display

  OpenLCD.print(rows[0] + rows[1]); //Print first row characters to LCD

  // Logical OR the overflow flags to indicate if a character overflow was detected and corrected
  return (rows_overflow[0] || rows_overflow[1]);
}
