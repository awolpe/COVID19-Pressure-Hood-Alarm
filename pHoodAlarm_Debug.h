// Enables serial output of data needed for debugging only
#define DEBUG 0

// Define debugging functions
#if DEBUG
  Serial.begin(57600);
  // If debugging is on, these functions print the passed data to serial output
  #define PRINTS(s)   { Serial.print(F(s)); }                  // Prints static text only. The 'F()' functions stores the string in non-volitile memory to save RAM
  #define PRINT(s,v)  { Serial.print(F(s)); Serial.print(v); } // Prints static text, followed by any dyncamic text passed in runtime. The 'F()' function for static text saves RAM 
#else
  // If debugging is off, these functions will do nothing
  #define PRINTS(s)
  #define PRINT(s,v)
#endif
