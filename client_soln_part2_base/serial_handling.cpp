#include "serial_handling.h"

#include <Arduino.h>
#include <errno.h>
#include <assert13.h>
#include "dprintf.h"

/* YOUR TASK:
replace the srv_get_pathlen and srv_get_waypoints functions
with the proper implementations.
*/

/*
Gets the length of the path between start and end.

The function sends a request to the server listening on the serial
port and waits for the server responding with the number of
waypoints along the path from start to end.

Inputs:

start, end - the start and end point of the requested path

Returns:

the length of the path, -1 if an error occurred

*/
int16_t srv_get_pathlen(LonLat32 start, LonLat32 end) {
  int16_t path_len;
  int16_t buf_size = 100;
  char path_len_from_server[buf_size];
  uint16_t buf_len;
  bool msgflag;
  bool timeflag;
  // start the server communication with a path request
  dprintf("Requesting lat %ld lon %ld to lat %ld lon %ld",
  start.lat, start.lon, end.lat, end.lon);

  //Send request to the server
  Serial.print("R ");
  Serial.print(start.lat); Serial.print(" ");
  Serial.print(start.lon); Serial.print(" ");
  Serial.print(end.lat); Serial.print(" ");
  Serial.print(end.lon), Serial.println("");

  //Check for timeout on recieving path from the server.(ERROR)
  uint32_t start_time = millis();
  while (Serial.available() == 0){
      if (millis() - start_time > 10000){
        dprintf("WAITING FOR PATH TIMEOUT.");
        return -1;
      }
  }


  // Read into buffer path_len_from_server
  buf_len = serial_readline(path_len_from_server, 100);
  //buffer for holding the number string.
  char number[buf_len-2];
  // If path_len_from_server is the correct format, read into number
  // buffer, convert to int and return the path_len
  if (path_len_from_server[0] == 'N' && path_len_from_server[1] == ' '){
    for (int i; i < buf_len-2; ++i){
      number[i] = path_len_from_server[i+2];
    }
    path_len = string_get_int(number);
  }
  // If path_len_from_server is not the correct format, return -1 (ERROR)
  else{
    dprintf("Server Response Format Incorrect: Start a new request.");
    return -1;
  }

  // now you will expect to get a N dddddd message back from server
  return path_len;
}

/*
Fetch the waypoints from the server corresponding to a previously
requested path. This function must be called once after a call to
srv_get_pathlen returns a path of length > 0.

The server wants to send path_len points, but we can only accept
max_path_len. So we will continue reading the waypoints past
max_path_len, but not store them.

waypoints - an array of [max_path_len] waypoints, each one being
a LonLat32 coordinate pair.

path_len - the number of waypoints expected from the server, which
could be more than the available storage in waypoints.  Excess
waypoints are discarded by the client, even though the protocol
continues.

max_path_len - the maximum mumber of waypoints that can fit int the
waypoints array

Returns
-1 if an error occurred
>= 0 if ok.
*/

int16_t srv_get_waypoints(LonLat32* waypoints,
  int16_t path_len, int16_t max_path_len) {

    uint16_t index;
    int32_t lat;
    int32_t lon;
    char str_lat[12];
    char str_lon[12];
    uint16_t bytes;
    uint16_t buf_len = 100;
    char waypoint_from_server [buf_len];
    char* sep = " ";

    dprintf("Fetching %d way points, keeping at most %d",
    path_len, max_path_len);

    if ( path_len <= 0 || max_path_len < 0 ) {
      dprintf("Bad length %d or max %d", path_len, max_path_len);
      return -1;
    }

    uint32_t start_time;

    for(int16_t i = 0; i <= path_len; ++i){
      // Request waypoint transmission with "A"
      Serial.print("A"); Serial.println("");

      //Check for timeout when requesting waypoint from the server. (ERROR)
      start_time = millis();
      while (Serial.available() == 0){
          if (millis() - start_time > 1000){
            dprintf("%d", millis()- start_time);
            dprintf("WAITING FOR WAYPOINT TIMEOUT.");
            Serial.print("TIMEOUT: Server Reset");
            return -1;
          }
      }
      // dprintf("time: %d", millis() - start_time);


      // Fill waypoint buffer with the waypoint line.
      bytes = serial_readline(waypoint_from_server, buf_len);
      // if the waypoint buffer has the correct format, set index to 2,
      // read the lat into str_lat, and the lon into str_long
      // as strings and then convert them to ints and store them in the
      // waypoints LonLat32 structure.
      if (waypoint_from_server[0] == 'W' && waypoint_from_server[1] == ' '){

        index = 2;
        index = string_read_field(waypoint_from_server, index, str_lat, bytes, sep);
        index = string_read_field(waypoint_from_server, index, str_lon, bytes, sep);
        waypoints[i] = LonLat32(string_get_int(str_lon), string_get_int(str_lat));
      }
      // If waypoint_from_server is the character 'E', know that it is the end
      // of the waypoints to be sent
      else if (waypoint_from_server[0] == 'E'){
        dprintf("End of waypoints");
      }
      // Else, incorrect waypoint format, return -1 (ERROR)
      else {
        dprintf("Incorrect Waypoint or Endpoint Format from Server: Start a new request.");
        return -1;
      }
    }
    return 0;
  }


  /*
  Function to read a single line from the serial buffer up to a
  specified length (length includes the null termination character
  that must be appended onto the string). This function is blocking.
  The newline character sequence is given by CRLF, or "\r\n".

  Arguments:

  buffer - Pointer to a buffer of characters where the string will
  be stored.

  length - The maximum length of the string to be read.

  Preconditions:  None.

  Postconditions: Function will block until a full newline has been
  read, or the maximum length has been reached. Afterwards the new
  string will be stored in the buffer passed to the function.

  Returns: the number of bytes read

  */
  int16_t serial_readline(char *line, uint16_t line_size) {
    int bytes_read = 0;    // Number of bytes read from the serial port.

    // Read until we hit the maximum length, or a newline.
    // One less than the maximum length because we want to add a null terminator.
    while (bytes_read < line_size - 1) {
      while (Serial.available() == 0) {
        // There is no data to be read from the serial port.
        // Wait until data is available.
      }

      line[bytes_read] = (char) Serial.read();

      // A newline is given by \r or \n, or some combination of both
      // or the read may have failed and returned 0
      if ( line[bytes_read] == '\r' || line[bytes_read] == '\n' ||
      line[bytes_read] == 0 ) {
        // We ran into a newline character!  Overwrite it with \0
        break;    // Break out of this - we are done reading a line.
      } else {
        bytes_read++;
      }
    }

    // Add null termination to the end of our string.
    line[bytes_read] = '\0';
    return bytes_read;
  }

  /*
  Function to read a portion of a string into a buffer, up to any
  given separation characters. This will read up to the specified
  length of the character buffer if a separation character is not
  encountered, or until the end of the string which is being copied. A
  starting index of the string being copied can also be specified.

  Arguments:

  str -  The string which is having a portion copied.
  str_start -  The index of the string to start at. (Less than str's length).
  buf -  The buffer to store the copied chunk of the string into.
  buf_len -  The length of the buffer.
  sep -  String containing characters that will be used as separators.

  Preconditions:  Make sure str_start does *NOT* exceed the size of str.

  Postconditions: Stores the resulting string in buf, and returns the
  position where reading was left off at.  The position returned will skip
  separation characters.

  */
  int16_t string_read_field(const char *str, uint16_t str_start,
    char *field, uint16_t field_size, const char *sep) {

      // Want to read from the string until we encounter the separator.

      // Character that we are reading from the string.
      uint16_t str_index = str_start;

      while (1) {
        if ( str[str_index] == '\0') {
          str_index++;  // signal off end of str
          break;
        }

        if ( field_size <= 1 ) break;

        if (strchr(sep, str[str_index])) {
          // field finished, skip over the separator character.
          str_index++;
          break;
        }

        // Copy the string character into buffer and move over to next
        *field = str[str_index];
        field++;
        field_size--;
        // Move on to the next character.
        str_index++;
      }

      // Make sure to add NULL termination to our new string.
      *field = '\0';

      // Return the index of where the next token begins.
      return str_index;
    }

    /*
    Function to convert a string to an int32_t.

    Arguments:
    str -  The string to convert to an integer.

    Preconditions:  The string should probably represent an integer value.

    Postconditions:
    Will return the equivalent integer. If an error occured the
    blink assert may be triggered, and sometimes zero is just
    returned from this function because EINVAL does not exist for
    the Arduino strtol.

    */

    int32_t string_get_int(const char *str) {
      // Attempt to convert the string to an integer using strtol...
      int32_t val = strtol(str, NULL, 10);

      if (val == 0) {
        // Must check errno for possible error.
        if (errno == ERANGE) {
          dprintf("ERROR string_get_int failed with %d on '%s'",
          errno, str);
          assert13(0, errno);
        }
      }

      return val;
    }
