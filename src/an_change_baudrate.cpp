#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "spatial_packets.h"


int main(int argc, char **argv)
{
  if (argc != 4)
  {
    printf("Usage - program com_port old_baud_rate new_baud_rate\nExample - ./an_change_baudrate /dev/ttyUSB0 115200 912600\n");
    exit(EXIT_FAILURE);
  }
  
  char *com_port = argv[1];
  int old_baudrate = atoi(argv[2]);
  int new_baudrate = atoi(argv[3]);

  /* open the com port */
  if (OpenComport(com_port, old_baudrate))
  {
    printf("Could not open serial port\n");
    exit(EXIT_FAILURE);
  }

  printf("Connected to device with %s %d\n", com_port, old_baudrate);

  an_packet_t *an_packet;
  baud_rates_packet_t baud_rate_packet;

  baud_rate_packet.permanent = TRUE;
  baud_rate_packet.primary_baud_rate = new_baudrate;
  baud_rate_packet.gpio_1_2_baud_rate = new_baudrate;
  baud_rate_packet.auxiliary_baud_rate = new_baudrate;
  baud_rate_packet.reserved = 0;

  an_packet = encode_baud_rates_packet(&baud_rate_packet);
  an_packet_encode(an_packet);
  SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
  an_packet_free(&an_packet);
  
  printf("Changed baudrate from %d to %d\nTry reconnecting...\n", old_baudrate, new_baudrate);
  CloseComport();
  if (OpenComport(com_port, new_baudrate))
  {
    printf("Could not open serial port\n");
    exit(EXIT_FAILURE);
  }
  printf("Everything seems okay. This doesn't mean that the baudrate was changed properly,");
  printf(" check by running the driver and see if the advanced navigation topics are published.'\n");
  return 0;
}
