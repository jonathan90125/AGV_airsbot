#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <modbus.h>

#include "unit-test.h"

const int EXCEPTION_RC = 2;


uint16_t UT_BITS_ADDRESS = 0x100;
uint16_t UT_BITS_NB = 0x1;
uint8_t UT_BITS_TAB[] = { 0x01 };
 

uint16_t UT_INPUT_BITS_ADDRESS = 0x200;
uint16_t UT_INPUT_BITS_NB = 0x1;
uint8_t UT_INPUT_BITS_TAB[] = { 0x01 };

uint16_t UT_REGISTERS_ADDRESS = 0x300;
uint16_t UT_REGISTERS_NB = 0xD;
uint16_t UT_REGISTERS_NB_MAX = 0x400;
uint16_t UT_REGISTERS_TAB[] = { 0x0000,0x0002,0x0000,0x0000,                 //flags
                                      0x0012,0x0000,0x0000,0x0000,0x0000,0x0000, //twist
                                      -0x0064,0x0000,0x0000,                     //goal pose 
                                     };

uint16_t UT_INPUT_REGISTERS_ADDRESS = 0x400;
uint16_t UT_INPUT_REGISTERS_NB = 0x12;
uint16_t UT_INPUT_REGISTERS_TAB[] = { 0x0000,0x0000,0x0001,0x0000,0x0000,0x0001, //flags
                                      0x0032,0x0000,0x0000,0x0000,0x0000,0x0000, //twist
                                      0x0010,0x00010,0x0000,                     //current pose
                                      0x0000,0x0000,0x0000,                     //goal pose
                                      };
 

int test_server(modbus_t *ctx, int use_backend);
int send_crafted_request(modbus_t *ctx, int function,
                         uint8_t *req, int req_size,
                         uint16_t max_value, uint16_t bytes,
                         int backend_length, int backend_offset);
int equal_dword(uint16_t *tab_reg, const uint32_t value);
 

int equal_dword(uint16_t *tab_reg, const uint32_t value) {
    return ((tab_reg[0] == (value >> 16)) && (tab_reg[1] == (value & 0xFFFF)));
}

int main(int argc, char *argv[])
{
    const int NB_REPORT_SLAVE_ID = 10;
    uint8_t *tab_rp_bits = NULL;
    uint16_t *tab_rp_registers = NULL;
    uint16_t *tab_rp_registers_bad = NULL;
    modbus_t *ctx = NULL;
    int i;
    uint8_t value;
    int nb_points;
    int rc;
    uint32_t old_response_to_sec;
    uint32_t old_response_to_usec;
    uint32_t new_response_to_sec;
    uint32_t new_response_to_usec;
    uint32_t old_byte_to_sec;
    uint32_t old_byte_to_usec;  
    int success = FALSE;
   
 
  
    ctx = modbus_new_tcp("127.0.0.1", 1502);
 
    if (ctx == NULL) {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
        return -1;
    }
    modbus_set_debug(ctx, TRUE);
    modbus_set_error_recovery(ctx,
                              MODBUS_ERROR_RECOVERY_LINK |
                              MODBUS_ERROR_RECOVERY_PROTOCOL);

     
    modbus_get_response_timeout(ctx, &old_response_to_sec, &old_response_to_usec);
    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Connection failed: %s\n", modbus_strerror(errno));
        modbus_free(ctx);
        return -1;
    }
    modbus_get_response_timeout(ctx, &new_response_to_sec, &new_response_to_usec);

    printf("** UNIT TESTING **\n");
 

    /* Allocate and initialize the memory to store the bits */
    nb_points = (UT_BITS_NB > UT_INPUT_BITS_NB) ? UT_BITS_NB : UT_INPUT_BITS_NB;
    tab_rp_bits = (uint8_t *) malloc(nb_points * sizeof(uint8_t));
    memset(tab_rp_bits, 0, nb_points * sizeof(uint8_t));

    /* Allocate and initialize the memory to store the registers */
    nb_points = (UT_REGISTERS_NB > UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    tab_rp_registers = (uint16_t *) malloc(nb_points * sizeof(uint16_t));
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));

    printf("\nTEST WRITE/READ:\n");

    /** COIL BITS **/

    /* Single */
    rc = modbus_write_bit(ctx, UT_BITS_ADDRESS, ON);
  

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, 1, tab_rp_bits);
   
 

    /* Multiple bits */
    {
        uint8_t tab_value[UT_BITS_NB];

        modbus_set_bits_from_bytes(tab_value, 0, UT_BITS_NB, UT_BITS_TAB);
        rc = modbus_write_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_value);
       
    }

    rc = modbus_read_bits(ctx, UT_BITS_ADDRESS, UT_BITS_NB, tab_rp_bits);
 
    i = 0;
    nb_points = UT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;

        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
        
        nb_points -= nb_bits;
        i++;
    }
  
 

    /** DISCRETE INPUTS **/
    rc = modbus_read_input_bits(ctx, UT_INPUT_BITS_ADDRESS,
                                UT_INPUT_BITS_NB, tab_rp_bits);
     

    i = 0;
    nb_points = UT_INPUT_BITS_NB;
    while (nb_points > 0) {
        int nb_bits = (nb_points > 8) ? 8 : nb_points;
        value = modbus_get_byte_from_bits(tab_rp_bits, i*8, nb_bits);
    
        nb_points -= nb_bits;
        i++;
    }
 
  

    /* Many registers */
    rc = modbus_write_registers(ctx, UT_REGISTERS_ADDRESS,
                                UT_REGISTERS_NB, UT_REGISTERS_TAB);

    rc = modbus_read_registers(ctx, UT_REGISTERS_ADDRESS,
                               UT_REGISTERS_NB, tab_rp_registers);
   
  
    nb_points = (UT_REGISTERS_NB >
                 UT_INPUT_REGISTERS_NB) ?
        UT_REGISTERS_NB : UT_INPUT_REGISTERS_NB;
    memset(tab_rp_registers, 0, nb_points * sizeof(uint16_t));
 

    modbus_close(ctx);
    modbus_free(ctx);
    ctx = NULL;
 
    printf("\nALL TESTS PASS WITH SUCCESS.\n");
    success = TRUE;

close:
    /* Free the memory */
    free(tab_rp_bits);
    free(tab_rp_registers);

    /* Close the connection */
    modbus_close(ctx);
    modbus_free(ctx);

    return (success) ? 0 : -1;
}

 
