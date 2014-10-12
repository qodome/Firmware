/*
 * Define Huffman code book and encoding functions
 */

#ifndef __HUFFMAN_ENCODE__
#define __HUFFMAN_ENCODE__

struct huffman_code_info {
    uint8 nbit;
    uint8 low;
    uint8 high;
};

uint8 huffman_get_code_nbits(uint8 idx);
uint16 huffman_get_code(uint8 idx);

#endif
