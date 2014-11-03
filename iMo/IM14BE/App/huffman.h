/*
 * Huffman coding used by ECG to compress raw data
 * Header file for type definitions
 */
#ifndef __HUFFMAN__

// A Huffman tree node
struct MinHeapNode
{
    uint8 data;  // One of the input characters
    uint16 freq;  // Frequency of the character
    struct MinHeapNode *left, *right; // Left and right child of this node
};

// A Min Heap:  Collection of min heap (or Hufmman tree) nodes
struct MinHeap
{
    uint16 size;    // Current size of min heap
    uint16 capacity;   // capacity of min heap
    struct MinHeapNode **array;  // Attay of minheap node pointers
};

struct huffman_mapping
{
    uint8 bits;
    union {
        uint8 *b_ptr;
        uint8 b[2];
    };
};

void huffmanInit();
void huffmanSample(int16 s);
uint8 huffman();

#endif
