/*
 * Huffman coding used by ECG to compress raw data
 */
#include "OSAL_Memory.h"
#include "hal_assert.h"
#include "OSAL.h"
#include "huffman.h"

#define MAC_CODE_ARRAY_BYTES    64
#define HUFFMAN_STEPS           256
#define HUFFMAN_STEP_MASK       0xFF

struct huffman_mapping hmap[HUFFMAN_STEPS];
int16 hLastSample = 0;
uint16 hMonitorThresholdCount = 0;
uint16 hSamples[HUFFMAN_STEPS] = {0};
uint16 hTotalSamples = 0;
uint8 hDone = 0;
uint8 hMonitorDone = 0;
uint16 hError = 0;
int8 hMax = -128;
int8 hMin = 127;

// A utility function allocate a new min heap node with given character
// and frequency of the character
struct MinHeapNode* newNode(uint8 data, uint16 freq)
{
    struct MinHeapNode* temp =
        (struct MinHeapNode*)osal_mem_alloc(sizeof(struct MinHeapNode));
    if (temp == NULL) {
        hError |= (1 << 0);
        return NULL;
    }
    temp->left = temp->right = NULL;
    temp->data = data;
    temp->freq = freq;
    return temp;
}

// A utility function to create a min heap of given capacity
struct MinHeap* createMinHeap(uint16 capacity)
{
    struct MinHeap* minHeap =
        (struct MinHeap*)osal_mem_alloc(sizeof(struct MinHeap));
    if (minHeap == NULL) {
        hError |= (1 << 1);
        return NULL;
    }
    minHeap->size = 0;  // current size is 0
    minHeap->capacity = capacity;
    minHeap->array =
        (struct MinHeapNode**)osal_mem_alloc(minHeap->capacity * sizeof(struct MinHeapNode*));
    if (minHeap->array == NULL) {
        hError |= (1 << 2);
        return NULL;
    }
    return minHeap;
}

// A utility function to swap two min heap nodes
void swapMinHeapNode(struct MinHeapNode** a, struct MinHeapNode** b)
{
    struct MinHeapNode* t = *a;
    *a = *b;
    *b = t;
}

// The standard minHeapify function.
void minHeapify(struct MinHeap* minHeap, uint16 idx)
{
    int smallest = idx;
    int left = 2 * idx + 1;
    int right = 2 * idx + 2;

    if (left < minHeap->size &&
            minHeap->array[left]->freq < minHeap->array[smallest]->freq)
        smallest = left;

    if (right < minHeap->size &&
            minHeap->array[right]->freq < minHeap->array[smallest]->freq)
        smallest = right;

    if (smallest != idx)
    {
        swapMinHeapNode(&minHeap->array[smallest], &minHeap->array[idx]);
        minHeapify(minHeap, smallest);
    }
}

// A utility function to check if size of heap is 1 or not
int isSizeOne(struct MinHeap* minHeap)
{
    return (minHeap->size == 1);
}

// A standard function to extract minimum value node from heap
struct MinHeapNode* extractMin(struct MinHeap* minHeap)
{
    struct MinHeapNode* temp = minHeap->array[0];
    minHeap->array[0] = minHeap->array[minHeap->size - 1];
    minHeap->array[minHeap->size - 1] = NULL;
    --minHeap->size;
    minHeapify(minHeap, 0);
    return temp;
}

// A utility function to insert a new node to Min Heap
void insertMinHeap(struct MinHeap* minHeap, struct MinHeapNode* minHeapNode)
{
    ++minHeap->size;
    int i = minHeap->size - 1;
    while (i && minHeapNode->freq < minHeap->array[(i - 1)/2]->freq)
    {
        minHeap->array[i] = minHeap->array[(i - 1)/2];
        i = (i - 1)/2;
    }
    minHeap->array[i] = minHeapNode;
}

// A standard funvtion to build min heap
void buildMinHeap(struct MinHeap* minHeap)
{
    int n = minHeap->size - 1;
    int i;
    for (i = (n - 1) / 2; i >= 0; --i)
        minHeapify(minHeap, i);
}

// Utility function to check if this node is leaf
int isLeaf(struct MinHeapNode* root)
{
    return !(root->left) && !(root->right) ;
}

// Creates a min heap of capacity equal to size and inserts all character of
// data[] in min heap. Initially size of min heap is equal to capacity
struct MinHeap* createAndBuildMinHeap(uint16 freq[], uint16 size)
{
    struct MinHeap* minHeap = createMinHeap(size);
    if (minHeap == NULL) {
        hError |= (1 << 3);
        return NULL;
    }
    for (uint16 i = 0; i < size; ++i) {
        minHeap->array[i] = newNode((uint8)(i & 0xFF), freq[i]);
        if (minHeap->array[i] == NULL) {
            hError |= (1 << 4);
            return NULL;
        }
    }
    minHeap->size = size;
    buildMinHeap(minHeap);
    return minHeap;
}

// The main function that builds Huffman tree
struct MinHeapNode* buildHuffmanTree(uint16 freq[])
{
    struct MinHeapNode *left, *right, *top, *ret_node;

    // Step 1: Create a min heap of capacity equal to size.  Initially, there are
    // modes equal to size.
    struct MinHeap* minHeap = createAndBuildMinHeap(freq, HUFFMAN_STEPS);
    if (minHeap == NULL) {
        hError |= (1 << 5);
        return NULL;
    }

    // Iterate while size of heap doesn't become 1
    while (!isSizeOne(minHeap))
    {
        // Step 2: Extract the two minimum freq items from min heap
        left = extractMin(minHeap);
        right = extractMin(minHeap);

        // Step 3:  Create a new internal node with frequency equal to the
        // sum of the two nodes frequencies. Make the two extracted node as
        // left and right children of this new node. Add this node to the min heap
        // '$' is a special value for internal nodes, not used
        top = newNode('$', left->freq + right->freq);
        if (top == NULL) {
            hError |= (1 << 6);
            return NULL;
        }
        top->left = left;
        top->right = right;
        insertMinHeap(minHeap, top);
    }

    // Step 4: The remaining node is the root node and the tree is complete.
    ret_node = extractMin(minHeap);
    osal_mem_free(minHeap);

    return ret_node;
}

void freeHuffmanTree(struct MinHeapNode *root)
{
    if (root != NULL) {
        freeHuffmanTree(root->left);
        root->left = NULL;
        freeHuffmanTree(root->right);
        root->right = NULL;
        osal_mem_free(root);
    }    
}

void fillHuffmanCode(uint8 *c_ptr, uint16 c_idx, uint8 v)
{
    uint16 B_idx = 0, b_idx = 0;

    B_idx = c_idx / 8;
    if (B_idx >= MAC_CODE_ARRAY_BYTES) {
        hError |= (1 << 9);
        B_idx = MAC_CODE_ARRAY_BYTES - 1;
    }
    b_idx = c_idx % 8;

    if (v == 0) {
        c_ptr[B_idx] &= ~(1 << b_idx);
    } else {
        c_ptr[B_idx] |= (1 << b_idx);
    }
}

int assignHuffmanCode(struct MinHeapNode* root, uint8 *c_ptr, uint16 c_idx)
{
    if (root->left)
    {
        fillHuffmanCode(c_ptr, c_idx, 0);
        if (assignHuffmanCode(root->left, c_ptr, c_idx + 1) == 1) {
            return 1;
        }
    }

    if (root->right)
    {
        fillHuffmanCode(c_ptr, c_idx, 1);
        if (assignHuffmanCode(root->right, c_ptr, c_idx + 1) == 1) {
            return 1;
        }
    }

    // If this is a leaf node, then it contains one of the final code 
    if (isLeaf(root)) {
        hmap[root->data].bits = c_idx;
        hmap[root->data].b_ptr = osal_mem_alloc((c_idx + 7) / 8);
        if (hmap[root->data].b_ptr == NULL) {
            hError |= (1 << 7);
            return 1;
        }
        osal_memcpy(hmap[root->data].b_ptr, c_ptr, ((c_idx + 7) / 8));
    }
    return 0;
}

int genHuffmanMapping(struct MinHeapNode* root)
{
    uint8 code_array[MAC_CODE_ARRAY_BYTES] = {0};
    uint16 i = 0;

    for (i = 0; i < MAC_CODE_ARRAY_BYTES; i++) {
        code_array[i] = 0;
    }

    return assignHuffmanCode(root, &(code_array[0]), 0);
}

void huffmanInit()
{
    uint16 idx = 0;
    hLastSample = 0;
    hMonitorThresholdCount = 0;
    hTotalSamples = 0;
    hDone = 0;
    hMonitorDone = 0;

    for (idx = 0; idx < HUFFMAN_STEPS; idx++) {
        hSamples[idx] = 0;
    }
}

uint8 huffman()
{
    struct MinHeapNode* root = NULL;
    
    if (hDone == 1) {
        /*
        root = buildHuffmanTree(hSamples);
        if (root == NULL) {
            hError |= (1 << 8);
            return 1;
        }
        */
        /*
        if (genHuffmanMapping(root) == 0) {
            freeHuffmanTree(root);
        }
        */

        return 1;
    } else {
        return 0;
    }
}

void huffmanSample(int16 s)
{
    int16 delta;
    int8 delta8;
    uint8 idx;

    if (hDone == 0) {
        delta = s - hLastSample;
        if (hMonitorDone == 0) {

            if (delta > 126 || delta < -127) {
                hMonitorThresholdCount = 0;
            } else {
                hMonitorThresholdCount++;
                if (hMonitorThresholdCount > 5000) {
                    hMonitorDone = 1;
                }
            }
        } else {
            idx = (uint8)(delta & HUFFMAN_STEP_MASK);
            hSamples[idx]++;
            hTotalSamples++;
            // Take the 1 minute sample to do huffman
            if (hTotalSamples > 30000) {
                hDone = 1;
            }
            delta8 = (int8)(delta & 0xFF);
            if (delta8 > hMax) {
                hMax = delta8;
            }
            if (delta8 < hMin) {
                hMin = delta8;
            }
        }
        hLastSample = s;
    }
}
