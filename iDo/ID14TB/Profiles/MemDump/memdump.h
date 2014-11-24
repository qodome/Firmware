/*
 * Memory dump service for iDo debugging
 */
#ifndef __MEM_DUMP__
#define __MEM_DUMP__

#define MEMDUMP_SERVICE             0x5555
#define MEMDUMP_BYTES               0x6666

bStatus_t MemDump_AddService(void);
bStatus_t MemDump_DelService(void);
uint8 MemDump_ServiceNeedDelete(void);

#endif