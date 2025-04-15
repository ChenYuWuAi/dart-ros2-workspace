
#ifndef _DebugVisualdata_H
#define _DebugVisualdata_H

#define DebugVisualdataSize (4096)
typedef struct
{
    float data[DebugVisualdataSize];
    int count;
} DebugVisualdata_t;

void AddDebugVisual(DebugVisualdata_t * DebugVisualdata,float point);

#endif
