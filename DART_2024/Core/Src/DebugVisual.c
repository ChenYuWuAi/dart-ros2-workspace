//BY:gongxd@126.com

#include  <stdlib.h>
#include  <stdio.h>
#include  <string.h>
#include  <math.h>
#include  "DebugVisual.h"




void AddDebugVisual(DebugVisualdata_t * DebugVisualdata,float point)
{
    if(DebugVisualdata->count>=DebugVisualdataSize)
    {
        memset(DebugVisualdata->data,0x00,DebugVisualdataSize*sizeof(float));
        DebugVisualdata->count=0;
    }
		
    DebugVisualdata->data[DebugVisualdata->count]=point;
    DebugVisualdata->count++;
}
