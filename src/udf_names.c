/* This file generated automatically. */
/*          Do not modify.            */
#include <math.h>
#include "udf.h"
#include "prop.h"
#include "dpm.h"
extern DEFINE_ON_DEMAND(Mode_calculation);
extern DEFINE_ON_DEMAND(Preprocess);
extern DEFINE_EXECUTE_AFTER_DATA(AutoPreprocess, libudf);
extern DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dTime);
extern DEFINE_EXECUTE_AT_END(Set_next_time_step);
extern DEFINE_ON_DEMAND(Clear_memories);
extern DEFINE_EXECUTE_AT_EXIT(Finish_process);
extern DEFINE_PROFILE(Velocity_inlet, thread, iVar);
 __declspec(dllexport) UDF_Data udf_data[] = {
{"Mode_calculation", (void (*)(void))Mode_calculation, UDF_TYPE_ON_DEMAND},
{"Preprocess", (void (*)(void))Preprocess, UDF_TYPE_ON_DEMAND},
{"AutoPreprocess", (void (*)(void))AutoPreprocess, UDF_TYPE_EXECUTE_AFTER_DATA},
{"FDM_method", (void (*)(void))FDM_method, UDF_TYPE_GRID_MOTION},
{"Set_next_time_step", (void (*)(void))Set_next_time_step, UDF_TYPE_EXECUTE_AT_END},
{"Clear_memories", (void (*)(void))Clear_memories, UDF_TYPE_ON_DEMAND},
{"Finish_process", (void (*)(void))Finish_process, UDF_TYPE_EXECUTE_AT_EXIT},
{"Velocity_inlet", (void (*)(void))Velocity_inlet, UDF_TYPE_PROFILE},
};
__declspec(dllexport) int n_udf_data = sizeof(udf_data)/sizeof(UDF_Data);
#include "version.h"
__declspec(dllexport) void UDF_Inquire_Release(int *major, int *minor, int *revision)
{
	*major = RampantReleaseMajor;
	*minor = RampantReleaseMinor;
	*revision = RampantReleaseRevision;
}