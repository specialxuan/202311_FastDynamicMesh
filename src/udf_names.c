/* This file generated automatically. */
/*          Do not modify.            */
#include <math.h>
#include "udf.h"
#include "prop.h"
#include "dpm.h"
extern DEFINE_ON_DEMAND(ModeCalculation);
extern DEFINE_ON_DEMAND(Preprocess);
 __declspec(dllexport) UDF_Data udf_data[] = {
{"ModeCalculation", (void (*)(void))ModeCalculation, UDF_TYPE_ON_DEMAND},
{"Preprocess", (void (*)(void))Preprocess, UDF_TYPE_ON_DEMAND},
};
__declspec(dllexport) int n_udf_data = sizeof(udf_data)/sizeof(UDF_Data);
#include "version.h"
__declspec(dllexport) void UDF_Inquire_Release(int *major, int *minor, int *revision)
{
	*major = RampantReleaseMajor;
	*minor = RampantReleaseMinor;
	*revision = RampantReleaseRevision;
}