# 生成物统一放到构建目录，避免配置时污染源码树
set(UDF_NAMES_DIR "${CMAKE_CURRENT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${UDF_NAMES_DIR}")

set(UDF_SOURCE_PATHS "")
foreach(CFILE ${CSOURCES})
    list(APPEND UDF_SOURCE_PATHS "${CMAKE_CURRENT_SOURCE_DIR}/${CFILE}")
    file(STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/${CFILE}" SOURCE_STR)
    foreach(LINE ${SOURCE_STR})
        if(LINE MATCHES "^ *DEFINE_[_A-Z]*\\(.*\\)")
            string(REGEX REPLACE "^ *(DEFINE_[_A-Z]*\\(.*\\))"
                "extern \\1;\n" F_DEF ${LINE})
            list(APPEND FUNCTION_DEFS ${F_DEF})
            string(REGEX REPLACE "^ *DEFINE_([_A-Z]*)\\( *([_a-zA-Z0-9]*).*\\)"
                "{\"\\2\", (void (*)(void))\\2, UDF_TYPE_\\1},\n" F_ARR ${LINE})
            list(APPEND FUNCTION_ARRS ${F_ARR})
        endif()
    endforeach(LINE)
endforeach(CFILE)

message("FUNCTION_DEFS: ${FUNCTION_DEFS}")
message("FUNCTION_ARRS: ${FUNCTION_ARRS}")

string(REPLACE "\n" "\;\n" FUNCTION_DEFS ${FUNCTION_DEFS})
string(REPLACE "\n" "\n" FUNCTION_ARRS ${FUNCTION_ARRS})

set(UDFNAMES_CONTENT
    "/* This file generated automatically. */\n"
    "/*          Do not modify.            */\n"
    "#include <math.h>\n"
    "#include \"udf.h\"\n"
    "#include \"prop.h\"\n"
    "#include \"dpm.h\"\n"
    "${FUNCTION_DEFS} ;"
    "__declspec(dllexport) UDF_Data udf_data[] = {\n"
    "${FUNCTION_ARRS}"
    "}\;\n"
    "__declspec(dllexport) int n_udf_data = sizeof(udf_data)/sizeof(UDF_Data)\;\n"
    "#include \"version.h\"\n"
    "__declspec(dllexport) void UDF_Inquire_Release(int *major, int *minor, int *revision)\n"
    "{\n"
    "\t*major = RampantReleaseMajor\;\n"
    "\t*minor = RampantReleaseMinor\;\n"
    "\t*revision = RampantReleaseRevision\;\n"
    "}"
)

file(WRITE "${UDF_NAMES_DIR}/udf_names.c" ${UDFNAMES_CONTENT})

execute_process(
    COMMAND ${TOOL_PATH}/resolve.exe -udf ${UDF_SOURCE_PATHS} -head_file ud_io1.h
    WORKING_DIRECTORY "${UDF_NAMES_DIR}"
    RESULT_VARIABLE UDF_RESOLVE_RESULT
)
if(NOT UDF_RESOLVE_RESULT STREQUAL "0")
    message(WARNING
        "resolve.exe failed (result: ${UDF_RESOLVE_RESULT}); ud_io1.h was not generated. "
        "Check that ${TOOL_PATH}/resolve.exe exists and that FLUENT_ROOT/FLUENT_ARCH are correct.")
endif()
