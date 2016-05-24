message(STATUS "verifying file...
     file='/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip'")

set(verified 0)

# If an expected md5 checksum exists, compare against it:
#
if(NOT "3b36420e54998c674b8dba3a5bbaf3f6" STREQUAL "")
  execute_process(COMMAND ${CMAKE_COMMAND} -E md5sum "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip"
    OUTPUT_VARIABLE ov
    OUTPUT_STRIP_TRAILING_WHITESPACE
    RESULT_VARIABLE rv)

  if(NOT rv EQUAL 0)
    message(FATAL_ERROR "error: computing md5sum of '/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip' failed")
  endif()

  string(REGEX MATCH "^([0-9A-Fa-f]+)" md5_actual "${ov}")

  string(TOLOWER "${md5_actual}" md5_actual)
  string(TOLOWER "3b36420e54998c674b8dba3a5bbaf3f6" md5)

  if(NOT "${md5}" STREQUAL "${md5_actual}")
    message(FATAL_ERROR "error: md5sum of '/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip' does not match expected value
  md5_expected: ${md5}
    md5_actual: ${md5_actual}
")
  endif()

  set(verified 1)
endif()

if(verified)
  message(STATUS "verifying file... done")
else()
  message(STATUS "verifying file... warning: did not verify file - no URL_MD5 checksum argument? corrupt file?")
endif()
