message(STATUS "downloading...
     src='https://github.com/danfis/libccd/archive/v2.0.tar.gz'
     dst='/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/v2.0.tar.gz'
     timeout='none'")

file(DOWNLOAD
  "https://github.com/danfis/libccd/archive/v2.0.tar.gz"
  "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/v2.0.tar.gz"
  SHOW_PROGRESS
  EXPECTED_MD5;ee45fc60980d76f25cad3a0eda8bd9aa
  # no TIMEOUT
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'https://github.com/danfis/libccd/archive/v2.0.tar.gz' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
