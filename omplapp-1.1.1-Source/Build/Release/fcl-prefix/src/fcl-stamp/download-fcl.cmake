message(STATUS "downloading...
     src='http://downloads.sourceforge.net/project/ompl/dependencies/fcl-0.3.1.zip'
     dst='/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip'
     timeout='none'")

file(DOWNLOAD
  "http://downloads.sourceforge.net/project/ompl/dependencies/fcl-0.3.1.zip"
  "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/src/external/fcl-0.3.1.zip"
  SHOW_PROGRESS
  EXPECTED_MD5;3b36420e54998c674b8dba3a5bbaf3f6
  # no TIMEOUT
  STATUS status
  LOG log)

list(GET status 0 status_code)
list(GET status 1 status_string)

if(NOT status_code EQUAL 0)
  message(FATAL_ERROR "error: downloading 'http://downloads.sourceforge.net/project/ompl/dependencies/fcl-0.3.1.zip' failed
  status_code: ${status_code}
  status_string: ${status_string}
  log: ${log}
")
endif()

message(STATUS "downloading... done")
