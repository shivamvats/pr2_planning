FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/arm_navigation_msgs/msg"
  "../src/arm_navigation_msgs/srv"
  "CMakeFiles/tests"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/tests.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
