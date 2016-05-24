# CMake generated Testfile for 
# Source directory: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/ompl/tests
# Build directory: /home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/ompl/tests
# 
# This file includes the relevent testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
ADD_TEST(test_heap "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_heap")
ADD_TEST(test_grid "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_grid")
ADD_TEST(test_gridb "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_gridb")
ADD_TEST(test_nearestneighbors "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_nearestneighbors")
ADD_TEST(test_pdf "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_pdf")
ADD_TEST(test_random "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_random")
ADD_TEST(test_machine_specs "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_machine_specs")
ADD_TEST(test_state_operations "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_state_operations")
ADD_TEST(test_state_spaces "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_state_spaces")
ADD_TEST(test_state_storage "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_state_storage")
ADD_TEST(test_ptc "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_ptc")
ADD_TEST(test_planner_data "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_planner_data")
ADD_TEST(test_2denvs_geometric "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_2denvs_geometric")
ADD_TEST(test_2dmap_geometric_simple "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_2dmap_geometric_simple")
ADD_TEST(test_2dmap_ik "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_2dmap_ik")
ADD_TEST(test_2dcircles_opt_geometric "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_2dcircles_opt_geometric")
ADD_TEST(test_2dmap_control "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_2dmap_control")
ADD_TEST(test_planner_data_control "/home/osboxes/groovy_ws2/pr2_planning/omplapp-1.1.1-Source/Build/Release/bin/test_planner_data_control")
SUBDIRS(regression_tests)
