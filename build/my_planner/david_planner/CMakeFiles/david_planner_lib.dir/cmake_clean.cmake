file(REMOVE_RECURSE
  "libdavid_planner_lib.a"
  "libdavid_planner_lib.pdb"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/david_planner_lib.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
