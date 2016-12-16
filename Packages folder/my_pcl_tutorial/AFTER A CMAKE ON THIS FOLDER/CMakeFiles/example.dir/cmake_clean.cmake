FILE(REMOVE_RECURSE
  "CMakeFiles/example.dir/src/example.cpp.o"
  "devel/lib/my_pcl_tutorial/example.pdb"
  "devel/lib/my_pcl_tutorial/example"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/example.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
