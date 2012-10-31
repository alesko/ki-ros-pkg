FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/fastrak/srv/__init__.py"
  "../src/fastrak/srv/_StartPublishing.py"
  "../src/fastrak/srv/_GetPose.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
