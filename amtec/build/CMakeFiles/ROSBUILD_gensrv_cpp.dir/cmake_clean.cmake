FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/amtec/SetPosition.h"
  "../srv_gen/cpp/include/amtec/SweepPan.h"
  "../srv_gen/cpp/include/amtec/Halt.h"
  "../srv_gen/cpp/include/amtec/SetVelocity.h"
  "../srv_gen/cpp/include/amtec/TargetAcceleration.h"
  "../srv_gen/cpp/include/amtec/GetStatus.h"
  "../srv_gen/cpp/include/amtec/SetPositionVelocity.h"
  "../srv_gen/cpp/include/amtec/Home.h"
  "../srv_gen/cpp/include/amtec/AddTwoInts.h"
  "../srv_gen/cpp/include/amtec/TargetVelocity.h"
  "../srv_gen/cpp/include/amtec/Reset.h"
  "../srv_gen/cpp/include/amtec/SweepTilt.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
