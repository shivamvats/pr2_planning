FILE(REMOVE_RECURSE
  "CMakeFiles/fcl"
  "CMakeFiles/fcl-complete"
  "fcl-prefix/src/fcl-stamp/fcl-install"
  "fcl-prefix/src/fcl-stamp/fcl-mkdir"
  "fcl-prefix/src/fcl-stamp/fcl-download"
  "fcl-prefix/src/fcl-stamp/fcl-update"
  "fcl-prefix/src/fcl-stamp/fcl-patch"
  "fcl-prefix/src/fcl-stamp/fcl-configure"
  "fcl-prefix/src/fcl-stamp/fcl-build"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/fcl.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
