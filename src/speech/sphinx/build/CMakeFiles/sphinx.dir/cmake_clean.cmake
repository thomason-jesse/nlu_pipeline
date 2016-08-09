FILE(REMOVE_RECURSE
  "CMakeFiles/sphinx.dir/src/sphinx.c.o"
  "libsphinx.pdb"
  "libsphinx.so"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang C)
  INCLUDE(CMakeFiles/sphinx.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
