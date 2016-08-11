cmake_minimum_required(VERSION 2.8)

project(sphinx)
set(CMAKE_BUILD_TYPE Release)

include_directories(/scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/sphinx_builds/sphinxbase_install/include)
include_directories(/scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/sphinx_builds/sphinxbase_install/include/sphinxbase)
include_directories(/scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/sphinx_builds/pocketsphinx_install/include/pocketsphinx)

add_library(sphinx SHARED /scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/src/sphinx.c)

target_link_libraries(sphinx /scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/sphinx_builds/sphinxbase_install/lib/libsphinxbase.so)
target_link_libraries(sphinx /scratch/cluster/rcorona/nlu_pipeline/src/speech/sphinx/sphinx_builds/pocketsphinx_install/lib/libpocketsphinx.so)
