# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-src")
  file(MAKE_DIRECTORY "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-src")
endif()
file(MAKE_DIRECTORY
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-build"
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix"
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/tmp"
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/src/implot-populate-stamp"
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/src"
  "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/src/implot-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/src/implot-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/rohannair/Desktop/Roboracing/roboracing-micromouse/tools/analyzer-ui/build/_deps/implot-subbuild/implot-populate-prefix/src/implot-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
