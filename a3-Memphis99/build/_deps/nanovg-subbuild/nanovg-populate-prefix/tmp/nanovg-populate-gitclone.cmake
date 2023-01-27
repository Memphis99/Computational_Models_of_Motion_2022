
if(NOT "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitinfo.txt" IS_NEWER_THAN "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitclone-lastrun.txt")
  message(STATUS "Avoiding repeated git clone, stamp file is up to date: 'C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitclone-lastrun.txt'")
  return()
endif()

execute_process(
  COMMAND ${CMAKE_COMMAND} -E rm -rf "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to remove directory: 'C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-src'")
endif()

# try the clone 3 times in case there is an odd git clone issue
set(error_code 1)
set(number_of_tries 0)
while(error_code AND number_of_tries LESS 3)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  clone --no-checkout --config "advice.detachedHead=false" "https://github.com/memononen/nanovg" "nanovg-src"
    WORKING_DIRECTORY "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps"
    RESULT_VARIABLE error_code
    )
  math(EXPR number_of_tries "${number_of_tries} + 1")
endwhile()
if(number_of_tries GREATER 1)
  message(STATUS "Had to git clone more than once:
          ${number_of_tries} times.")
endif()
if(error_code)
  message(FATAL_ERROR "Failed to clone repository: 'https://github.com/memononen/nanovg'")
endif()

execute_process(
  COMMAND "C:/Program Files/Git/cmd/git.exe"  checkout 077b65e0cf3e22ee4f588783e319b19b0a608065 --
  WORKING_DIRECTORY "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-src"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to checkout tag: '077b65e0cf3e22ee4f588783e319b19b0a608065'")
endif()

set(init_submodules TRUE)
if(init_submodules)
  execute_process(
    COMMAND "C:/Program Files/Git/cmd/git.exe"  submodule update --recursive --init 
    WORKING_DIRECTORY "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-src"
    RESULT_VARIABLE error_code
    )
endif()
if(error_code)
  message(FATAL_ERROR "Failed to update submodules in: 'C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-src'")
endif()

# Complete success, update the script-last-run stamp file:
#
execute_process(
  COMMAND ${CMAKE_COMMAND} -E copy
    "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitinfo.txt"
    "C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitclone-lastrun.txt"
  RESULT_VARIABLE error_code
  )
if(error_code)
  message(FATAL_ERROR "Failed to copy script-last-run stamp file: 'C:/Users/DELL/Desktop/University/CMM/a3-Memphis99/build/_deps/nanovg-subbuild/nanovg-populate-prefix/src/nanovg-populate-stamp/nanovg-populate-gitclone-lastrun.txt'")
endif()

