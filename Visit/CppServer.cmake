if(NOT TARGET cppserver)

  # Module flag
  set(CPPSERVER_MODULE Y)

  # Module subdirectory
  add_subdirectory("CppServer")

  # Module folder
  set_target_properties(cppserver PROPERTIES FOLDER CppServer)

endif()

